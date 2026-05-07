#!/usr/bin/env python3
"""Passive PDA08 CAN sniffer for OpenExo bring-up.

This tool does not transmit. Put the USB-CAN adapter in listen-only/silent mode
when the adapter supports it, then start an OpenExo trial and watch for:
  - 0x03F write_property frames from Teensy
  - 0x03D set_torque frames from Teensy
  - PDA feedback frames with pda_id == 1
"""

import argparse
import struct
import time

try:
    import can
except ImportError as exc:
    raise SystemExit(
        "python-can is required. Install it in the GUI Python environment: "
        "python -m pip install python-can"
    ) from exc

try:
    from serial.tools import list_ports
except ImportError:
    list_ports = None


CMD_SET_TORQUE = 0x1D
CMD_READ_PROPERTY = 0x1E
CMD_WRITE_PROPERTY = 0x1F

KNOWN_PROPERTIES = {
    22001: "enable_angle_speed_torque_feedback",
    30003: "control_mode",
    31001: "device_or_state_probe",
    31002: "feedback_period_ms",
}


def frame_id(pda_id, cmd):
    return (pda_id << 5) + cmd


def hex_bytes(data):
    return " ".join(f"{byte:02X}" for byte in data)


def decode_property(data):
    if len(data) < 8:
        return None

    address, param_type, value = struct.unpack_from("<HHI", bytes(data), 0)
    return address, param_type, value, KNOWN_PROPERTIES.get(address, "unknown")


def decode_torque(data):
    if len(data) < 8:
        return None

    torque_nm = struct.unpack_from("<f", bytes(data), 0)[0]
    ramp_rate = struct.unpack_from("<h", bytes(data), 4)[0]
    input_mode = struct.unpack_from("<H", bytes(data), 6)[0]
    return torque_nm, ramp_rate, input_mode


def decode_feedback(data):
    if len(data) < 8:
        return None

    angle_deg = struct.unpack_from("<f", bytes(data), 0)[0]
    speed_rpm = struct.unpack_from("<h", bytes(data), 4)[0] * 0.01
    torque_nm = struct.unpack_from("<h", bytes(data), 6)[0] * 0.01
    return angle_deg, speed_rpm, torque_nm


def looks_like_known_property(data):
    decoded = decode_property(data)
    if decoded is None:
        return False

    address, param_type, _value, _name = decoded
    return param_type <= 3 and address in KNOWN_PROPERTIES


def classify_message(msg, target_id):
    if msg.is_extended_id:
        return "EXTENDED_FRAME_IGNORED"

    arb_id = msg.arbitration_id
    pda_id = (arb_id & 0x07E0) >> 5
    cmd = arb_id & 0x001F
    data = bytes(msg.data)

    if pda_id != target_id:
        return f"other_id={pda_id} cmd=0x{cmd:02X} data={hex_bytes(data)}"

    if cmd == CMD_WRITE_PROPERTY:
        decoded = decode_property(data)
        if decoded is None:
            return f"PDA_WRITE_PROPERTY malformed data={hex_bytes(data)}"
        address, param_type, value, name = decoded
        return (
            f"PDA_WRITE_PROPERTY id={pda_id} addr={address}({name}) "
            f"type={param_type} value={value} data={hex_bytes(data)}"
        )

    if cmd == CMD_READ_PROPERTY:
        if looks_like_known_property(data):
            decoded = decode_property(data)
            if decoded is None:
                return f"PDA_READ_PROPERTY malformed data={hex_bytes(data)}"
            address, param_type, value, name = decoded
            return (
                f"PDA_READ_PROPERTY id={pda_id} addr={address}({name}) "
                f"type={param_type} value={value} data={hex_bytes(data)}"
            )

        decoded = decode_feedback(data)
        if decoded is None:
            return f"PDA_FEEDBACK_ON_READ_ID malformed data={hex_bytes(data)}"
        angle_deg, speed_rpm, torque_nm = decoded
        return (
            f"PDA_FEEDBACK_ON_READ_ID id={pda_id} angleDeg={angle_deg:.2f} "
            f"speedRpm={speed_rpm:.2f} torqueNm={torque_nm:.3f} data={hex_bytes(data)}"
        )

    if cmd == CMD_SET_TORQUE:
        decoded = decode_torque(data)
        if decoded is None:
            return f"PDA_SET_TORQUE malformed data={hex_bytes(data)}"
        torque_nm, ramp_rate, input_mode = decoded
        return (
            f"PDA_SET_TORQUE id={pda_id} torqueNm={torque_nm:.3f} "
            f"ramp={ramp_rate} inputMode={input_mode} data={hex_bytes(data)}"
        )

    decoded = decode_feedback(data)
    if decoded is None:
        return f"PDA_FEEDBACK_CANDIDATE id={pda_id} cmd=0x{cmd:02X} malformed data={hex_bytes(data)}"
    angle_deg, speed_rpm, torque_nm = decoded
    return (
        f"PDA_FEEDBACK_CANDIDATE id={pda_id} cmd=0x{cmd:02X} "
        f"angleDeg={angle_deg:.2f} speedRpm={speed_rpm:.2f} torqueNm={torque_nm:.3f} "
        f"data={hex_bytes(data)}"
    )


def main():
    parser = argparse.ArgumentParser(description="Passive PDA08 CAN sniffer.")
    parser.add_argument("--list-serial", action="store_true", help="List Windows serial ports and exit.")
    parser.add_argument("--interface", help="python-can backend, e.g. pcan, kvaser, vector, slcan.")
    parser.add_argument("--channel", help="CAN channel, e.g. PCAN_USBBUS1 or COM11.")
    parser.add_argument("--bitrate", type=int, default=1_000_000, help="CAN bitrate, default 1000000.")
    parser.add_argument("--pda-id", type=int, default=1, help="PDA CAN ID, default 1.")
    parser.add_argument("--listen-only", action="store_true", help="Request listen-only mode if backend supports it.")
    parser.add_argument("--timeout", type=float, default=0.5, help="Receive timeout in seconds.")
    args = parser.parse_args()

    if args.list_serial:
        if list_ports is None:
            raise SystemExit("pyserial is required to list serial ports: python -m pip install pyserial")

        ports = list(list_ports.comports())
        if not ports:
            print("No serial ports found.")
            return

        for port in ports:
            print(f"{port.device}: {port.description} [{port.hwid}]")
        return

    if not args.interface or not args.channel:
        parser.error("--interface and --channel are required unless --list-serial is used.")

    bus_kwargs = {
        "interface": args.interface,
        "channel": args.channel,
        "bitrate": args.bitrate,
        "receive_own_messages": False,
    }
    if args.listen_only:
        bus_kwargs["listen_only"] = True

    print(
        f"Opening CAN: interface={args.interface} channel={args.channel} "
        f"bitrate={args.bitrate} pda_id={args.pda_id} listen_only={args.listen_only}"
    )
    print(
        "Expected Teensy TX IDs: "
        f"write=0x{frame_id(args.pda_id, CMD_WRITE_PROPERTY):03X}, "
        f"read=0x{frame_id(args.pda_id, CMD_READ_PROPERTY):03X}, "
        f"torque=0x{frame_id(args.pda_id, CMD_SET_TORQUE):03X}"
    )
    print("Press Ctrl+C to stop.\n")

    with can.Bus(**bus_kwargs) as bus:
        while True:
            msg = bus.recv(args.timeout)
            if msg is None:
                continue

            timestamp = time.strftime("%H:%M:%S")
            print(f"{timestamp} arb=0x{msg.arbitration_id:03X} {classify_message(msg, args.pda_id)}")


if __name__ == "__main__":
    main()
