# OpenExo 外骨骼控制器架构文档 (原始设计)

> 基于 OpenExo 原始代码库，以 CubeMars (T-Motor) AK系列电机为核心

---

## 1. 双MCU架构 (Dual-MCU)

```
┌────────────────────────────────────────────────────────────────┐
│                    用户层 (User Layer)                          │
│     Python GUI (PC)  ◄── BLE ──►  iOS App (手机)               │
└───────────────────────────┬────────────────────────────────────┘
                            │ BLE (Nordic UART Service)
┌───────────────────────────▼────────────────────────────────────┐
│         通讯MCU (Coms MCU) — Arduino Nano 33 BLE (nRF52840)    │
│   ComsMCU → ExoBLE → BleParser → GattDb                       │
│   Battery采样 (I2C: INA219/INA260)                              │
└───────────────────────────┬────────────────────────────────────┘
                            │ UART (SLIP协议, 256kbaud)
┌───────────────────────────▼────────────────────────────────────┐
│         逻辑MCU (Logic MCU) — Teensy 4.1 (Cortex-M7, 600MHz)   │
│   Exo → Side(L/R) → Joint(×6) → Controller + Motor            │
│   SD卡配置 │ FSR步态检测 │ 扭矩传感器 │ IMU │ 气压计             │
└──────┬─────────────┬──────────────┬────────────┬───────────────┘
       │ CAN 1Mbps   │ analogRead   │ I2C        │ I2C
  ┌────▼────┐   ┌────▼─────┐  ┌────▼─────┐  ┌───▼──────┐
  │CubeMars │   │FSR/Torque│  │BNO055 IMU│  │LPS22HB   │
  │AK系列    │   │Sensor    │  │ThighIMU  │  │气压计     │
  └─────────┘   └──────────┘  └──────────┘  └──────────┘
```

| MCU | 芯片 | 角色 |
|-----|------|------|
| Logic MCU | Teensy 4.1 (ARM Cortex-M7) | 实时控制@500Hz、CAN电机通讯、传感器采集、SD卡配置、控制算法 |
| Coms MCU | Arduino Nano 33 BLE (nRF52840) | BLE GUI通讯、UART转发、电池采样、LED状态 |

---

## 2. 硬件层

### 2.1 PCB版本 (`Config.h` → `Board.h`)

| 宏 | 值 | Teensy | CAN引脚 |
|----|---|--------|---------|
| `AK_Board_V0_1` | 1 | Teensy 3.6 | CAN0 (Pin3 TX, Pin4 RX) |
| `AK_Board_V0_3` | 2 | Teensy 4.1 | CAN1 (Pin22 TX, Pin23 RX) |
| `AK_Board_V0_4` | 3 | Teensy 4.1 | CAN1 (Pin22 TX, Pin23 RX) |
| `AK_Board_V0_5_1` | 4 | Teensy 4.1 | CAN1 (Pin22 TX, Pin23 RX) |

### 2.2 CubeMars AK系列电机

所有CAN电机继承 `_CANMotor`，通过 `FlexCAN_T4` 库通讯，MIT Mini Cheetah协议。

| 型号 | 类名 | I_MAX (A) | V_MAX (rad/s) | Kt (Nm/A) | CAN帧 |
|------|------|-----------|---------------|-----------|-------|
| AK60 V1.0 | `AK60` | 22.0 | 41.87 | 0.068×6 = 0.408 | Standard |
| AK60 V1.1 | `AK60v1_1` | 13.5 | 23.04 | 0.1725×6 = 1.035 | Standard |
| AK60 V3 | `AK60v3` | 10.3 | 48.0 | 0.420×6 = 2.52 | **Extended** |
| AK70 | `AK70` | 23.2 | 15.5 | 0.13×10 = 1.3 | Standard |
| AK80 | `AK80` | 24.0 | 25.65 | 0.091×9 = 0.819 | Standard |
| AK45-36 | `AK45_36` | 6.5 | 5.44 | 0.127 | Standard |
| AK45-10 | `AK45_10` | 6.5 | 18.85 | 0.127 | Standard |

辅助电机类型：`NullMotor`（占位，无操作）

### 2.3 传感器系统

| 传感器 | 文件 | 通讯 | 功能 |
|--------|------|------|------|
| **电机内置编码器** | Motor.cpp | CAN (内置) | 位置p、速度v、电流i反馈 |
| **FSR** (Force Sensitive Resistor) | FSR.h/cpp | analogRead | 2分区(heel/toe)步态检测，Schmitt触发器判触地 |
| **FSR_Regressed** | FSR.h/cpp | analogRead | 回归标定版FSR，多步标定+refinement |
| **TorqueSensor** (应变片) | TorqueSensor.h/cpp | analogRead | 关节外部扭矩测量，含自动标定流程 |
| **AnkleIMU** (BNO055) | AnkleIMU.h/cpp | I2C | 踝关节9轴IMU姿态 |
| **ThIMU** (大腿IMU) | ThIMU.h/cpp | I2C (Wire) | 大腿段姿态测量 |
| **AnkleAngles** (电位器) | AnkleAngles.h/cpp | analogRead | 踝关节角度传感器 |
| **WaistBarometer** (LPS22HB) | WaistBarometer.h/cpp | I2C | 腰部气压/海拔高度 |
| **InclineDetector** | InclineDetector.h/cpp | I2C (依赖WaistBarometer) | 气压式坡度检测 |
| **InclinationDetector** | InclinationDetector.h/cpp | 纯算法 | 基于toe_stance + ankle_angle的坡度分类(Incline/Level/Decline) |
| **Battery** (INA219/INA260) | Battery.h/cpp | I2C | 电池电压电流监测 (SmartBattery/RCBattery) |
| **RealTimeI2C** | RealTimeI2C.h/cpp | I2C | I2C实时数据通道(Teensy→Nano备选路径) |

### 2.4 关节系统

每侧6个关节位，通过 `joint_id` 位掩码寻址：

| 关节 | 位掩码 | Left ID | Right ID |
|------|--------|---------|----------|
| Hip | 0x01 | 65 | 33 |
| Knee | 0x02 | 66 | 34 |
| Ankle | 0x04 | 68 | 36 |
| Elbow | 0x08 | 72 | 40 |
| Arm_1 | 0x10 | 80 | 48 |
| Arm_2 | 0x80 | 192 | 160 |

支持的外骨骼构型 (`exo_name`): bilateral_ankle, bilateral_hip, bilateral_knee, bilateral_elbow, bilateral_hip_ankle, bilateral_hip_elbow, bilateral_ankle_elbow, bilateral_arm, 以及各单侧变体和test模式，共23种。

---

## 3. 通讯协议

### 3.1 CAN Bus — 电机通讯

- **库**: `FlexCAN_T4` (Teensy原生)
- **波特率**: 1 Mbps
- **Singleton**: `CAN` 类 (`CAN.h`)
- **帧格式**:
  - Standard帧: `msg.id = motor_id` (大部分AK电机)
  - Extended帧: `msg.id = (8<<8)|motor_id` (AK60v3)
- **TX协议** (Teensy→电机, 8字节):
  - Standard: `[p_des_H, p_des_L, v_des_H, v_des_L|kp_H, kp_L, kd_H, kd_L|i_ff_H, i_ff_L]`
  - Extended: `[kp_H, kp_L|kd_H, kd_L, p_des_H, p_des_L, v_des_H|v_des_L|i_ff_H, i_ff_L]`
- **RX协议** (电机→Teensy, 8字节):
  - Standard: `[id, pos_H, pos_L, vel_H, vel_L|cur_H, cur_L, ...]`
  - Extended: `[pos_H, pos_L, vel_H, vel_L, cur_H, cur_L]`
- **特殊命令**: Enable `0xFC`, Disable `0xFD`, Zero `0xFE` (buf[7])
- **数据编码**: 定点数 `_float_to_uint` / `_uint_to_float`, 12位或16位
- **超时**: `_timeout = 500µs`, `timeout_count` 累计检测

### 3.2 UART — 板间通讯 (Teensy ↔ Nano)

- **物理**: Teensy Serial8 ↔ Nano Serial1
- **波特率**: 256000
- **帧协议**: SLIP (Serial Line Internet Protocol)
  - END=0300, ESC=0333, ESC_END=0334, ESC_ESC=0335
- **消息结构** (`UART_msg_t`): `{command, joint_id, len, data[32]:float}`
- **定点编码**: float×100→short int
- **Handler**: `UARTHandler` (Singleton)
- **命令集** (`UART_command_names`):

| 命令对 (get/update) | 功能 |
|---------------------|------|
| config | SD卡配置同步 (71个键值) |
| status | 系统状态 (trial_on/off/error等) |
| controller_params | 控制器+参数集切换 |
| controller_param | 单个参数更新 |
| real_time_data | 实时绘图数据 (最多11通道) |
| cal_trq_sensor | 扭矩传感器标定触发 |
| cal_fsr / refine_fsr | FSR标定/精修 |
| motor_enable_disable | 电机使能/禁用 |
| motor_zero | 电机归零 |
| error_code | 错误码上报 |
| FSR_thresholds | FSR阈值更新 |
| system_reset | 系统重启 |

### 3.3 BLE — GUI通讯

- **库**: ArduinoBLE (Nano 33 BLE原生)
- **协议**: Nordic UART Service (NUS)
- **类层次**: `ComsMCU` → `ExoBLE` → `BleParser` → `GattDb`
- **设备名**: `EXOBLE_` + MAC后6位
- **BLE命令** (`ble_names`):

| 字符 | 命令 | 方向 | 数据量 |
|------|------|------|--------|
| 'E' | start | GUI→FW | 0 |
| 'G' | stop | GUI→FW | 0 |
| 'H' | cal_trq | GUI→FW | 0 |
| 'L' | cal_fsr | GUI→FW | 0 |
| 'F' | new_trq (切换控制器) | GUI→FW | 4 |
| 'f' | update_param | GUI→FW | 4 |
| 'R' | new_fsr (阈值) | GUI→FW | 2 |
| 'x'/'w' | motors_on/off | GUI→FW | 0 |
| 'N' | mark | GUI→FW | 0 |
| 'Z' | reset_system | GUI→FW | 0 |
| '?' | send_real_time_data | FW→GUI | 11 |
| '~' | send_batt | FW→GUI | 1 |

### 3.4 SD卡 — 配置

- **库**: IniFile
- **接口**: SPI (Teensy内置SD)
- **文件**: `config.ini`, 71个键 (`ini_config::number_of_keys`)
- **内容**: 板型号、电机型号、减速比(1:1/2:1/3:1/4.5:1)、默认控制器、扭矩传感器开关、方向翻转、RoM、扭矩偏移等

### 3.5 I2C — 传感器总线

- **管理**: `I2CHandler.h` (统一I2C事务管理)
- **RealTimeI2C**: Teensy→Nano的I2C备选实时数据通道 (可通过 `REAL_TIME_I2C` 宏启用)
- **设备**: BNO055 IMU, INA219/INA260电池芯片, LPS22HB气压计

### 3.6 时序参数

| 参数 | 值 | 说明 |
|------|---|------|
| `LOOP_FREQ_HZ` | 500 | Teensy主循环 (2ms) |
| `LOOP_TIME_TOLERANCE` | 10% | 循环容差 |
| `BLE _real_time_msg_delay` | 9ms (~111Hz) | 实时BLE发送 |
| `BLE _status_msg_delay` | 2s | 状态消息 |
| `UART UPDATE_PERIOD` | 1ms | UART刷新 |
| `CONFIG_TIMEOUT` | 8s | 配置同步超时 |
| ADC分辨率 | 12位 (4096) | `analogReadResolution(12)` |

---

## 4. 软件架构

### 4.1 对象组合层次

```
ExoCode.ino (同一份代码，条件编译)
│
├── [Teensy编译] ─────────────────────────────
│   Exo (顶层)
│   ├── Side left_side
│   │   ├── HipJoint → _Joint
│   │   │   ├── Motor* (_CANMotor → AK60/AK70/AK80等)
│   │   │   ├── Controller* (PJMC/ZhangCollins/TREC等)
│   │   │   └── TorqueSensor (应变片)
│   │   ├── KneeJoint, AnkleJoint, ElbowJoint, Arm1Joint, Arm2Joint
│   │   ├── FSR _heel_fsr, FSR_Regressed _toe_fsr
│   │   └── InclinationDetector*
│   ├── Side right_side (镜像)
│   ├── SyncLed (动捕同步)
│   └── StatusLed (RGB状态)
│   + UARTHandler (Singleton) ── UART板间通讯
│
└── [Nano编译] ──────────────────────────────
    ComsMCU
    ├── ExoBLE
    │   ├── BleParser (序列化/反序列化)
    │   ├── GattDb (GATT特征定义)
    │   └── BleMessageQueue
    ├── Battery* (_Battery → SmartBattery / RCBattery)
    └── ComsLed (状态LED)
    + UARTHandler (Singleton)
```

### 4.2 数据层次 (`ExoData`)

```
ExoData
├── config[71]          // SD卡配置
├── status              // trial_on/off/error/fsr_cal等
├── mark, estop, user_paused, sync_led_state
├── SideData left_side / right_side
│   ├── heel_fsr, toe_fsr                    // FSR读数
│   ├── heel_stance, toe_stance              // 触地状态
│   ├── percent_gait, percent_stance, percent_swing
│   ├── ground_strike, toe_on, toe_off       // 步态事件
│   ├── expected_step/stance/swing_duration
│   ├── inclination                          // 坡度枚举
│   ├── fsr_lower/upper_threshold            // FSR阈值
│   ├── do_calibration_toe/heel_fsr          // FSR标定标志
│   └── JointData hip/knee/ankle/elbow/arm_1/arm_2
│       ├── is_used, id, flip_direction
│       ├── position, velocity               // 来自CAN电机
│       ├── torque_reading, torque_offset     // 来自TorqueSensor
│       ├── calibrate_torque_sensor           // 标定标志
│       ├── MotorData motor
│       │   ├── p, v, i                      // CAN反馈
│       │   ├── p_des, v_des, kp, kd, t_ff   // CAN命令
│       │   ├── enabled, is_on, motor_type, gearing, kt
│       │   └── last_command, timeout_count, flip_direction
│       └── ControllerData controller
│           ├── controller (ID), parameter_set
│           ├── parameters[20]               // 可调参数
│           ├── desired_torque, filtered_torque_reading
│           └── (各控制器特有字段)
```

### 4.3 Teensy 主循环 (Superloop @ 500Hz)

```
setup():
  analogReadResolution(12)
  ini_parser() → SD卡读config.ini → config_to_send[71]
  ctrl_param_array_gen() → 生成控制参数列表字符串
  send_bulk_char() → 通过UART发送参数列表给Nano

loop():
  [首次] ExoData exo_data(config) + Exo exo(&exo_data)
  [首次] UART: wait_for_get_config() → 等Nano请求config
  [首次] 各关节: motor.is_on=true, kp=kd=0
  [首次,HEADLESS] 设置默认控制器, 先zero_torque再切换

  [每次] exo.run():  (仅当delta_t >= 2ms时执行)
    sync_led.handler()
    left_side.run_side() / right_side.run_side():
      ├─ read_data():
      │   FSR读取 → 步态检测(ground_strike/toe_on/toe_off)
      │   → percent_gait/stance/swing计算
      │   → InclinationDetector检测坡度
      │   → 各Joint.read_data(): Motor CAN读取 + TorqueSensor读取
      ├─ check_calibration(): FSR标定 + TorqueSensor标定
      └─ update_motor_cmds():
          各Joint: Controller.calc_motor_cmd() → Motor.transaction(torque)
    status_led.update()
    UART: poll() → handle_msg()
    UART: get_real_time_data() (每9ms发送)
```

### 4.4 Nano 主循环

```
setup():
  readSingleMessageBlocking() → 接收Teensy参数列表
  UART: get_config() → 获取SD卡配置
  ComsLed: 绿=成功, 黄=超时
  [可选] real_time_i2c::init()

loop():
  [首次] ExoData + ComsMCU
  [每次]
    mcu.handle_ble()      // BLE事件处理
    mcu.local_sample()    // 电池采样
    mcu.update_UART()     // UART消息处理
    mcu.update_gui()      // 发送实时数据到GUI
    mcu.handle_errors()   // 错误处理
```

---

## 5. 控制器算法

所有控制器继承 `_Controller`，核心接口 `float calc_motor_cmd()` 返回扭矩(Nm)。

### 5.1 通用控制器

| 控制器 | 类名 | 适用关节 | 算法 |
|--------|------|---------|------|
| Zero Torque | `ZeroTorque` | 全部 | 输出0 Nm |
| Constant Torque | `ConstantTorque` | 全部 | 恒扭矩，带切换滤波 |
| Calibration Manager | `CalibrManager` | 全部 | 方向标定工具 |
| Chirp | `Chirp` | 全部 | 正弦扫频，硬件验证 |
| Step | `Step` | 全部 | 阶跃响应，硬件验证 |

### 5.2 踝关节控制器

| 控制器 | 类名 | 算法 |
|--------|------|------|
| PJMC | `ProportionalJointMoment` | 基于toe FSR归一化值的比例跖屈扭矩 (Gasparri 2019) |
| Zhang-Collins | `ZhangCollins` | 步态相位spline曲线，4节点 (Zhang 2017) |
| TREC | `TREC` | 地形自适应踝关节控制，带中性角捕获 (Cuddeback) |
| SPV2 | `SPV2` | 自适应优化控制 + INA260电流监测 + Golden Search (开发中) |
| PJMC_PLUS | `PJMC_PLUS` | PJMC增强版 (开发中) |
| Spline | `Spline` | 5节点spline曲线 |

### 5.3 髋关节控制器

| 控制器 | 类名 | 算法 |
|--------|------|------|
| Franks-Collins Hip | `FranksCollinsHip` | 双峰spline (trough+peak)，步态相位驱动 (Franks 2021) |
| PHMC | `ProportionalHipMoment` | 3状态机(Mid-Late Swing/Stance/Early Swing)，基于GRF的髋矩估计 (开发中) |
| Spline | `Spline` | 5节点spline曲线 |

### 5.4 肘关节控制器

| 控制器 | 类名 | 算法 |
|--------|------|------|
| Elbow Min-Max | `ElbowMinMax` | 基于FlexSense/ExtenseSense FSR的屈伸辅助 (Colley 2024) |

### 5.5 底层控制工具

- **PID**: `_pid(cmd, measurement, p, i, d)` — 含积分累加、微分滤波、时间步检查
- **CF-MFAC**: `_cf_mfac(reference, measurement)` — 紧凑格式无模型自适应控制
- **PJMC Generic**: `_pjmc_generic(fsr, threshold, sp_pos, sp_neg)` — 通用比例关节矩

---

## 6. 步态检测与标定流程

### 6.1 FSR步态检测 (`Side.cpp`)

```
heel_fsr.read() + toe_fsr.read()
  → Schmitt触发器 → heel_stance / toe_stance
  → _check_ground_strike() → ground_strike (heel着地)
  → _check_toe_on() / _check_toe_off()
  → _update_expected_duration() → expected_step_duration
  → _calc_percent_gait() → percent_gait (0~100%)
  → _calc_percent_stance() / _calc_percent_swing()
```

### 6.2 标定流程

1. **FSR标定** (`FSR::calibrate`): 时间窗口内找信号粗范围
2. **FSR精修** (`FSR::refine_calibration`): 多步平均max/min
3. **扭矩标定** (`TorqueSensor::calibrate`): 记录零偏移，可从SD卡预设 (`torque_offset`)
4. **电机归零** (`_CANMotor::zero`): CAN命令 buf[7]=0xFE

---

## 7. 错误管理

- **ErrorManager** (`ErrorManager.h`): 错误检测与上报
- **ErrorReporter** (`ErrorReporter.h`): 格式化错误信息
- **error_codes.h**: 错误码定义
- **error_types.h / error_map.h**: 错误类型映射
- **电机超时检测**: `_CANMotor::check_response()` — 电流方差监测，低方差+disabled时自动重使能
- **E-Stop**: `motor_stop_pin` 硬件急停 (默认禁用，可在 `Exo.cpp` 中启用)

---

## 8. 外部工具

| 工具 | 路径 | 说明 |
|------|------|------|
| Python GUI | `Python_GUI/` | PC端BLE控制、实时绘图、CSV数据记录 |
| iOS App | `IOS_App/` | 移动端BLE控制 |
| SD Card | `SDCard/` | config.ini模板 |
| Documentation | `Documentation/` + `html/` + `latex/` | Doxygen文档 |
