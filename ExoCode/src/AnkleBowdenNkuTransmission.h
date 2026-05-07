#ifndef AnkleBowdenNkuTransmission_h
#define AnkleBowdenNkuTransmission_h

#include <float.h>
#include <math.h>

namespace ankle_bowden_nku
{
    static constexpr float PI_F = 3.14159265358979323846f;
    static constexpr float deg_to_rad = PI_F / 180.0f;
    static constexpr float rad_to_deg = 180.0f / PI_F;

    // Structural parameters. Units: m, rad, Nm.
    static constexpr float DRUM_RADIUS_M = 0.026f;
    static constexpr float ANKLE_ARM_M = 0.100f;
    static constexpr float SHEATH_TO_ATTACHMENT_M = 0.200f;
    static constexpr float BOWDEN_EFFICIENCY = 1.0f;
    static constexpr float THETA_MIN_RAD = -30.0f * deg_to_rad;
    static constexpr float THETA_MAX_RAD = 50.0f * deg_to_rad;

    static constexpr float MIN_VALID_JACOBIAN_ABS = 0.05f;
    static constexpr float MIN_VALID_TORQUE_GAIN_ABS = 0.05f;

    inline bool valid_float(float value)
    {
        return (value == value) && (value > -FLT_MAX) && (value < FLT_MAX);
    }

    inline float saturate_angle_rad(float theta_rad)
    {
        if (!valid_float(theta_rad))
        {
            return 0.0f;
        }

        if (theta_rad < THETA_MIN_RAD)
        {
            return THETA_MIN_RAD;
        }

        if (theta_rad > THETA_MAX_RAD)
        {
            return THETA_MAX_RAD;
        }

        return theta_rad;
    }

    inline float cable_length_m(float theta_rad)
    {
        if (!valid_float(theta_rad))
        {
            return 0.0f;
        }

        const float theta = saturate_angle_rad(theta_rad);
        return SHEATH_TO_ATTACHMENT_M - ANKLE_ARM_M * sinf(theta);
    }

    inline float moment_arm_m(float theta_rad)
    {
        if (!valid_float(theta_rad))
        {
            return 0.0f;
        }

        const float theta = saturate_angle_rad(theta_rad);
        return ANKLE_ARM_M * cosf(theta);
    }

    inline float jacobian_motor_per_joint(float theta_rad)
    {
        return moment_arm_m(theta_rad) / DRUM_RADIUS_M;
    }

    inline float torque_gain(float theta_rad)
    {
        return BOWDEN_EFFICIENCY * moment_arm_m(theta_rad) / DRUM_RADIUS_M;
    }

    inline bool motor_torque_from_joint_torque(float tau_joint_nm, float theta_rad, float* tau_motor_nm)
    {
        if (tau_motor_nm == nullptr)
        {
            return false;
        }

        *tau_motor_nm = 0.0f;

        if (!valid_float(tau_joint_nm) || !valid_float(theta_rad))
        {
            return false;
        }

        const float gain = torque_gain(theta_rad);
        if (!valid_float(gain) || (fabsf(gain) < MIN_VALID_TORQUE_GAIN_ABS))
        {
            return false;
        }

        *tau_motor_nm = tau_joint_nm / gain;
        return valid_float(*tau_motor_nm);
    }

    inline float joint_torque_from_motor_torque(float tau_motor_nm, float theta_rad)
    {
        if (!valid_float(tau_motor_nm) || !valid_float(theta_rad))
        {
            return 0.0f;
        }

        const float gain = torque_gain(theta_rad);
        if (!valid_float(gain) || (fabsf(gain) < MIN_VALID_TORQUE_GAIN_ABS))
        {
            return 0.0f;
        }

        return tau_motor_nm * gain;
    }

    inline bool joint_velocity_from_motor_velocity(float omega_motor_rad_s, float theta_rad, float* omega_joint_rad_s)
    {
        if (omega_joint_rad_s == nullptr)
        {
            return false;
        }

        *omega_joint_rad_s = 0.0f;

        if (!valid_float(omega_motor_rad_s) || !valid_float(theta_rad))
        {
            return false;
        }

        const float jacobian = jacobian_motor_per_joint(theta_rad);
        if (!valid_float(jacobian) || (fabsf(jacobian) < MIN_VALID_JACOBIAN_ABS))
        {
            return false;
        }

        *omega_joint_rad_s = omega_motor_rad_s / jacobian;
        return valid_float(*omega_joint_rad_s);
    }

    inline float motor_angle_from_joint_angle(float theta_rad)
    {
        if (!valid_float(theta_rad))
        {
            return 0.0f;
        }

        return (SHEATH_TO_ATTACHMENT_M - cable_length_m(theta_rad)) / DRUM_RADIUS_M;
    }

    inline bool joint_angle_from_motor_angle(float phi_motor_rad, float* theta_rad)
    {
        if (theta_rad == nullptr || !valid_float(phi_motor_rad))
        {
            return false;
        }

        if (ANKLE_ARM_M <= 0.0f)
        {
            *theta_rad = 0.0f;
            return false;
        }

        float argument = DRUM_RADIUS_M * phi_motor_rad / ANKLE_ARM_M;
        if (argument < -1.0f)
        {
            argument = -1.0f;
        }
        else if (argument > 1.0f)
        {
            argument = 1.0f;
        }

        const float theta = asinf(argument);
        if (!valid_float(theta))
        {
            *theta_rad = 0.0f;
            return false;
        }

        *theta_rad = saturate_angle_rad(theta);
        return true;
    }
}

#endif
