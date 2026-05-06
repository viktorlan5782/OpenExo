#include "MotorData.h"
#include "ParseIni.h"
#include "Config.h"
#include "Logger.h"

static bool motor_data_is_pda_motor(uint8_t motor_type)
{
    return (motor_type == (uint8_t)config_defs::motor::PDA08) ||
           (motor_type == (uint8_t)config_defs::motor::PDA01);
}

static uint8_t motor_data_default_pda_id(config_defs::joint_id id)
{
    switch (id)
    {
        case config_defs::joint_id::left_hip:
            return 1;
        case config_defs::joint_id::left_knee:
            return 2;
        case config_defs::joint_id::right_hip:
            return 3;
        case config_defs::joint_id::right_knee:
            return 4;
        case config_defs::joint_id::left_ankle:
            return 5;
        case config_defs::joint_id::right_ankle:
            return 6;
        case config_defs::joint_id::left_elbow:
            return 7;
        case config_defs::joint_id::right_elbow:
            return 8;
        case config_defs::joint_id::left_arm_1:
            return 9;
        case config_defs::joint_id::right_arm_1:
            return 10;
        case config_defs::joint_id::left_arm_2:
            return 11;
        case config_defs::joint_id::right_arm_2:
            return 12;
        default:
            return 1;
    }
}

static int motor_data_pda_can_id_idx(config_defs::joint_id id)
{
    switch (id)
    {
        case config_defs::joint_id::left_hip: return config_defs::left_hip_pda_can_id_idx;
        case config_defs::joint_id::right_hip: return config_defs::right_hip_pda_can_id_idx;
        case config_defs::joint_id::left_knee: return config_defs::left_knee_pda_can_id_idx;
        case config_defs::joint_id::right_knee: return config_defs::right_knee_pda_can_id_idx;
        case config_defs::joint_id::left_ankle: return config_defs::left_ankle_pda_can_id_idx;
        case config_defs::joint_id::right_ankle: return config_defs::right_ankle_pda_can_id_idx;
        case config_defs::joint_id::left_elbow: return config_defs::left_elbow_pda_can_id_idx;
        case config_defs::joint_id::right_elbow: return config_defs::right_elbow_pda_can_id_idx;
        case config_defs::joint_id::left_arm_1: return config_defs::left_arm_1_pda_can_id_idx;
        case config_defs::joint_id::right_arm_1: return config_defs::right_arm_1_pda_can_id_idx;
        case config_defs::joint_id::left_arm_2: return config_defs::left_arm_2_pda_can_id_idx;
        case config_defs::joint_id::right_arm_2: return config_defs::right_arm_2_pda_can_id_idx;
        default: return config_defs::left_hip_pda_can_id_idx;
    }
}

static int motor_data_pda_torque_limit_idx(config_defs::joint_id id)
{
    switch (id)
    {
        case config_defs::joint_id::left_hip: return config_defs::left_hip_pda_torque_limit_idx;
        case config_defs::joint_id::right_hip: return config_defs::right_hip_pda_torque_limit_idx;
        case config_defs::joint_id::left_knee: return config_defs::left_knee_pda_torque_limit_idx;
        case config_defs::joint_id::right_knee: return config_defs::right_knee_pda_torque_limit_idx;
        case config_defs::joint_id::left_ankle: return config_defs::left_ankle_pda_torque_limit_idx;
        case config_defs::joint_id::right_ankle: return config_defs::right_ankle_pda_torque_limit_idx;
        case config_defs::joint_id::left_elbow: return config_defs::left_elbow_pda_torque_limit_idx;
        case config_defs::joint_id::right_elbow: return config_defs::right_elbow_pda_torque_limit_idx;
        case config_defs::joint_id::left_arm_1: return config_defs::left_arm_1_pda_torque_limit_idx;
        case config_defs::joint_id::right_arm_1: return config_defs::right_arm_1_pda_torque_limit_idx;
        case config_defs::joint_id::left_arm_2: return config_defs::left_arm_2_pda_torque_limit_idx;
        case config_defs::joint_id::right_arm_2: return config_defs::right_arm_2_pda_torque_limit_idx;
        default: return config_defs::left_hip_pda_torque_limit_idx;
    }
}

static int motor_data_pda_speed_limit_idx(config_defs::joint_id id)
{
    switch (id)
    {
        case config_defs::joint_id::left_hip: return config_defs::left_hip_pda_speed_limit_idx;
        case config_defs::joint_id::right_hip: return config_defs::right_hip_pda_speed_limit_idx;
        case config_defs::joint_id::left_knee: return config_defs::left_knee_pda_speed_limit_idx;
        case config_defs::joint_id::right_knee: return config_defs::right_knee_pda_speed_limit_idx;
        case config_defs::joint_id::left_ankle: return config_defs::left_ankle_pda_speed_limit_idx;
        case config_defs::joint_id::right_ankle: return config_defs::right_ankle_pda_speed_limit_idx;
        case config_defs::joint_id::left_elbow: return config_defs::left_elbow_pda_speed_limit_idx;
        case config_defs::joint_id::right_elbow: return config_defs::right_elbow_pda_speed_limit_idx;
        case config_defs::joint_id::left_arm_1: return config_defs::left_arm_1_pda_speed_limit_idx;
        case config_defs::joint_id::right_arm_1: return config_defs::right_arm_1_pda_speed_limit_idx;
        case config_defs::joint_id::left_arm_2: return config_defs::left_arm_2_pda_speed_limit_idx;
        case config_defs::joint_id::right_arm_2: return config_defs::right_arm_2_pda_speed_limit_idx;
        default: return config_defs::left_hip_pda_speed_limit_idx;
    }
}

static int motor_data_pda_control_mode_idx(config_defs::joint_id id)
{
    switch (id)
    {
        case config_defs::joint_id::left_hip: return config_defs::left_hip_pda_control_mode_idx;
        case config_defs::joint_id::right_hip: return config_defs::right_hip_pda_control_mode_idx;
        case config_defs::joint_id::left_knee: return config_defs::left_knee_pda_control_mode_idx;
        case config_defs::joint_id::right_knee: return config_defs::right_knee_pda_control_mode_idx;
        case config_defs::joint_id::left_ankle: return config_defs::left_ankle_pda_control_mode_idx;
        case config_defs::joint_id::right_ankle: return config_defs::right_ankle_pda_control_mode_idx;
        case config_defs::joint_id::left_elbow: return config_defs::left_elbow_pda_control_mode_idx;
        case config_defs::joint_id::right_elbow: return config_defs::right_elbow_pda_control_mode_idx;
        case config_defs::joint_id::left_arm_1: return config_defs::left_arm_1_pda_control_mode_idx;
        case config_defs::joint_id::right_arm_1: return config_defs::right_arm_1_pda_control_mode_idx;
        case config_defs::joint_id::left_arm_2: return config_defs::left_arm_2_pda_control_mode_idx;
        case config_defs::joint_id::right_arm_2: return config_defs::right_arm_2_pda_control_mode_idx;
        default: return config_defs::left_hip_pda_control_mode_idx;
    }
}

static void motor_data_apply_pda_model_spec(MotorData* motor)
{
    if (motor->motor_type == (uint8_t)config_defs::motor::PDA01)
    {
        motor->pda_rated_torque_nm = pda_config::PDA01_RATED_TORQUE_NM;
        motor->pda_peak_torque_nm = pda_config::PDA01_PEAK_TORQUE_NM;
        motor->pda_rated_speed_rpm = pda_config::PDA01_RATED_SPEED_RPM;
        motor->pda_rated_current_a = pda_config::PDA01_RATED_CURRENT_A;
        motor->pda_stall_current_a = pda_config::PDA01_STALL_CURRENT_A;
        motor->pda_rotor_inertia_gcm2 = pda_config::PDA01_ROTOR_INERTIA_GCM2;
        return;
    }

    if (motor->motor_type == (uint8_t)config_defs::motor::PDA08)
    {
        motor->pda_rated_torque_nm = pda_config::PDA08_RATED_TORQUE_NM;
        motor->pda_peak_torque_nm = pda_config::PDA08_PEAK_TORQUE_NM;
        motor->pda_rated_speed_rpm = pda_config::PDA08_RATED_SPEED_RPM;
        motor->pda_rated_current_a = pda_config::PDA08_RATED_CURRENT_A;
        motor->pda_stall_current_a = pda_config::PDA08_STALL_CURRENT_A;
        motor->pda_rotor_inertia_gcm2 = pda_config::PDA08_ROTOR_INERTIA_GCM2;
        return;
    }

    motor->pda_rated_torque_nm = 0.0f;
    motor->pda_peak_torque_nm = 0.0f;
    motor->pda_rated_speed_rpm = 0.0f;
    motor->pda_rated_current_a = 0.0f;
    motor->pda_stall_current_a = 0.0f;
    motor->pda_rotor_inertia_gcm2 = 0.0f;
}

static void motor_data_load_pda_config(MotorData* motor, uint8_t* config_to_send)
{
    motor_data_apply_pda_model_spec(motor);

    const uint8_t raw_can_id = config_to_send[motor_data_pda_can_id_idx(motor->id)];
    const uint8_t raw_torque_limit = config_to_send[motor_data_pda_torque_limit_idx(motor->id)];
    const uint8_t raw_speed_limit = config_to_send[motor_data_pda_speed_limit_idx(motor->id)];
    const uint8_t raw_control_mode = config_to_send[motor_data_pda_control_mode_idx(motor->id)];

    motor->pda_id = raw_can_id;
    if (motor_data_is_pda_motor(motor->motor_type) && motor->pda_id == 0)
    {
        motor->pda_id = motor_data_default_pda_id(motor->id);
    }

    motor->pda_torque_limit_nm = raw_torque_limit > 0 ? (0.1f * raw_torque_limit) : motor->pda_rated_torque_nm;
    motor->pda_speed_limit_rpm = raw_speed_limit > 0 ? (10.0f * raw_speed_limit) : motor->pda_rated_speed_rpm;
    motor->pda_control_mode = raw_control_mode > 0 ? raw_control_mode : (uint8_t)config_defs::pda_control_mode::torque_direct;
}

/*
 * Constructor for the motor data.
 * Takes the joint id and the array from the INI parser.
 * Stores the id, if it is on the left side (for convenience), and the motor type
 * It also has the info for the motor CAN packages.
 */
MotorData::MotorData(config_defs::joint_id id, uint8_t* config_to_send)
{
    this->id = id;
    this->is_left = ((uint8_t)this->id & (uint8_t)config_defs::joint_id::left) == (uint8_t)config_defs::joint_id::left;
    
    switch ((uint8_t)this->id & (~(uint8_t)config_defs::joint_id::left & ~(uint8_t)config_defs::joint_id::right))  //Use the id with the side masked out.
    {
        case (uint8_t)config_defs::joint_id::hip:
        {
            motor_type = config_to_send[config_defs::hip_idx];
            
            switch (config_to_send[config_defs::hip_gear_idx])
            {
                case (uint8_t)config_defs::gearing::gearing_1_1:
                {
                    gearing = 1;
                    break;
                }
                case (uint8_t)config_defs::gearing::gearing_2_1:
                {
                    gearing = 2;
                    break;
                }                
                case (uint8_t)config_defs::gearing::gearing_3_1:
                {
                    gearing = 3;
                    break;
                }
                case (uint8_t)config_defs::gearing::gearing_4_5_1:
                {
                    gearing = 4.5;
                    break;
                }
                default:
                {
                    gearing = 1;
                    break;
                }
            }  
            if ((config_to_send[config_defs::hip_flip_motor_dir_idx] == (uint8_t)config_defs::flip_motor_dir::both) || ((config_to_send[config_defs::hip_flip_motor_dir_idx] == (uint8_t)config_defs::flip_motor_dir::left) && this->is_left) || ((config_to_send[config_defs::hip_flip_motor_dir_idx] == (uint8_t)config_defs::flip_motor_dir::right) && (!this->is_left)))
            {
                this->flip_direction = 1;
            }
            else
            {
                this->flip_direction = 0;
            }
            break;
        }
        case (uint8_t)config_defs::joint_id::knee:
        {
            motor_type = config_to_send[config_defs::knee_idx];
            
            switch (config_to_send[config_defs::knee_gear_idx])
            {
                case (uint8_t)config_defs::gearing::gearing_1_1:
                {
                    gearing = 1;
                    break;
                }
                case (uint8_t)config_defs::gearing::gearing_2_1:
                {
                    gearing = 2;
                    break;
                } 
                case (uint8_t)config_defs::gearing::gearing_3_1:
                {
                    gearing = 3;
                    break;
                }
                case (uint8_t)config_defs::gearing::gearing_4_5_1:
                {
                    gearing = 4.5;
                    break;
                }
                default:
                {
                    gearing = 1;
                    break;
                }
            }  
            
            if ((config_to_send[config_defs::knee_flip_motor_dir_idx] == (uint8_t)config_defs::flip_motor_dir::both) || ((config_to_send[config_defs::knee_flip_motor_dir_idx] == (uint8_t)config_defs::flip_motor_dir::left) && this->is_left) || ((config_to_send[config_defs::knee_flip_motor_dir_idx] == (uint8_t)config_defs::flip_motor_dir::right) && (!this->is_left)))
            {
                this->flip_direction = 1;
            }
            else
            {
                this->flip_direction = 0;
            }
            break;
        }
        case (uint8_t)config_defs::joint_id::ankle:
        {
            motor_type = config_to_send[config_defs::ankle_idx];
            
            switch (config_to_send[config_defs::ankle_gear_idx])
            {
                case (uint8_t)config_defs::gearing::gearing_1_1:
                {
                    gearing = 1;
                    break;
                }
                case (uint8_t)config_defs::gearing::gearing_2_1:
                {
                    gearing = 2;
                    break;
                } 
                case (uint8_t)config_defs::gearing::gearing_3_1:
                {
                    gearing = 3;
                    break;
                }
                case (uint8_t)config_defs::gearing::gearing_4_5_1:
                {
                    gearing = 4.5;
                    break;
                }
                default:
                {
                    gearing = 1;
                    break;
                }
            }  
            
            if ((config_to_send[config_defs::ankle_flip_motor_dir_idx] == (uint8_t)config_defs::flip_motor_dir::both) || ((config_to_send[config_defs::ankle_flip_motor_dir_idx] == (uint8_t)config_defs::flip_motor_dir::left) && this->is_left) || ((config_to_send[config_defs::ankle_flip_motor_dir_idx] == (uint8_t)config_defs::flip_motor_dir::right) && (!this->is_left)))
            {
                this->flip_direction = 1;
            }
            else
            {
                this->flip_direction = 0;
            }
            break;
        }
        case (uint8_t)config_defs::joint_id::elbow:
        {
            motor_type = config_to_send[config_defs::elbow_idx];

            switch (config_to_send[config_defs::elbow_gear_idx])
            {
            case (uint8_t)config_defs::gearing::gearing_1_1:
            {
                gearing = 1;
                break;
            }
            case (uint8_t)config_defs::gearing::gearing_2_1:
            {
                gearing = 2;
                break;
            }
            case (uint8_t)config_defs::gearing::gearing_3_1:
            {
                gearing = 3;
                break;
            }
            case (uint8_t)config_defs::gearing::gearing_4_5_1:
            {
                gearing = 4.5;
                break;
            }
            default:
            {
                gearing = 1;
                break;
            }
            }

            if ((config_to_send[config_defs::elbow_flip_motor_dir_idx] == (uint8_t)config_defs::flip_motor_dir::both) || ((config_to_send[config_defs::elbow_flip_motor_dir_idx] == (uint8_t)config_defs::flip_motor_dir::left) && this->is_left) || ((config_to_send[config_defs::elbow_flip_motor_dir_idx] == (uint8_t)config_defs::flip_motor_dir::right) && (!this->is_left)))
            {
                this->flip_direction = 1;
            }
            else
            {
                this->flip_direction = 0;
            }
            break;
        }
        case (uint8_t)config_defs::joint_id::arm_1:
        {
            motor_type = config_to_send[config_defs::arm_1_idx];

            switch (config_to_send[config_defs::arm_1_gear_idx])
            {
                case (uint8_t)config_defs::gearing::gearing_1_1:
                {
                    gearing = 1;
                    break;
                }
                case (uint8_t)config_defs::gearing::gearing_2_1:
                {
                    gearing = 2;
                    break;
                }
                case (uint8_t)config_defs::gearing::gearing_3_1:
                {
                    gearing = 3;
                    break;
                }
                case (uint8_t)config_defs::gearing::gearing_4_5_1:
                {
                    gearing = 4.5;
                    break;
                }
                default:
                {
                    gearing = 1;
                    break;
                }
            }

            if ((config_to_send[config_defs::arm_1_flip_motor_dir_idx] == (uint8_t)config_defs::flip_motor_dir::both) || ((config_to_send[config_defs::arm_1_flip_motor_dir_idx] == (uint8_t)config_defs::flip_motor_dir::left) && this->is_left) || ((config_to_send[config_defs::arm_1_flip_motor_dir_idx] == (uint8_t)config_defs::flip_motor_dir::right) && (!this->is_left)))
            {
                this->flip_direction = 1;
            }
            else
            {
                this->flip_direction = 0;
            }
            break;
        }
        case (uint8_t)config_defs::joint_id::arm_2:
        {
            motor_type = config_to_send[config_defs::arm_2_idx];

            switch (config_to_send[config_defs::arm_2_gear_idx])
            {
                case (uint8_t)config_defs::gearing::gearing_1_1:
                {
                    gearing = 1;
                    break;
                }
                case (uint8_t)config_defs::gearing::gearing_2_1:
                {
                    gearing = 2;
                    break;
                }
                case (uint8_t)config_defs::gearing::gearing_3_1:
                {
                    gearing = 3;
                    break;
                }
                case (uint8_t)config_defs::gearing::gearing_4_5_1:
                {
                    gearing = 4.5;
                    break;
                }
                default:
                {
                    gearing = 1;
                    break;
                }
            }

            if ((config_to_send[config_defs::arm_2_flip_motor_dir_idx] == (uint8_t)config_defs::flip_motor_dir::both) || ((config_to_send[config_defs::arm_2_flip_motor_dir_idx] == (uint8_t)config_defs::flip_motor_dir::left) && this->is_left) || ((config_to_send[config_defs::arm_2_flip_motor_dir_idx] == (uint8_t)config_defs::flip_motor_dir::right) && (!this->is_left)))
            {
                this->flip_direction = 1;
            }
            else
            {
                this->flip_direction = 0;
            }
            break;
        }
    }
    
    
    //For AK-Series Motors Only
    p = 0;      //Read position
    v = 0;      //Read velocity
    i = 0;      //Read current
    p_des = 0;  
    v_des = 0;
    kp = 0;
    kd = 0;
    t_ff = 0;
    kt = 0;
    last_command = 0;
    timeout_count = 0;
    pda_id = 0;
    pda_control_mode = (uint8_t)config_defs::pda_control_mode::torque_direct;
    pda_angle_deg = 0;
    pda_speed_rpm = 0;
    pda_torque_nm = 0;
    pda_voltage_v = 0;
    pda_current_a = 0;
    pda_torque_limit_nm = 0;
    pda_speed_limit_rpm = 0;
    pda_rated_torque_nm = 0;
    pda_peak_torque_nm = 0;
    pda_rated_speed_rpm = 0;
    pda_rated_current_a = 0;
    pda_stall_current_a = 0;
    pda_rotor_inertia_gcm2 = 0;
    pda_last_feedback_us = 0;
    pda_feedback_valid = false;
    motor_data_load_pda_config(this, config_to_send);
    
};

void MotorData::reconfigure(uint8_t* config_to_send) 
{
    switch ((uint8_t)this->id & (~(uint8_t)config_defs::joint_id::left & ~(uint8_t)config_defs::joint_id::right))  //Use the id with the side masked out.
    {
        case (uint8_t)config_defs::joint_id::hip:
        {
            motor_type = config_to_send[config_defs::hip_idx];
            
            switch (config_to_send[config_defs::hip_gear_idx])
            {
                case (uint8_t)config_defs::gearing::gearing_1_1:
                {
                    gearing = 1;
                    break;
                }
                case (uint8_t)config_defs::gearing::gearing_2_1:
                {
                    gearing = 2;
                    break;
                }                
                case (uint8_t)config_defs::gearing::gearing_3_1:
                {
                    gearing = 3;
                    break;
                }
                case (uint8_t)config_defs::gearing::gearing_4_5_1:
                {
                    gearing = 4.5;
                    break;
                }
                default:
                {
                    gearing = 1;
                    break;
                }
            }  
            if ((config_to_send[config_defs::hip_flip_motor_dir_idx] == (uint8_t)config_defs::flip_motor_dir::both) || ((config_to_send[config_defs::hip_flip_motor_dir_idx] == (uint8_t)config_defs::flip_motor_dir::left) && this->is_left) || ((config_to_send[config_defs::hip_flip_motor_dir_idx] == (uint8_t)config_defs::flip_motor_dir::right) && (!this->is_left)))
            {
                this->flip_direction = 1;
            }
            else
            {
                this->flip_direction = 0;
            }
            break;
        }
        case (uint8_t)config_defs::joint_id::knee:
        {
            motor_type = config_to_send[config_defs::knee_idx];
            
            switch (config_to_send[config_defs::knee_gear_idx])
            {
                case (uint8_t)config_defs::gearing::gearing_1_1:
                {
                    gearing = 1;
                    break;
                }
                case (uint8_t)config_defs::gearing::gearing_2_1:
                {
                    gearing = 2;
                    break;
                } 
                case (uint8_t)config_defs::gearing::gearing_3_1:
                {
                    gearing = 3;
                    break;
                }
                case (uint8_t)config_defs::gearing::gearing_4_5_1:
                {
                    gearing = 4.5;
                    break;
                }
                default:
                {
                    gearing = 1;
                    break;
                }
            }  
            
            if ((config_to_send[config_defs::knee_flip_motor_dir_idx] == (uint8_t)config_defs::flip_motor_dir::both) || ((config_to_send[config_defs::knee_flip_motor_dir_idx] == (uint8_t)config_defs::flip_motor_dir::left) && this->is_left) || ((config_to_send[config_defs::knee_flip_motor_dir_idx] == (uint8_t)config_defs::flip_motor_dir::right) && (!this->is_left)))
            {
                this->flip_direction = 1;
            }
            else
            {
                this->flip_direction = 0;
            }
            break;
        }
        case (uint8_t)config_defs::joint_id::ankle:
        {
            motor_type = config_to_send[config_defs::ankle_idx];
            
            switch (config_to_send[config_defs::ankle_gear_idx])
            {
                case (uint8_t)config_defs::gearing::gearing_1_1:
                {
                    gearing = 1;
                    break;
                }
                case (uint8_t)config_defs::gearing::gearing_2_1:
                {
                    gearing = 2;
                    break;
                } 
                case (uint8_t)config_defs::gearing::gearing_3_1:
                {
                    gearing = 3;
                    break;
                }
                case (uint8_t)config_defs::gearing::gearing_4_5_1:
                {
                    gearing = 4.5;
                    break;
                }
                default:
                {
                    gearing = 1;
                    break;
                }
            }  
            
            if ((config_to_send[config_defs::ankle_flip_motor_dir_idx] == (uint8_t)config_defs::flip_motor_dir::both) || ((config_to_send[config_defs::ankle_flip_motor_dir_idx] == (uint8_t)config_defs::flip_motor_dir::left) && this->is_left) || ((config_to_send[config_defs::ankle_flip_motor_dir_idx] == (uint8_t)config_defs::flip_motor_dir::right) && (!this->is_left)))
            {
                this->flip_direction = 1;
            }
            else
            {
                this->flip_direction = 0;
            }
            break;
        }
        case (uint8_t)config_defs::joint_id::elbow:
        {
            motor_type = config_to_send[config_defs::elbow_idx];

            switch (config_to_send[config_defs::elbow_gear_idx])
            {
            case (uint8_t)config_defs::gearing::gearing_1_1:
            {
                gearing = 1;
                break;
            }
            case (uint8_t)config_defs::gearing::gearing_2_1:
            {
                gearing = 2;
                break;
            }
            case (uint8_t)config_defs::gearing::gearing_3_1:
            {
                gearing = 3;
                break;
            }
            case (uint8_t)config_defs::gearing::gearing_4_5_1:
            {
                gearing = 4.5;
                break;
            }
            default:
            {
                gearing = 1;
                break;
            }
            }

            if ((config_to_send[config_defs::elbow_flip_motor_dir_idx] == (uint8_t)config_defs::flip_motor_dir::both) || ((config_to_send[config_defs::elbow_flip_motor_dir_idx] == (uint8_t)config_defs::flip_motor_dir::left) && this->is_left) || ((config_to_send[config_defs::elbow_flip_motor_dir_idx] == (uint8_t)config_defs::flip_motor_dir::right) && (!this->is_left)))
            {
                this->flip_direction = 1;
            }
            else
            {
                this->flip_direction = 0;
            }
            break;
        }
        case (uint8_t)config_defs::joint_id::arm_1:
        {
            motor_type = config_to_send[config_defs::arm_1_idx];

            switch (config_to_send[config_defs::arm_1_gear_idx])
            {
                case (uint8_t)config_defs::gearing::gearing_1_1:
                {
                    gearing = 1;
                    break;
                }
                case (uint8_t)config_defs::gearing::gearing_2_1:
                {
                    gearing = 2;
                    break;
                }
                case (uint8_t)config_defs::gearing::gearing_3_1:
                {
                    gearing = 3;
                    break;
                }
                case (uint8_t)config_defs::gearing::gearing_4_5_1:
                {
                    gearing = 4.5;
                    break;
                }
                default:
                {
                    gearing = 1;
                    break;
                }
            }

            if ((config_to_send[config_defs::arm_1_flip_motor_dir_idx] == (uint8_t)config_defs::flip_motor_dir::both) || ((config_to_send[config_defs::arm_1_flip_motor_dir_idx] == (uint8_t)config_defs::flip_motor_dir::left) && this->is_left) || ((config_to_send[config_defs::arm_1_flip_motor_dir_idx] == (uint8_t)config_defs::flip_motor_dir::right) && (!this->is_left)))
            {
                this->flip_direction = 1;
            }
            else
            {
                this->flip_direction = 0;
            }
            break;
        }
        case (uint8_t)config_defs::joint_id::arm_2:
        {
            motor_type = config_to_send[config_defs::arm_2_idx];

            switch (config_to_send[config_defs::arm_2_gear_idx])
            {
                case (uint8_t)config_defs::gearing::gearing_1_1:
                {
                    gearing = 1;
                    break;
                }
                case (uint8_t)config_defs::gearing::gearing_2_1:
                {
                    gearing = 2;
                    break;
                }
                case (uint8_t)config_defs::gearing::gearing_3_1:
                {
                    gearing = 3;
                    break;
                }
                case (uint8_t)config_defs::gearing::gearing_4_5_1:
                {
                    gearing = 4.5;
                    break;
                }
                default:
                {
                    gearing = 1;
                    break;
                }
            }

            if ((config_to_send[config_defs::arm_2_flip_motor_dir_idx] == (uint8_t)config_defs::flip_motor_dir::both) || ((config_to_send[config_defs::arm_2_flip_motor_dir_idx] == (uint8_t)config_defs::flip_motor_dir::left) && this->is_left) || ((config_to_send[config_defs::arm_2_flip_motor_dir_idx] == (uint8_t)config_defs::flip_motor_dir::right) && (!this->is_left)))
            {
                this->flip_direction = 1;
            }
            else
            {
                this->flip_direction = 0;
            }
            break;
        }
    }
    
    //For AK-Series Motors Only
    p = 0;      //Read position
    v = 0;      //Read velocity
    i = 0;      //Read current
    p_des = 0;  
    v_des = 0;
    kp = 0;
    kd = 0;
    t_ff = 0;
    last_command = 0;
    timeout_count = 0;
    pda_id = 0;
    pda_control_mode = (uint8_t)config_defs::pda_control_mode::torque_direct;
    pda_angle_deg = 0;
    pda_speed_rpm = 0;
    pda_torque_nm = 0;
    pda_voltage_v = 0;
    pda_current_a = 0;
    pda_torque_limit_nm = 0;
    pda_speed_limit_rpm = 0;
    pda_rated_torque_nm = 0;
    pda_peak_torque_nm = 0;
    pda_rated_speed_rpm = 0;
    pda_rated_current_a = 0;
    pda_stall_current_a = 0;
    pda_rotor_inertia_gcm2 = 0;
    pda_last_feedback_us = 0;
    pda_feedback_valid = false;
    motor_data_load_pda_config(this, config_to_send);
};
