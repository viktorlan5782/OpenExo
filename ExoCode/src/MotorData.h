/**
 * @file MotorData.h
 *
 * @brief Declares a class used to store data for motors to access 
 * 
 * @author P. Stegall 
 * @date Jan. 2022
*/

#ifndef MotorData_h
#define MotorData_h

#include "Arduino.h"

#include "ParseIni.h"
#include "Board.h"

#include <stdint.h>

//Forward declaration
class ExoData;

class MotorData 
{
	public:
        MotorData(config_defs::joint_id id, uint8_t* config_to_send);
        
        /**
         * @brief reconfigures the the motor data if the configuration changes after constructor called.
         * 
         * @param configuration array
         */
        void reconfigure(uint8_t* config_to_send);
        
        config_defs::joint_id id;   /**< Motor id, should be the same as the joint id. */ 
        uint8_t motor_type;         /**< Type of motor being used. */
        float last_command;         /**< Last command sent to the motor. */
        float p;                    /**< Read position. */ 
        float v;                    /**< Read velocity. */
        float i;                    /**< Read current. */
        float kt;                   /**< Motor torque constant. */
        float p_des = 0;            /**< Desired position, not currently used but available for position control */
        float v_des = 0;            /**< Desired velocity, not currently used but available for velocity control */
        float kp = 0;               /**< Proportional gain */
        float kd = 0;               /**< Derivative gain */
        float t_ff = 0;             /**< Torque command */
        
        bool do_zero;               /**< Flag to zero the position of the motor */
        bool enabled;               /**< Motor enable state*/
        bool is_on;                 /**< Motor power state */
        bool is_left;               /**< Motor side information 1 if on the left, 0 otherwise */
        bool flip_direction;        /**< Should the motor direction be flipped, if true torque commands and position/velocity information will be inverted */
        float gearing;              /**< Motor gearing used to convert motor position, velocity, and torque between the motor and joint frames. */

        //Timeout state
        int timeout_count;          /**< Number of timeouts in a row */
        int timeout_count_max = 40; /**< Number of timeouts in a row before the motor is disabled */

        //PDA-series feedback state. Native PDA units are kept here; p/v/i mirror legacy OpenExo units.
        uint8_t pda_id;             /**< PDA CAN node ID, not the OpenExo joint_id bitmask. */
        uint8_t pda_control_mode;   /**< Default PDA control mode selected in config.ini. */
        float pda_angle_deg;        /**< PDA output-shaft angle feedback in degrees. */
        float pda_speed_rpm;        /**< PDA output-shaft speed feedback in revolutions per minute. */
        float pda_torque_nm;        /**< PDA output torque feedback in Nm. */
        float pda_voltage_v;        /**< PDA bus voltage feedback in V, when explicitly read. */
        float pda_current_a;        /**< PDA q-axis current feedback in A, when explicitly read. */
        float pda_torque_limit_nm;  /**< Software torque saturation for PDA command in Nm. */
        float pda_speed_limit_rpm;  /**< Software speed saturation for PDA native modes in rpm. */
        float pda_rated_torque_nm;  /**< Rated continuous torque in Nm. */
        float pda_peak_torque_nm;   /**< Peak/stall torque in Nm. */
        float pda_rated_speed_rpm;  /**< Rated speed in rpm. */
        float pda_rated_current_a;  /**< Rated phase/current-loop value in A. */
        float pda_stall_current_a;  /**< Stall current in A. */
        float pda_rotor_inertia_gcm2; /**< Rotor inertia in g*cm^2. */
        uint32_t pda_last_feedback_us; /**< Timestamp of the last decoded PDA feedback frame. */
        bool pda_feedback_valid;    /**< True after at least one valid PDA feedback frame. */
		
		//Real-time Maxon Motor Reset Feedback
		int maxon_plotting_scalar = 1;

};


#endif
