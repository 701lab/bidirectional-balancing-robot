/*!
 * @file device.h
 *
 * Contains the main, system related definitions and function declarations.
 *  Unlike interfacing libraries, this file can be freely updated and changed in order to get the desired behavior.
 */

#ifndef DEVICE_H_
#define DEVICE_H_

// Interfacing includes
#include "itf_gpio.h"
#include "itf_clocks.h"
#include "itf_drv8701.h"
#include "itf_icm20600.h"
#include "itf_nrf24l01p.h"

// Hardware libraries includes
#include "icm20600_itf_in.h"
#include "drv8701_itf_in.h"
#include "nrf24l01p_impi.h" // Old Fashioned library.
#include "motors.h" // Super old fashioned library.

// Other includes
#include "math.h"
#include "stdlib.h"


//#define BOARD_1
#define BOARD_2

#define SWITCH_ANGLE            (22.5f)
#define SWITCH_HYSTERESYS       (2.5f)


typedef enum robot_mode
{
    ROBOT_BALANCING = 0,
    ROBOT_HORIZONTAL_TOP = 1,
    ROBOT_HORIZONTAL_BOTTOM = 2,
    ROBOT_IDLE = 3,
} robot_mode;

typedef struct
{
    float angle_kp;
    float angle_ki;
    float angle_kd;
    float angle_integral;

    float speed_kp;
    float speed_ki;
    float speed_integral;
    float speed_integral_limit;
    float target_linear_speed;
    float current_linear_speed;
    float current_rotational_speed;
    float previous_linear_speed_mistake;
    float target_rotational_speed;

    float zero_angle;
    float target_angle;
    float current_angle;
    float previous_angle; // for mode calculations
    float previous_angle_mistake;
    float controller_output_limitat;
    int16_t regulator_control_signal;

    robot_mode mode;

    motor *left_motor;
    motor *right_motor;
    icm20600 *icm;
    nrf24l01p *nrf;

} balancing_robot;

//! Sets up all project related peripherals.
void init_robot( balancing_robot *robot_instance, uint8_t nrf_tx_address[] );

void dummy_delay( uint32_t duration );

void calibrate_icm20600_gyro( icm20600 *icm_instance, uint8_t calibration_coef, uint32_t cycle_length );

void calculate_base_angle( balancing_robot *robot_instance, uint32_t cycle_length );

void calculate_x_angle( balancing_robot *robot_instance, float *processed_values, float integration_period );
float calculate_x_angle_2( icm20600 *icm_instance, float *processed_values, float *gyro_intergral_value, float *accel_based_value, float *buffer,
                           float integration_period );

void handle_angle_loop( balancing_robot *robot_instance, float integration_period );
void handle_speed_loop( balancing_robot *robot_instance, float integration_period );

void reset_control_system ( balancing_robot *robot);

#endif /* DEVICE_H_ */
