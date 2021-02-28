/*!
 * @file devive.c
 *
 * Contains implementations of functions from device.h
 */

#include "device.h"

/****************************************************************************************/
/*                                                                                      */
/*                            Static functions declarations                             */
/*                                                                                      */
/****************************************************************************************/

static void init_icm( icm20600 *icm_instance );
static void init_nrf_as_tx( nrf24l01p *nrf_instance, uint8_t tx_address[] );
static void update_robot_mode(balancing_robot *robot_instance);

static void handle_message_in_balancing_mode(balancing_robot *robot, uint8_t message[]);
static void handle_message_in_horizontal_modes(balancing_robot *robot, uint8_t message[]);

/****************************************************************************************/
/*                                                                                      */
/*                          Static functions implementations                            */
/*                                                                                      */
/****************************************************************************************/

static void init_icm( icm20600 *icm_instance )
{
    add_mistake_to_the_log( icm20600_reset_all_registeres( icm_instance ) );
    delay_in_milliseconds( 10 );
    add_mistake_to_the_log( icm20600_init( icm_instance ) );
//    add_mistake_to_the_log( icm20600_disable_one_gyro_channel( icm_instance, icm_y_axis_index ) );
//    add_mistake_to_the_log( icm20600_disable_one_gyro_channel( icm_instance, icm_z_axis_index ) );
}

static void init_nrf_as_tx( nrf24l01p *nrf_instance, uint8_t tx_address[] )
{
    add_mistake_to_the_log( nrf24_basic_init( nrf_instance ) );
    add_mistake_to_the_log( nrf24_enable_pipe1( nrf_instance, tx_address ) );
    add_mistake_to_the_log( nrf24_rx_mode( nrf_instance ) );
}

// очень умное решение, на которые наравлся абсолютно случайно. Если проверять только одну границу условия, то даже при реальной скорости изменения состояиня
// На противоположное, такое изменение не произойдет, а произойдет только изменение на "соседнее состояние" из которого уже при необходимости может произойти изменение d
// в следующее, противоположное первому состояние.
static void update_robot_mode( balancing_robot *robot_instance )
{
    switch ( robot_instance->mode )
    {
        case ROBOT_BALANCING: // balancing around 0 degrees
        {
            if ( robot_instance->current_angle < -(SWITCH_ANGLE + SWITCH_HYSTERESYS) )
            {
                robot_instance->mode = ROBOT_HORIZONTAL_BOTTOM;
                reset_control_system(robot_instance);
            }
            else if ( robot_instance->current_angle > (SWITCH_ANGLE + SWITCH_HYSTERESYS) )
            {
                robot_instance->mode = ROBOT_HORIZONTAL_TOP;
                reset_control_system(robot_instance);
            }

            break;
        }
        case ROBOT_HORIZONTAL_TOP: // between 0 and +180 degrees
        {
            if ( robot_instance->current_angle < (SWITCH_ANGLE - SWITCH_HYSTERESYS) && robot_instance->current_angle >= -SWITCH_ANGLE)
            {
                robot_instance->mode = ROBOT_BALANCING;
            }
            else if ( robot_instance->current_angle > 180 - (SWITCH_ANGLE - SWITCH_HYSTERESYS) || robot_instance->current_angle <= -180 + SWITCH_ANGLE)
            {
                robot_instance->mode = ROBOT_IDLE;
                robot_instance->left_motor->set_pwm_duty_cycle( 0 );
                robot_instance->right_motor->set_pwm_duty_cycle( 0 );
            }
            else if ( robot_instance->current_angle < -(SWITCH_ANGLE) && robot_instance->current_angle > -180 + SWITCH_ANGLE )
            {
                robot_instance->mode = ROBOT_HORIZONTAL_BOTTOM;
                robot_instance->left_motor->set_pwm_duty_cycle( 0 );
                robot_instance->right_motor->set_pwm_duty_cycle( 0 );
            }
            break;
        }
        case ROBOT_HORIZONTAL_BOTTOM: // between 0 and -180 degrees
        {
            if ( robot_instance->current_angle > -(SWITCH_ANGLE - SWITCH_HYSTERESYS) && robot_instance->current_angle <= SWITCH_ANGLE )
            {
                robot_instance->mode = ROBOT_BALANCING;
            }
            else if ( robot_instance->current_angle < -180 + (SWITCH_ANGLE - SWITCH_HYSTERESYS) || robot_instance->current_angle >= 180 - SWITCH_ANGLE  )
            {
                robot_instance->mode = ROBOT_IDLE;
                robot_instance->left_motor->set_pwm_duty_cycle( 0 );
                robot_instance->right_motor->set_pwm_duty_cycle( 0 );
            }
            else if ( robot_instance->current_angle > SWITCH_ANGLE && robot_instance->current_angle < 180 - SWITCH_ANGLE )
            {
                robot_instance->mode = ROBOT_HORIZONTAL_TOP;
                robot_instance->left_motor->set_pwm_duty_cycle( 0 );
                robot_instance->right_motor->set_pwm_duty_cycle( 0 );
            }
            break;
        }
        case ROBOT_IDLE: // balancing on the other side around -180/180 degrees
        {
            if ( robot_instance->current_angle < 180 - (SWITCH_ANGLE + SWITCH_HYSTERESYS) && robot_instance->current_angle >= SWITCH_ANGLE)
            {
                robot_instance->mode = ROBOT_HORIZONTAL_TOP;
            }
            else if ( robot_instance->current_angle > -180 + (SWITCH_ANGLE + SWITCH_HYSTERESYS) && robot_instance->current_angle <= -SWITCH_ANGLE)
            {
                robot_instance->mode = ROBOT_HORIZONTAL_BOTTOM;
            }
            else if ( robot_instance->current_angle < SWITCH_ANGLE && robot_instance->current_angle > -SWITCH_ANGLE )
            {
                robot_instance->mode = ROBOT_BALANCING;
            }
            break;
        }
        default:
            break;
    }
}

static void handle_message_in_balancing_mode(balancing_robot *robot, uint8_t message[])
{
    uint8_t right_joystick_top_bottom = (message[0] & 0x03);
    uint8_t left_joystick_left_right = (message[0] & 0x30) >> 4;

    if ( right_joystick_top_bottom == 1 )
    {
        robot->target_linear_speed = -0.8f;
    }
    else if ( right_joystick_top_bottom == 2 )
    {
        robot->target_linear_speed = 0.8f;
    }
    else
    {
        robot->target_linear_speed = 0.0f;
    }

    if ( left_joystick_left_right == 1 )
    {
        if ( robot->target_rotational_speed > -0.5f )
        {
            robot->rotation_integral = 0.0f;
            robot->target_rotational_speed = -0.6f;
        }
    }
    else if ( left_joystick_left_right == 2 )
    {
        if ( robot->target_rotational_speed < 0.5f )
        {
            robot->rotation_integral = 0.0f;
            robot->target_rotational_speed = 0.6f;
        }
    }
    else
    {
        if ( robot->target_rotational_speed > 0.1f || robot->target_rotational_speed < -0.1f )
        {
            robot->rotation_integral = 0.0f;
            robot->target_rotational_speed = 0.0f;
        }
    }
}

static void handle_message_in_horizontal_modes( balancing_robot *robot, uint8_t message[] )
{
    uint8_t right_joystick_top_bottom = (message[0] & 0x03);
    uint8_t left_joystick_left_right = (message[0] & 0x30) >> 4;

    if ( right_joystick_top_bottom == 1 )
    {
        if( robot->current_angle > 180 - (SWITCH_ANGLE + 3.5f*SWITCH_HYSTERESYS) || robot->current_angle < -180 + (SWITCH_ANGLE + 3.5f*SWITCH_HYSTERESYS) ||
                    (robot->current_angle < (SWITCH_ANGLE + 3.5f*SWITCH_HYSTERESYS) && robot->current_angle > -(SWITCH_ANGLE + 3.5f*SWITCH_HYSTERESYS)))
        {
            robot->right_motor->speed_controller->target_speed = 0.45f;
            robot->left_motor->speed_controller->target_speed = 0.45f;
        }
        else
        {
            robot->right_motor->speed_controller->target_speed = 1.2f;
            robot->left_motor->speed_controller->target_speed = 1.2f;
        }
    }
    else if ( right_joystick_top_bottom == 2 )
    {
        if( robot->current_angle > 180 - (SWITCH_ANGLE + 3.5f*SWITCH_HYSTERESYS) || robot->current_angle < -180 + (SWITCH_ANGLE + 3.5f*SWITCH_HYSTERESYS) ||
                    (robot->current_angle < (SWITCH_ANGLE + 3.5f*SWITCH_HYSTERESYS) && robot->current_angle > -(SWITCH_ANGLE + 3.5f*SWITCH_HYSTERESYS)))
        {
            robot->right_motor->speed_controller->target_speed = -0.45f;
            robot->left_motor->speed_controller->target_speed = -0.45f;
        }
        robot->right_motor->speed_controller->target_speed = -1.2f;
        robot->left_motor->speed_controller->target_speed = -1.2f;
    }
    else if ( left_joystick_left_right == 1 )
    {
        robot->right_motor->speed_controller->target_speed = 0.8f;
        robot->left_motor->speed_controller->target_speed = -0.8f;
    }
    else if ( left_joystick_left_right == 2 )
    {
        robot->right_motor->speed_controller->target_speed = -0.8f;
        robot->left_motor->speed_controller->target_speed = 0.8f;
    }
    else
    {
        robot->right_motor->speed_controller->current_integral = 0.0f;
        robot->left_motor->speed_controller->current_integral = 0.0f;
        robot->right_motor->speed_controller->target_speed = 0.0f;
        robot->left_motor->speed_controller->target_speed = 0.0f;
    }
}

/****************************************************************************************/
/*                                                                                      */
/*                          Global functions implementations                            */
/*                                                                                      */
/****************************************************************************************/

/*!
 * @brief Sets up all project-related peripherals but system timer, that is set up after all are finished.
 */
void init_robot( balancing_robot *robot_instance, uint8_t nrf_tx_address[] )
{
    // Enable FPU.
    SCB->CPACR |= 0x3 << 20;

    /*! Every single time related parameter is based on SYSTEM_MAIN_FREQUENCY, thus clock be set up a soon as possible. */
    setup_system_clock();

    setup_timers();
    setup_leds();

    setup_motor1();
    setup_motor2();

    setup_nrf24l01p_peripherals();
    setup_icm20600_peripherals();

    delay_in_milliseconds( 100 );

    init_icm( robot_instance->icm );
    init_nrf_as_tx( robot_instance->nrf, nrf_tx_address );

    reset_control_system( robot_instance );
}


void dummy_delay(uint32_t duration)
{
    for(uint32_t i = 0; i < duration; ++i){}
}

void calibrate_icm20600_gyro( icm20600 *icm_instance, uint8_t calibration_coef, uint32_t cycle_length )
{
    uint32_t number_of_measurements = 1 << calibration_coef;
    int32_t calibration_array[3] = { 0, 0, 0 };

    if ( icm_instance->gyro_calibration_coefficients[0] != 0 || icm_instance->gyro_calibration_coefficients[1] != 0
         || icm_instance->gyro_calibration_coefficients[2] != 0 )
    {
        icm_instance->gyro_calibration_coefficients[0] = 0;
        icm_instance->gyro_calibration_coefficients[1] = 0;
        icm_instance->gyro_calibration_coefficients[2] = 0;
        icm20600_set_calibration_values( icm_instance );
    }

    for ( int32_t i = 0; i < number_of_measurements; ++i )
    {
        delay_in_milliseconds( cycle_length );
        icm20600_get_raw_data( icm_instance );
        calibration_array[icm_x_axis_index] -= icm_instance->raw_data[icm_gyro_x_index];
        calibration_array[icm_y_axis_index] -= icm_instance->raw_data[icm_gyro_y_index];
        calibration_array[icm_z_axis_index] -= icm_instance->raw_data[icm_gyro_z_index];
    }

    icm_instance->gyro_calibration_coefficients[icm_x_axis_index] = calibration_array[icm_x_axis_index] >> calibration_coef;
    icm_instance->gyro_calibration_coefficients[icm_y_axis_index] = calibration_array[icm_y_axis_index] >> calibration_coef;
    icm_instance->gyro_calibration_coefficients[icm_z_axis_index] = calibration_array[icm_z_axis_index] >> calibration_coef;
    icm20600_set_calibration_values( icm_instance );
}

void calculate_starting_angle(balancing_robot *robot_instance, uint32_t cycle_length)
{
    int32_t accel_values[2] = { 0, 0 };

    for ( uint32_t i = 0; i < 8; ++i )
    {
        delay_in_milliseconds( cycle_length );
        icm20600_get_raw_data( robot_instance->icm );
        accel_values[0] += robot_instance->icm->raw_data[icm_accel_y_index];
        accel_values[1] += robot_instance->icm->raw_data[icm_accel_z_index];
    }
    float temp_processed_data[7];
    icm20600_process_raw_data(robot_instance->icm, temp_processed_data);

    accel_values[0] >>= 3;
    accel_values[1] >>= 3;

    robot_instance->current_angle = atan2f( temp_processed_data[icm_accel_z_index], -1 * temp_processed_data[icm_accel_y_index] ) * 57.296f;

    update_robot_mode(robot_instance);
}

// Вычисляет только нужный угол из трех. Может вызываться без вызова каких либо других функций, так как будет считывать значения датчика самостоятельно
void calculate_x_angle( balancing_robot *robot_instance, float *processed_values, float integration_period )
{
    // Calculations, unique for every plane
    float accel_based_value = atan2f( processed_values[icm_accel_z_index], -1 * processed_values[icm_accel_y_index] ) * 57.296f;
    float gyro_intergral_value = robot_instance->current_angle;
    gyro_intergral_value += (robot_instance->icm->previous_gyro_values[icm_x_axis_index] + processed_values[icm_gyro_x_index]) / 2.0f * integration_period;

    if( abs((int)(gyro_intergral_value - accel_based_value)) > 220)
    {
        if ( gyro_intergral_value > 0.0f )
        {
            gyro_intergral_value -= 360.0f;
        }
        else
        {
            gyro_intergral_value += 360.0f;
        }
    }

    robot_instance->icm->previous_gyro_values[icm_x_axis_index] = processed_values[icm_gyro_x_index]; // For trapezoidal integration

    robot_instance->previous_angle = robot_instance->current_angle;
    robot_instance->current_angle = accel_based_value * robot_instance->icm->complementary_filter_coef
                + gyro_intergral_value * (1.0f - robot_instance->icm->complementary_filter_coef);

    update_robot_mode(robot_instance);
}

// Старая функция, будет использоваться только для тестирования параметров гироскопа и акселерометра в целом.
float calculate_x_angle_2(icm20600 *icm_instance, float *processed_values, float *gyro_intergral_value, float *accel_based_value, float *buffer, float integration_period)
{
    float return_value = 0.0f;

    // Calculations, unique for every plane
    *accel_based_value = atan2f( processed_values[icm_accel_z_index], -1 * processed_values[icm_accel_y_index] ) * 57.296f;


    *gyro_intergral_value += (icm_instance->previous_gyro_values[icm_x_axis_index] + processed_values[icm_gyro_x_index]) / 2.0f * integration_period;

    if(*gyro_intergral_value > 180.0f)
    {
        *gyro_intergral_value -= 360.0f;
    }
    if(*gyro_intergral_value < -180.0f)
    {
        *gyro_intergral_value += 360.0f;
    }

    if (icm_instance->gyro_faults[icm_x_axis_index])
    {
        buffer[3] = buffer[2];
        buffer[2] = buffer[1];
        buffer[1] = buffer[0];
        buffer[0] = *accel_based_value;

        float buffer_average = (buffer[0] + buffer[1] + buffer [2] + buffer [3])/4.0f;
        if(buffer_average - buffer[0] < 0.5f && buffer_average - buffer[0] > -0.5f)
        {
            buffer[0] = 0;
            buffer[1] = 0;
            buffer[2] = 0;
            buffer[3] = 0;
            *gyro_intergral_value = buffer_average;
            icm_instance->gyro_faults[icm_x_axis_index] = 0;
        }
    }

    icm_instance->previous_gyro_values[icm_x_axis_index] = processed_values[icm_gyro_x_index]; // For trapezoidal integration

//    return_value = accelerometer_based_angle * icm_instance->complementary_filter_coef + gyro_intergral_value * (1.0f - icm_instance->complementary_filter_coef);
    return_value = *accel_based_value * icm_instance->complementary_filter_coef + *gyro_intergral_value*(1.0f - icm_instance->complementary_filter_coef);

    return return_value;
}



void handle_angle_loop(balancing_robot *robot_instance, float integration_period)
{
    if (robot_instance->mode != ROBOT_BALANCING)
    {
        return;
    }

    float mistake = robot_instance->target_angle - robot_instance->current_angle;

    robot_instance->angle_integral += (mistake + robot_instance->previous_angle_mistake) / 2.0f * integration_period * robot_instance->angle_ki;
    if ( robot_instance->angle_integral > 1000.0f )
    {
        robot_instance->angle_integral = 1000.0f;
    }
    else if ( robot_instance->angle_integral < -1000.0f )
    {
        robot_instance->angle_integral = -1000.0f;
    }
    robot_instance->previous_angle_mistake = mistake;

    robot_instance->angle_p_part = mistake * robot_instance->angle_kp;
    robot_instance->angle_d_part = robot_instance->icm->raw_data[icm_gyro_x_index] * robot_instance->angle_kd;

    robot_instance->regulator_control_signal = robot_instance->angle_p_part + robot_instance->angle_integral + robot_instance->angle_d_part;

//   robot_instance->left_motor->set_pwm_duty_cycle( robot_instance->regulator_control_signal );
//   robot_instance->right_motor->set_pwm_duty_cycle( robot_instance->regulator_control_signal );
    robot_instance->left_motor->set_pwm_duty_cycle( robot_instance->regulator_control_signal + robot_instance->rotational_task);
    robot_instance->right_motor->set_pwm_duty_cycle( robot_instance->regulator_control_signal - robot_instance->rotational_task);
}

/*!
 * Handles both linear and rotational movement loops.
 *
 * @note Function is written a bit messy an probably should be disassembled into two additional functions.
 */
void handle_speed_loops(balancing_robot *robot, float integration_period)
{
    motor_get_speed_by_incements(robot->left_motor, integration_period);
    motor_get_speed_by_incements(robot->right_motor, integration_period);

    robot->current_linear_speed = (robot->left_motor->speed_controller->current_speed + robot->right_motor->speed_controller->current_speed) / 2.0;
    robot->current_rotational_speed = (robot->left_motor->speed_controller->current_speed - robot->right_motor->speed_controller->current_speed) / 2.0;


    if(robot->mode == ROBOT_HORIZONTAL_BOTTOM || robot->mode == ROBOT_HORIZONTAL_TOP)
    {
        int16_t left_m_voltage_target = motors_speed_controller_handler(robot->left_motor, integration_period);
        int16_t right_m_voltage_target = motors_speed_controller_handler(robot->right_motor, integration_period);

        // this is a wrong thing to do because it breaks encapsulation and makes code hard to update, but it is an easy solution and will do.
        robot->left_motor->set_pwm_duty_cycle(left_m_voltage_target);
        robot->right_motor->set_pwm_duty_cycle(right_m_voltage_target);

        return;
    }
    else if (robot->mode == ROBOT_IDLE)
    {
        return;
    }

    float linear_mistake = robot->target_linear_speed - robot->current_linear_speed;
    float rotational_mistake = robot->target_rotational_speed - robot->current_rotational_speed;

    // linear speed loop handler
    robot->speed_integral += (linear_mistake + robot->previous_linear_speed_mistake) / 2.0f * integration_period;
    robot->speed_i_part = robot->speed_integral * robot->speed_ki;
    if ( robot->speed_i_part > 2.0f )
    {
        robot->speed_i_part = 2.0f;
    }
    else if ( robot->speed_i_part < -2.0f )
    {
        robot->speed_i_part = -2.0f;
    }
    robot->previous_linear_speed_mistake = linear_mistake;

    // rotational speed loop handler
    robot->rotation_integral += (rotational_mistake + robot->previous_rotational_mistake) / 2.0f * integration_period;
    robot->rotation_i_part = robot->rotation_integral * robot->rotation_ki;
    if(robot->rotation_i_part > 300.0f)
    {
        robot->rotation_i_part = 300.0f;
    }
    else if (robot->rotation_i_part < -300.0f)
    {
        robot->rotation_i_part = -300.0f;
    }
    robot->previous_rotational_mistake = rotational_mistake;

    robot->rotation_p_part = rotational_mistake * robot->rotation_kp;
    robot->rotational_task = (int16_t)(robot->rotation_p_part + robot->rotation_i_part);

    robot->speed_p_part = linear_mistake*robot->speed_kp;
    robot->target_angle = robot->zero_angle - ( robot->speed_p_part + robot->speed_i_part);
}

void reset_control_system ( balancing_robot *robot)
{
    robot->target_angle = robot->zero_angle;
    robot->angle_integral = 0.0f;
    robot->previous_angle_mistake = 0.0f;

    robot->speed_integral = 0.0f;
    robot->target_linear_speed = 0.0f;

    robot->rotation_integral = 0.0f;
    robot->target_rotational_speed = 0.0f;

    robot->previous_linear_speed_mistake = 0.0f;
    robot->regulator_control_signal = 0;

    robot->left_motor->speed_controller->current_integral = 0.0f;
    robot->left_motor->speed_controller->target_speed = 0.0f;
    robot->right_motor->speed_controller->current_integral = 0.0f;
    robot->right_motor->speed_controller->target_speed = 0.0f;

    robot->left_motor->set_pwm_duty_cycle(robot->regulator_control_signal);
    robot->right_motor->set_pwm_duty_cycle(robot->regulator_control_signal);

    robot->target_rotational_speed = 0.0f;
}

void handle_nrf_message (balancing_robot *robot, uint8_t message[])
{
    if (robot->mode == ROBOT_BALANCING)
    {
        handle_message_in_balancing_mode(robot, message);
    }
    else if (robot->mode == ROBOT_HORIZONTAL_TOP || robot->mode == ROBOT_HORIZONTAL_BOTTOM)
    {
        handle_message_in_horizontal_modes(robot, message);
    }

}


// EOF
