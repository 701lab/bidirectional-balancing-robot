 /*!
 * @file main.c
 */

#define DECLARE_GLOBAL_VARIABLES
#include "device.h"
#include "basic_tests.h"

void encode_float_variable(float value_to_encode,  uint8_t array_to_store_result[]);
float decode_float_variable(uint8_t array_with_value[]);

/****************************************************************************************/
/*                                                                                      */
/*                           Global parameters initialization                           */
/*                                                                                      */
/****************************************************************************************/

nrf24l01p robot_nrf = {
                        .spi_write_byte = spi3_write_single_byte,
                        .ce_high = nrf24_set_ce_high,
                        .ce_low = nrf24_set_ce_low,
                        .csn_high = nrf24_set_cs_high,
                        .csn_low = nrf24_set_cs_low,

                        .frequency_channel = 45,
                        .power_output = nrf24_pa_max,
                        .data_rate = nrf24_250_kbps,

                        .payload_size_in_bytes = 6,

                        .was_initialized = 0,
};

uint8_t nrf_tx_address[5] = { 0x11, 0x22, 0x33, 0x44, 0x66 }; // blue-LED controller
uint8_t nrf_input_data[6] = { 0, 0, 0, 0, 0, 0 };
uint32_t nrf_watchdog_counter = SYSTICK_INTERRUPT_FREQUENCY; // 1 second watchdog


icm20600 robot_icm =
                          {
                            .spi_write_byte = spi2_write_single_byte,
                            .set_cs_high = set_icm20600_cs_high,
                            .set_cs_low = set_icm20600_cs_low,

//                            .gyro_averaging_setup = icm_gyro_1_sample_averaging,
                            .gyro_averaging_setup = icm_gyro_8_samples_averaging,
                            .gyro_filter_bw = icm_gyro_92_hz,
                            .gyro_scale_setup = icm_gyro_1000dps_scale,

                            .accel_averaging_setup = icm_accel_4_samples_averaging,
                            .accel_filter_bw = icm_accel_99_hz,
                            .accel_scale_setup = icm_accel_2g_scale,

                            .sample_rate_divider = 7,

                            .complementary_filter_coef = 0.15f,
                            .enable_temperature_sensor = 0,

                            .raw_data = { 0, 0, 0, 0, 0, 0, 0 },
                            .previous_gyro_values = { 0, 0, 0 },

#ifdef BOARD_1
                            .gyro_calibration_coefficients = { 103, -63, 6 },
#endif
#ifdef BOARD_2
                            .gyro_calibration_coefficients = { -32, -68, -23 },
#endif

                            .was_initialized = 0
                          };

float icm_processed_data[7] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };

motor_speed_controler m1_spd_controller =
                                          {
                                            .controller_output_limitat = PWM_PRECISION,
                                            .ki = 750.0f,
                                            .kp = 200.0f,

                                            .previous_encoder_counter_value = 0,

                                            .current_integral = 0.0f,
                                            .current_speed = 0.0f,
                                            .target_speed = 0.0f,

                                            .previous_speed_mistake = 0.0f,
                                            .regulator_control_signal = 0.0f,
                                          };

motor_speed_controler m2_spd_controller =
                                          {
                                            .controller_output_limitat = PWM_PRECISION,
                                            .ki = 750.0f,
                                            .kp = 200.0f,

                                            .previous_encoder_counter_value = 0,

                                            .current_integral = 0.0f,
                                            .current_speed = 0.0f,
                                            .target_speed = 0.0f,

                                            .previous_speed_mistake = 0.0f,
                                            .regulator_control_signal = 0.0f,
                                          };

motor motor1 =
               {
                 .disable = set_drv8701_enable_low,
                 .enable = set_drv8701_enable_high,
                 .get_encoder_counter_val = get_encoder1_value,
                 .set_pwm_duty_cycle = set_drv8701_1_duty_cycle,

                 .encoder_constant = 937.2f,
                 .max_duty_cycle = PWM_PRECISION,

                 .speed_controller = &m1_spd_controller,
                 .position_controller = 0,
               };

motor motor2 =
               {
                 .disable = set_drv8701_enable_low,
                 .enable = set_drv8701_enable_high ,
                 .get_encoder_counter_val = get_encoder2_value,
                 .set_pwm_duty_cycle = set_drv8701_2_duty_cycle,

                 .encoder_constant = 937.2f,
                 .max_duty_cycle = PWM_PRECISION,

                 .speed_controller = &m2_spd_controller,
                 .position_controller = 0,
               };

/****************************************************************************************/
/*                                                                                      */
/*                               Control loop parameters                                */
/*                                                                                      */
/****************************************************************************************/

balancing_robot robot =
                        {
                          .speed_integral = 0.0f,
                          .speed_integral_limit = PWM_PRECISION,
                          .current_linear_speed = 0.0f,
                          .current_rotational_speed = 0.0f,
                          .previous_linear_speed_mistake = 0.0f,

                          .target_linear_speed = 0.0f,
                          .target_rotational_speed = 0.0f,

                          .speed_kp = 1.2f, //1.2, 1.3 ,70. 700
                          .speed_ki = 1.0f, //1.0-1.1 good: 0.3  160 360 0

                          .angle_kp = 70.0f, // 65 //80
                          .angle_ki = 620.0f, // 800 //72
                          .angle_kd = 0.00f,

                          .rotation_kp = 50.0f, // needs testing
                          .rotation_ki = 150.0f, // needs even more testing
                          .rotation_integral = 0.0f,
                          .rotation_p_part = 0.0f,
                          .previous_rotational_mistake = 0.0f,

                          .angle_integral = 0.0f,
                          .controller_output_limitat = 0.0f,
                          .previous_angle_mistake = 0.0f,
                          .regulator_control_signal = 0.0f,

                          .zero_angle = 0.8f,
                          .target_angle = 0.8f,

                          .current_angle = 0.0f,
                          .previous_angle = 0.0f,
                          .mode = ROBOT_IDLE,

                          .left_motor = &motor2,
                          .right_motor = &motor1,
                          .icm = &robot_icm,
                          .nrf = &robot_nrf,
                        };


/****************************************************************************************/
/*                                                                                      */
/*                                     Main function                                    */
/*                                                                                      */
/****************************************************************************************/

int main( void )
{
    init_robot( &robot, nrf_tx_address );
    calculate_starting_angle( &robot, 20 );

//    calibrate_icm20600_gyro(robot.icm, 10, 20); // to calibrate IMU values.
    setup_system_timer();

    robot.left_motor->enable();
    robot.right_motor->enable();

    while ( 1 )
    {
//        run_motors_triangle_movement_test(&motor1, &motor2);

        if ( nrf24_is_new_data_availiable( robot.nrf ) )
        {
            nrf24_read_message( robot.nrf, nrf_input_data, 6 );
            handle_nrf_message(&robot, nrf_input_data);
            nrf_watchdog_counter = SYSTICK_INTERRUPT_FREQUENCY;
            toggle_d4_led();
        }
    }
}

/****************************************************************************************/
/*                                                                                      */
/*                              System interrupt handler                                */
/*                                                                                      */
/****************************************************************************************/

uint32_t system_counter = 0;

const uint32_t angle_loop_freq = SYSTICK_INTERRUPT_FREQUENCY;
uint32_t angle_loop_counter = 0;

const uint32_t speed_loop_freq = SYSTICK_INTERRUPT_FREQUENCY / 2;
uint32_t speed_loop_counter = 0;

void SysTick_Handler()
{
    // Simple blink just to show, that the board is online.
    system_counter += 1;
    if ( system_counter == SYSTICK_INTERRUPT_FREQUENCY / 2 )
    {
        toggle_d2_led();
        system_counter = 0;
    }

    // Nrf watchdog.
    nrf_watchdog_counter -= 1;
    if(nrf_watchdog_counter == 0)
    {
        if(robot.target_linear_speed > 0.1f || robot.target_linear_speed < -0.1f)
        {
            robot.speed_integral = 0.0f;
        }
        robot.target_linear_speed = 0.0f;

        if(robot.target_rotational_speed > 0.1f || robot.target_rotational_speed < -0.1f)
        {
            robot.rotation_integral = 0.0f;
        }
        robot.target_rotational_speed = 0.0f;
    }

    // Control loops
    speed_loop_counter += 1;
    if ( speed_loop_counter == (SYSTICK_INTERRUPT_FREQUENCY / speed_loop_freq) )
    {
        handle_speed_loops(&robot, 1.0f/ (float)(speed_loop_freq));
        speed_loop_counter = 0;
    }

    angle_loop_counter += 1;
    if ( angle_loop_counter == (SYSTICK_INTERRUPT_FREQUENCY / angle_loop_freq) )
    {
        add_mistake_to_the_log( icm20600_get_raw_data( &robot_icm ) );
        add_mistake_to_the_log( icm20600_process_raw_data( &robot_icm, icm_processed_data ) );
//        robot_measured_angle_2 = calculate_x_angle_2( &robot_icm20600, icm_processed_data, &gyro_angle_integral, &accel_measured_angle, accel_buffer, 1.0f / (float)(SYSTICK_INTERRUPT_FREQUENCY) );

        calculate_x_angle( &robot, icm_processed_data, 1.0f / (float)(angle_loop_freq) );
        handle_angle_loop( &robot, 1.0f / (float)(angle_loop_freq) );
        angle_loop_counter = 0;
    }
}

