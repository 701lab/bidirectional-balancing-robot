/*!
 * @file main.c
 */

#define DECLARE_GLOBAL_VARIABLES
#include "device.h"

#define BOARD_1
//#define BOARD_2




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

                        .frequency_channel = 90,
                        .power_output = nrf24_pa_max,
                        .data_rate = nrf24_250_kbps,

                        .payload_size_in_bytes = 6,

                        .was_initialized = 0,
};

uint8_t nrf_rx_address[5] = { 0x11, 0x22, 0x33, 0x44, 0x66 }; // blue-LED controller
uint8_t nrf_input_data[6] = { 0, 0, 0, 0, 0, 0 };

icm20600 robot_icm20600 =
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

                            .complementary_filter_coef = 0.1f,
                            .enable_temperature_sensor = 0,

                            .raw_data = { 0, 0, 0, 0, 0, 0, 0 },
                            .previous_gyro_values = { 0, 0, 0 },

#ifdef BOARD_1
//                            .gyro_calibration_coefficients = { 0, 0, 0 },
                            .gyro_calibration_coefficients = { 104, -62, 5 },
#endif
#ifdef BOARD_2
                            .gyro_calibration_coefficients = { 0, 0, 0 },
#endif

                            .was_initialized = 0
                          };

float icm_processed_data[7] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };

motor motor1 =
               {
                 .disable = set_drv8701_enable_low,
                 .enable = set_drv8701_enable_high,
                 .get_encoder_value = get_encoder1_value,
                 .set_pwm_duty_cycle = set_drv8701_1_duty_cycle,

                 .encoder_constant = 937.2f,
                 .max_duty_cycle = PWM_PRECISION,
               };

motor motor2 =
               {
                 .disable = set_drv8701_enable_low,
                 .enable = set_drv8701_enable_high ,
                 .get_encoder_value = get_encoder2_value,
                 .set_pwm_duty_cycle = set_drv8701_2_duty_cycle,

                 .encoder_constant = 937.2f,
                 .max_duty_cycle = PWM_PRECISION,
               };

/****************************************************************************************/
/*                                                                                      */
/*                               Control loop parameters                                */
/*                                                                                      */
/****************************************************************************************/

float robot_measured_angle = 0.0f;
float robot_measured_angle_2 = 0.0f;
float gyro_angle_integral = 0.0f;
float accel_measured_angle = 0.0f;
float accel_buffer[4] = { 0.0f, 0.0f, 0.0f, 0.0f };

// ****** Motor  initialization ****** //
//motor1.speed_controller = &motor1_speed_cotroller;
//motor1.position_controller = &motor1_position_controller;
//motor1_speed_cotroller.kp = 220.0f;
//motor1_speed_cotroller.ki = 5000.0f;
//motor1_position_controller.kp = 2.0f;
//motor1_position_controller.position_precision = 4.0f/motor1.encoder_constant;

/****************************************************************************************/
/*                                                                                      */
/*                                     Main function                                    */
/*                                                                                      */
/****************************************************************************************/

//uint8_t array_with_result[4] = {0, 0, 0, 0};
//float result_val = 0.0f;

int main( void )
{
    setup_device();

    add_mistake_to_the_log( nrf24_basic_init( &robot_nrf ) );
    add_mistake_to_the_log( nrf24_enable_pipe1( &robot_nrf, nrf_rx_address ) );
    add_mistake_to_the_log( nrf24_rx_mode( &robot_nrf ) );

    add_mistake_to_the_log( icm20600_reset_all_registeres( &robot_icm20600 ) );
    delay_in_milliseconds(10);
    add_mistake_to_the_log( icm20600_init( &robot_icm20600 ) );
    add_mistake_to_the_log( icm20600_disable_one_gyro_channel( &robot_icm20600, icm_y_axis_index ) );
    add_mistake_to_the_log( icm20600_disable_one_gyro_channel( &robot_icm20600, icm_z_axis_index ) );

    robot_measured_angle = calculate_base_angle(&robot_icm20600, 20);
    gyro_angle_integral = robot_measured_angle;



//    calibrate_icm20600_gyro(&robot_icm20600, 10, 20);

    setup_system_timer();

    motor1.enable();
    motor2.enable();


    motor1.set_pwm_duty_cycle(PWM_PRECISION/2);
    motor2.set_pwm_duty_cycle(PWM_PRECISION/2);

    while ( 1 )
    {

//        for ( int32_t i = 0; i <= PWM_PRECISION; ++i )
//        {
//            motor1.set_pwm_duty_cycle(i);
//            motor2.set_pwm_duty_cycle(i);
//            dummy_delay(10000);
//        }
//        for (int32_t i = PWM_PRECISION; i >= -PWM_PRECISION; --i)
//        {
//            motor1.set_pwm_duty_cycle(i);
//            motor2.set_pwm_duty_cycle(i);
//            dummy_delay(10000);
//        }
//        for ( int32_t i = -PWM_PRECISION; i <= 0; ++i )
//        {
//            motor1.set_pwm_duty_cycle(i);
//            motor2.set_pwm_duty_cycle(i);
//            dummy_delay(10000);
//        }

//
//        if ( nrf24_is_new_data_availiable( &robot_nrf ) )
//        {
//            nrf24_read_message(&robot_nrf, nrf_input_data, 6);
//            toggle_d4_led();
//        }
    }
}

/****************************************************************************************/
/*                                                                                      */
/*                              System interrupt handler                                */
/*                                                                                      */
/****************************************************************************************/

uint32_t system_counter = 0;

void SysTick_Handler()
{
    system_counter += 1;
    add_mistake_to_the_log( icm20600_get_raw_data( &robot_icm20600 ) );
    add_mistake_to_the_log( icm20600_process_raw_data( &robot_icm20600, icm_processed_data ) );
    calculate_x_angle( &robot_icm20600, icm_processed_data, &robot_measured_angle , 1.0f / (float)(SYSTICK_INTERRUPT_FREQUENCY) );
//    robot_measured_angle_2 = calculate_x_angle_2( &robot_icm20600, icm_processed_data, &gyro_angle_integral, &accel_measured_angle, accel_buffer, 1.0f / (float)(SYSTICK_INTERRUPT_FREQUENCY) );

    if ( system_counter == SYSTICK_INTERRUPT_FREQUENCY / 2 )
    {
        toggle_d2_led();
        system_counter = 0;
    }



}


//void encode_float_variable(float value_to_encode,  uint8_t array_to_store_result[])
//{
//    uint8_t *u8_ptr = &value_to_encode;
//
//    array_to_store_result[0] = *(u8_ptr + 0);
//    array_to_store_result[1] = *(u8_ptr + 1);
//    array_to_store_result[2] = *(u8_ptr + 2);
//    array_to_store_result[3] = *(u8_ptr + 3);
//
//}
//
//float decode_float_variable(uint8_t array_with_value[])
//{
//    float result;
//    uint8_t *u8_ptr = &result;
//
//    *(u8_ptr + 0) = array_with_value[0];
//    *(u8_ptr + 1) = array_with_value[1];
//    *(u8_ptr + 2) = array_with_value[2];
//    *(u8_ptr + 3) = array_with_value[3];
//
//    return result;
//}
