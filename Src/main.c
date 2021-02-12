/*!
 * @file main.c
 */

#define DECLARE_GLOBAL_VARIABLES
#include "device.h"

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

uint8_t nrf_rx_address[5] = { 0x11, 0x22, 0x33, 0x44, 0x66 }; // blue-LED controller
uint8_t nrf_input_data[6] = { 0, 0, 0, 0, 0, 0 };

icm20600 robot_icm20600 =
                          {
                            .spi_write_byte = spi2_write_single_byte,
                            .set_cs_high = set_icm20600_cs_high,
                            .set_cs_low = set_icm20600_cs_low,

                            .gyro_scale_setup = icm_gyro_500dps_scale,
                            .accel_scale_setup = icm_accel_4g_scale,
                            .complementary_filter_coef = 0.1f,
                            .enable_temperature_sensor = 0,

                            .raw_data = { 0, 0, 0, 0, 0, 0, 0 },
                            .gyro_calibration_coefficients = { 0, 0, 0 },
                            .previous_gyro_x = 0,
                            .previous_gyro_y = 0,
                            .previous_gyro_z = 0,

                            .was_initialized = 0
                          };

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

    add_mistake_to_the_log( icm20600_init( &robot_icm20600 ) );


    // test for Jonas
//    encode_float_variable(-10450.0f, array_with_result);
//    result_val = decode_float_variable(array_with_result);



    while ( 1 )
    {
        if ( nrf24_is_new_data_availiable( &robot_nrf ) )
        {
            nrf24_read_message(&robot_nrf, nrf_input_data, 6);
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

void SysTick_Handler()
{
    system_counter += 1;

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
