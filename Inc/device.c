/*!
 * @file devive.c
 *
 * Contains implementations of functions from device.h
 */

#include "device.h"

/*!
 * @brief Sets up all project related peripherals.
 */
void setup_device(void)
{
    // Enable FPU.
    SCB->CPACR |= 0x3 << 20 ;

    /*! Every single time related parameter is based on SYSTEM_MAIN_FREQUENCY, thus clock be set up a soon as possible. */
    setup_system_clock();

    setup_timers();

    setup_leds();

    setup_motor1();
    setup_motor2();

    setup_nrf24l01p_peripherals();
    setup_icm20600_peripherals();

    delay_in_milliseconds(100);

    // setup_system_timer();

    // @todo Add some kind of control loop zeroing.
    // @todo write the code to init slave devices.
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
        toggle_d4_led();
        calibration_array[icm_x_axis_index] -= icm_instance->raw_data[icm_gyro_x_index];
        calibration_array[icm_y_axis_index] -= icm_instance->raw_data[icm_gyro_y_index];
        calibration_array[icm_z_axis_index] -= icm_instance->raw_data[icm_gyro_z_index];
    }

    icm_instance->gyro_calibration_coefficients[icm_x_axis_index] = calibration_array[icm_x_axis_index] >> calibration_coef;
    icm_instance->gyro_calibration_coefficients[icm_y_axis_index] = calibration_array[icm_y_axis_index] >> calibration_coef;
    icm_instance->gyro_calibration_coefficients[icm_z_axis_index] = calibration_array[icm_z_axis_index] >> calibration_coef;
    icm20600_set_calibration_values( icm_instance );
}





// EOF
