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
        calibration_array[icm_x_axis_index] -= icm_instance->raw_data[icm_gyro_x_index];
        calibration_array[icm_y_axis_index] -= icm_instance->raw_data[icm_gyro_y_index];
        calibration_array[icm_z_axis_index] -= icm_instance->raw_data[icm_gyro_z_index];
    }

    icm_instance->gyro_calibration_coefficients[icm_x_axis_index] = calibration_array[icm_x_axis_index] >> calibration_coef;
    icm_instance->gyro_calibration_coefficients[icm_y_axis_index] = calibration_array[icm_y_axis_index] >> calibration_coef;
    icm_instance->gyro_calibration_coefficients[icm_z_axis_index] = calibration_array[icm_z_axis_index] >> calibration_coef;
    icm20600_set_calibration_values( icm_instance );
}

float calculate_base_angle(icm20600 *icm_instance, uint32_t cycle_length)
{
    int32_t accel_values[2] = { 0, 0 };

    for ( uint32_t i = 0; i < 8; ++i )
    {
        delay_in_milliseconds( cycle_length );
        icm20600_get_raw_data( icm_instance );
        accel_values[0] += icm_instance->raw_data[icm_accel_y_index];
        accel_values[1] += icm_instance->raw_data[icm_accel_z_index];
    }
    float temp_processed_data[7];
    icm20600_process_raw_data(icm_instance, temp_processed_data);

    accel_values[0] >>= 3;
    accel_values[1] >>= 3;

    float return_value = atan2f( temp_processed_data[icm_accel_z_index], -1 * temp_processed_data[icm_accel_y_index] ) * 57.296f;
    return return_value;
}

// Вычисляет только нужный угол из трех. Может вызываться без вызова каких либо других функций, так как будет считывать значения датчика самостоятельно
void calculate_x_angle( icm20600 *icm_instance, float *processed_values, float *current_angle, float integration_period )
{
    // Calculations, unique for every plane
    float accel_based_value = atan2f( processed_values[icm_accel_z_index], -1 * processed_values[icm_accel_y_index] ) * 57.296f;
    float gyro_intergral_value = *current_angle;
    gyro_intergral_value += (icm_instance->previous_gyro_values[icm_x_axis_index] + processed_values[icm_gyro_x_index]) / 2.0f * integration_period;

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

    icm_instance->previous_gyro_values[icm_x_axis_index] = processed_values[icm_gyro_x_index]; // For trapezoidal integration

    *current_angle = accel_based_value * icm_instance->complementary_filter_coef
                + gyro_intergral_value * (1.0f - icm_instance->complementary_filter_coef);
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

// EOF
