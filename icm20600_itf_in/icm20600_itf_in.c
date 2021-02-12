/*!
 * @file icm20600_itf_in.c
 */

#include "icm20600_itf_in.h"

/****************************************************************************************/
/*                                                                                      */
/*                            Static functions declarations                             */
/*                                                                                      */
/****************************************************************************************/

static void icm20600_write_register( icm20600 *icm_instance, uint8_t address, uint8_t data );
static uint8_t icm20600_read_register( icm20600 *icm_instance, uint8_t address );

static void icm20600_start_writing_sequence( icm20600 *icm_instance, uint8_t address, uint8_t data );
static void icm20600_write_next_sequenced_byte( icm20600 *icm_instance, uint8_t data );
static void icm20600_finish_writing_sequence( icm20600 *icm_instance, uint8_t data );

static uint8_t icm20600_start_reading_sequence(icm20600 *icm_instance, uint8_t address);
static uint8_t icm20600_read_next_sequenced_byte( icm20600 *icm_instance);
static uint8_t icm20600_finish_reading_sequence( icm20600 *icm_instance);

static uint16_t icm20600_check_declarations( icm20600 *icm_instance );

static uint16_t icm20600_check_parameters( icm20600 *icm_instance );

/****************************************************************************************/
/*                                                                                      */
/*                          Static functions implementations                            */
/*                                                                                      */
/****************************************************************************************/

static inline void icm20600_write_register( icm20600 *icm_instance, uint8_t address, uint8_t data )
{
    icm_instance->set_cs_low();
    icm_instance->spi_write_byte( (address & ICM20600_ADDRESS_MASK) | ICM20600_WRITE_REGISTER );
    icm_instance->spi_write_byte( data );
    icm_instance->set_cs_high();
}

static uint8_t icm20600_read_register( icm20600 *icm_instance, uint8_t address )
{
    icm_instance->set_cs_low();
    icm_instance->spi_write_byte( (address & ICM20600_ADDRESS_MASK) | ICM20600_READ_REGISTER);
    uint8_t return_value = icm_instance->spi_write_byte( 0xFF );
    icm_instance->set_cs_high();

    return return_value;
}

static inline void icm20600_start_writing_sequence( icm20600 *icm_instance, uint8_t address, uint8_t data )
{
    icm_instance->set_cs_low();
    icm_instance->spi_write_byte( (address & ICM20600_ADDRESS_MASK) | ICM20600_WRITE_REGISTER );
    icm_instance->spi_write_byte( data );
}

static inline void icm20600_write_next_sequenced_byte( icm20600 *icm_instance, uint8_t data )
{
    icm_instance->spi_write_byte( data );
}

static inline void icm20600_finish_writing_sequence( icm20600 *icm_instance, uint8_t data )
{
    icm_instance->spi_write_byte( data );
    icm_instance->set_cs_high();
}

static uint8_t icm20600_start_reading_sequence(icm20600 *icm_instance, uint8_t address)
{
    icm_instance->set_cs_low();
    icm_instance->spi_write_byte( (address & ICM20600_ADDRESS_MASK) | ICM20600_READ_REGISTER);
    return icm_instance->spi_write_byte( 0xFF );
}

static uint8_t icm20600_read_next_sequenced_byte( icm20600 *icm_instance)
{
    return icm_instance->spi_write_byte( 0xFF );
}

static uint8_t icm20600_finish_reading_sequence( icm20600 *icm_instance)
{
    uint8_t return_value = icm_instance->spi_write_byte( 0xFF );
    icm_instance->set_cs_high();
    return return_value;
}

static uint16_t icm20600_check_declarations( icm20600 *icm_instance )
{
    if ( icm_instance->spi_write_byte == 0 )
    {
        return ICM20600_SPI_WRITE_FUNCTION_ABSENT_Err;
    }

    if ( icm_instance->set_cs_high == 0 )
    {
        return ICM20600_SPI_CS_TOGGLE_FUNCTIONS_ABSENT_Err;
    }

    if ( icm_instance->set_cs_low == 0 )
    {
        return ICM20600_SPI_CS_TOGGLE_FUNCTIONS_ABSENT_Err;
    }

    return 0;
}

static uint16_t icm20600_check_parameters( icm20600 *icm_instance )
{
    if ( icm_instance->gyro_scale_setup > icm_gyro_2000dps_scale )
    {
        icm_instance->gyro_scale_setup = icm_gyro_2000dps_scale;
        return ICM20600_WRONG_GYROSCOPE_SCALE_Err;
    }
    if ( icm_instance->gyro_scale_setup < icm_gyro_250dps_scale )
    {
        icm_instance->gyro_scale_setup = icm_gyro_250dps_scale;
        return ICM20600_WRONG_GYROSCOPE_SCALE_Err;
    }

    if ( icm_instance->accel_scale_setup > icm_accel_16g_scale )
    {
        icm_instance->accel_scale_setup = icm_accel_16g_scale;
        return ICM20600_WRONG_ACCELEROMETER_SCALE_Err;
    }
    if ( icm_instance->accel_scale_setup < icm_accel_2g_scale )
    {
        icm_instance->accel_scale_setup = icm_accel_2g_scale;
        return ICM20600_WRONG_ACCELEROMETER_SCALE_Err;
    }

    return 0;
}

/****************************************************************************************/
/*                                                                                      */
/*                          Global functions implementations                            */
/*                                                                                      */
/****************************************************************************************/

uint16_t icm20600_init(icm20600 *icm_instance)
{
    uint32_t mistake_code = icm20600_check_declarations(icm_instance);
    if ( mistake_code != 0 )
    {
        return mistake_code;
    }

    icm_instance->set_cs_high(); // SPI must start with CS fall, so to get fall before first data transfer CS must be set high.

    mistake_code = icm20600_check_if_alive( icm_instance );
    if ( mistake_code != 0 )
    {
        return mistake_code;
    }

    // All possible mistakes are tolerable, so there is no need to return immediately.
    mistake_code = icm20600_check_parameters( icm_instance );

    // Disable I2C.
    icm20600_write_register( icm_instance, ICM20600_I2C_IF, ICM20600_I2C_IF_I2C_IF_DIS );

    // Setup gyro and accel scales
    icm20600_start_writing_sequence( icm_instance, ICM20600_GYRO_CONFIG, icm_instance->gyro_scale_setup << ICM20600_GYRO_CONFIG_FS_SEL_Pos );
    icm20600_finish_writing_sequence( icm_instance, icm_instance->accel_scale_setup << ICM20600_ACCEL_CONFIG_ACCEL_FS_SEL_Pos );

    if ( icm_instance->enable_temperature_sensor )
    {
        icm20600_write_register(icm_instance, ICM20600_PWR_MGMT_1, 1 << ICM20600_PWR_MGMT_1_CLKSEL_Pos);
    }
    else
    {
        icm20600_write_register(icm_instance, ICM20600_PWR_MGMT_1, ICM20600_PWR_MGMT_1_TEMP_DIS | 1 << ICM20600_PWR_MGMT_1_CLKSEL_Pos);
    }

    icm_instance->was_initialized = 1;

    return mistake_code;
}

uint16_t icm20600_get_raw_data(icm20600 *icm_instance, int16_t *data_storage_array)
{
    if (icm_instance->was_initialized == 0)
    {
        return ICM20600_WAS_NOT_INITIALIZED_Err;
    }


    icm_instance->set_cs_low();

    // Start sequential reading of sensor data
    icm_instance->spi_write_byte(ICM20600_ACCEL_XOUT_H | ICM20600_READ_REGISTER);

    // Get accelerometer data
    data_storage_array[icm_accel_x_index] = icm_instance->spi_write_byte(0xFF)<<8 | icm_instance->spi_write_byte(0xFF);
    data_storage_array[icm_accel_y_index] = icm_instance->spi_write_byte(0xFF)<<8 | icm_instance->spi_write_byte(0xFF);
    data_storage_array[icm_accel_z_index] = icm_instance->spi_write_byte(0xFF)<<8 | icm_instance->spi_write_byte(0xFF);

    if (icm_instance->enable_temperature_sensor)    // If array is at least 7 int16_t fields long
    {
        // Add temperature sensor data to array
        data_storage_array[icm_temperature_index] = icm_instance->spi_write_byte(0xFF)<<8 | icm_instance->spi_write_byte(0xFF);
    }
    else
    {
        // Skip temperature sensor data
        icm_instance->spi_write_byte(0xFF);
        icm_instance->spi_write_byte(0xFF);
    }

    // Get gyroscope data
    data_storage_array[icm_gyro_x_index] = (icm_instance->spi_write_byte(0xFF)<<8 | icm_instance->spi_write_byte(0xFF)) - icm_instance->gyro_calibration_coefficients[icm_x_axis_index];
    data_storage_array[icm_gyro_y_index] = (icm_instance->spi_write_byte(0xFF)<<8 | icm_instance->spi_write_byte(0xFF)) - icm_instance->gyro_calibration_coefficients[icm_y_axis_index];
    data_storage_array[icm_gyro_z_index] = (icm_instance->spi_write_byte(0xFF)<<8 | icm_instance->spi_write_byte(0xFF)) - icm_instance->gyro_calibration_coefficients[icm_z_axis_index];

    icm_instance->set_cs_high();

    return 0;
}

uint16_t icm20600_procces_raw_data(icm20600 *icm_instance, int16_t *raw_input_array, float * processed_output_array)
{
    if (icm_instance->was_initialized == 0)
    {
        return ICM20600_WAS_NOT_INITIALIZED_Err;
    }

    float gyro_divider = 1.0f;

    switch ( icm_instance->gyro_scale_setup )
    {
        case icm_gyro_250dps_scale:
            gyro_divider = 1.0f;
            break;
        case icm_gyro_500dps_scale:
            gyro_divider = 2.0f;
            break;
        case icm_gyro_1000dps_scale:
            gyro_divider = 4.0f;
            break;
        case icm_gyro_2000dps_scale:
            gyro_divider = 8.0f;
            break;
        default:
            break;
    }

        float gyro_sensitivity = icm20600_GYRO_BASIC_SENSITIVITY/gyro_divider;
        float accel_sensitivity = (uint32_t)(icm20600_ACC_BASIC_SENSITIVITY >> (icm_instance->accel_scale_setup));    // Will be the same value for the 2g setup and twice as big for every next one

        processed_output_array[icm_accel_x_index] = (float)(raw_input_array[icm_accel_x_index]) / accel_sensitivity;
        processed_output_array[icm_accel_y_index] = (float)(raw_input_array[icm_accel_y_index]) / accel_sensitivity;
        processed_output_array[icm_accel_z_index] = (float)(raw_input_array[icm_accel_z_index]) / accel_sensitivity;

        processed_output_array[icm_gyro_x_index] = (float)(raw_input_array[icm_gyro_x_index]) / gyro_sensitivity;
        processed_output_array[icm_gyro_y_index] = (float)(raw_input_array[icm_gyro_y_index]) / gyro_sensitivity;
        processed_output_array[icm_gyro_z_index] = (float)(raw_input_array[icm_gyro_z_index]) / gyro_sensitivity;

        if ( icm_instance->enable_temperature_sensor )
        {
            processed_output_array[icm_temperature_index] = (float)(raw_input_array[icm_temperature_index]) / icm20600_TEMPERATURE_SENSITIVITY + icm20600_TEMPERATURE_OFFSET;
        }

        return 0;
}


uint16_t icm20600_get_proccesed_data(icm20600 *icm_instance, float *processed_output_array)
{
    if (icm_instance->was_initialized == 0)
    {
        return ICM20600_WAS_NOT_INITIALIZED_Err;
    }

    int16_t current_sencor_measurements[7];
    icm20600_get_raw_data(icm_instance, current_sencor_measurements);

    icm20600_procces_raw_data(icm_instance, current_sencor_measurements, processed_output_array);

    return 0;
}

/*!
 * Default value should be 0x11. Function checks for the 0 or 0xff, both of them will mean that there is a connection problem.
 */
uint16_t icm20600_check_if_alive(icm20600 *icm_instance)
{
    uint8_t icm_who_am_i_value = icm20600_read_register(icm_instance, ICM20600_WHO_AM_I);

    if(icm_who_am_i_value == 0 || icm_who_am_i_value == 0xFF)
    {
        return ICM20600_IS_NOT_CONNECTED_Err;
    }
    else
    {
        return 0;
    }
}


// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! //
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! //
// Code danger zone
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! //
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! //

// Вычисляет все углы в радианах

// Скорее всего эта функция на самом деле не работает и ее надо будет полностью переписать, когда я разберусь в том,
//  как именно она должна работать. Но пока я не разобрался оставлю как есть
uint32_t icm20600_calculate_all_angles(icm20600 *icm_instance, float angles_storage[3], float integration_period)
{
    if (icm_instance->was_initialized == 0)
    {
        return ICM20600_WAS_NOT_INITIALIZED_Err;
    }

    float processed_values[7];
    icm20600_get_proccesed_data(icm_instance, processed_values);

    // z x angle calculations
    float accelerometer_based_angle = atan2(processed_values[icm_accel_x_index], processed_values[icm_accel_z_index]) * 57.296f;
    float gyroscope_based_angle = (icm_instance->previous_gyro_x + processed_values[icm_gyro_y_index])/2.0f * integration_period;
    icm_instance->previous_gyro_x = processed_values[icm_gyro_y_index];

    // Calculations, unique for every plane
    angles_storage[z_x_angle_index] = accelerometer_based_angle * icm_instance->complementary_filter_coef + gyroscope_based_angle * (1.0f - icm_instance->complementary_filter_coef);

    return 0;
}

// Вычисляет только нужный угол из трех. Может вызываться без вызова каких либо других функций, так как будет считывать значения датчика самостоятельно
uint32_t icm20600_calculate_z_x_angle(icm20600 *icm_instance, float *calculated_angle, float integration_period)
{
    if (icm_instance->was_initialized == 0)
    {
        return ICM20600_WAS_NOT_INITIALIZED_Err;
    }

    float processed_values[7];
    icm20600_get_proccesed_data(icm_instance, processed_values);

    // Calculations, unique for every plane
    float accelerometer_based_angle = atan2(processed_values[icm_accel_x_index], processed_values[icm_accel_z_index]) * 57.296f;
    float gyroscope_based_angle = (icm_instance->previous_gyro_y + processed_values[icm_gyro_y_index])/2.0f * integration_period;
    icm_instance->previous_gyro_y = processed_values[icm_gyro_y_index];

    *calculated_angle = accelerometer_based_angle * icm_instance->complementary_filter_coef + gyroscope_based_angle * (1.0f - icm_instance->complementary_filter_coef);

    return 0;
}


uint32_t icm20600_calculate_y_z_angle(icm20600 *icm_instance, float *calculated_angle, float integration_period)
{
    if (icm_instance->was_initialized == 0)
    {
        return ICM20600_WAS_NOT_INITIALIZED_Err;
    }

    float processed_values[7];
    icm20600_get_proccesed_data(icm_instance, processed_values);

    // Calculations, unique for every plane
    float accelerometer_based_angle = atan2(processed_values[icm_accel_z_index], processed_values[icm_accel_y_index]) * 57.296f;    // Value in degrees
    float gyroscope_based_angle = (icm_instance->previous_gyro_x + processed_values[icm_gyro_x_index])/2.0f * integration_period;
    icm_instance->previous_gyro_x = processed_values[icm_gyro_x_index];

    *calculated_angle = accelerometer_based_angle * icm_instance->complementary_filter_coef + gyroscope_based_angle * (1.0f - icm_instance->complementary_filter_coef);

    return 0;
}

uint32_t icm20600_calculate_x_y_angle(icm20600 *icm_instance, float *calculated_angle, float integration_period)
{
    if (icm_instance->was_initialized == 0)
    {
        return ICM20600_WAS_NOT_INITIALIZED_Err;
    }

    float processed_values[7];
    icm20600_get_proccesed_data(icm_instance, processed_values);

    // пока просто верну 0. Так как для балансирующего робота этот угол не нужен


    return 0;
}


