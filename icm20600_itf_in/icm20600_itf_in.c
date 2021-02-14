/*!
 * @file icm20600_itf_in.c
 */

#include "icm20600_itf_in.h"

/****************************************************************************************/
/*                                                                                      */
/*                            Static functions declarations                             */
/*                                                                                      */
/****************************************************************************************/

static void icm_write_register( icm20600 *icm_instance, uint8_t address, uint8_t data );
static uint8_t icm_read_register( icm20600 *icm_instance, uint8_t address );

static void icm_start_writing_sequence( icm20600 *icm_instance, uint8_t address, uint8_t data );
static void icm_write_next_sequenced_byte( icm20600 *icm_instance, uint8_t data );
static void icm_finish_writing_sequence( icm20600 *icm_instance, uint8_t data );

static uint8_t icm_start_reading_sequence( icm20600 *icm_instance, uint8_t address );
static uint8_t icm_read_next_sequenced_byte( icm20600 *icm_instance );
static uint8_t icm_finish_reading_sequence( icm20600 *icm_instance );

static uint16_t icm_check_declarations( icm20600 *icm_instance );

static uint16_t icm20600_check_parameters( icm20600 *icm_instance );

static void icm_reset_all_registers( icm20600 *icm_instance );

static void icm_reset_variable_parameters( icm20600 *icm_instance );

static uint16_t icm_check_gyro_parameters( icm20600 *icm_instance );
static uint16_t icm_check_accel_parameters( icm20600 *icm_instance );
static uint16_t icm_setup_gyro_and_accel( icm20600 *icm_instance );

static void icm_check_raw_data_for_faults( icm20600 *icm_instance );

static void icm_wakeup( icm20600 *icm_instance );

/****************************************************************************************/
/*                                                                                      */
/*                          Static functions implementations                            */
/*                                                                                      */
/****************************************************************************************/

static inline void icm_write_register( icm20600 *icm_instance, uint8_t address, uint8_t data )
{
    icm_instance->set_cs_low();
    icm_instance->spi_write_byte( (address & ICM20600_ADDRESS_MASK) | ICM20600_WRITE_REGISTER );
    icm_instance->spi_write_byte( data );
    icm_instance->set_cs_high();
}

static uint8_t icm_read_register( icm20600 *icm_instance, uint8_t address )
{
    icm_instance->set_cs_low();
    icm_instance->spi_write_byte( (address & ICM20600_ADDRESS_MASK) | ICM20600_READ_REGISTER);
    uint8_t return_value = icm_instance->spi_write_byte( 0xFF );
    icm_instance->set_cs_high();

    return return_value;
}

static inline void icm_start_writing_sequence( icm20600 *icm_instance, uint8_t address, uint8_t data )
{
    icm_instance->set_cs_low();
    icm_instance->spi_write_byte( (address & ICM20600_ADDRESS_MASK) | ICM20600_WRITE_REGISTER );
    icm_instance->spi_write_byte( data );
}

static inline void icm_write_next_sequenced_byte( icm20600 *icm_instance, uint8_t data )
{
    icm_instance->spi_write_byte( data );
}

static inline void icm_finish_writing_sequence( icm20600 *icm_instance, uint8_t data )
{
    icm_instance->spi_write_byte( data );
    icm_instance->set_cs_high();
}

static uint8_t icm_start_reading_sequence( icm20600 *icm_instance, uint8_t address )
{
    icm_instance->set_cs_low();
    icm_instance->spi_write_byte( (address & ICM20600_ADDRESS_MASK) | ICM20600_READ_REGISTER );
    return icm_instance->spi_write_byte( 0xFF );
}

//static uint8_t icm_read_next_sequenced_byte( icm20600 *icm_instance)
static inline uint8_t icm_read_next_sequenced_byte( icm20600 *icm_instance )
{
    return icm_instance->spi_write_byte( 0xFF );
}

static uint8_t icm_finish_reading_sequence( icm20600 *icm_instance )
{
    uint8_t return_value = icm_instance->spi_write_byte( 0xFF );
    icm_instance->set_cs_high();
    return return_value;
}

static uint16_t icm_check_declarations( icm20600 *icm_instance )
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
        return ICM20600_WRONG_GYRO_SCALE_Err;
    }
    if ( icm_instance->gyro_scale_setup < icm_gyro_250dps_scale )
    {
        icm_instance->gyro_scale_setup = icm_gyro_250dps_scale;
        return ICM20600_WRONG_GYRO_SCALE_Err;
    }

    if ( icm_instance->accel_scale_setup > icm_accel_16g_scale )
    {
        icm_instance->accel_scale_setup = icm_accel_16g_scale;
        return ICM20600_WRONG_ACCEL_SCALE_Err;
    }
    if ( icm_instance->accel_scale_setup < icm_accel_2g_scale )
    {
        icm_instance->accel_scale_setup = icm_accel_2g_scale;
        return ICM20600_WRONG_ACCEL_SCALE_Err;
    }

    return 0;
}

//! It is super important to make a delay for at least 10ms after this function, so that icm20600 will be able to reset all registers.
static void icm_reset_all_registers( icm20600 *icm_instance )
{
    uint8_t answer = icm_read_register( icm_instance, ICM20600_PWR_MGMT_1 );

    answer |= ICM20600_PWR_MGMT_1_DEVICE_RESET;

    icm_write_register( icm_instance, ICM20600_PWR_MGMT_1, answer );

    answer = icm_read_register( icm_instance, ICM20600_PWR_MGMT_1 );

    while ( answer & ICM20600_PWR_MGMT_1_DEVICE_RESET )
    {
        answer = icm_read_register( icm_instance, ICM20600_PWR_MGMT_1 );
    }
}

static void icm_reset_variable_parameters( icm20600 *icm_instance )
{
    icm_instance->previous_gyro_values[icm_x_axis_index] = 0;
    icm_instance->previous_gyro_values[icm_y_axis_index] = 0;
    icm_instance->previous_gyro_values[icm_z_axis_index] = 0;

    icm_instance->raw_data[icm_gyro_x_index] = 0;
    icm_instance->raw_data[icm_gyro_y_index] = 0;
    icm_instance->raw_data[icm_gyro_z_index] = 0;
    icm_instance->raw_data[icm_accel_x_index] = 0;
    icm_instance->raw_data[icm_accel_y_index] = 0;
    icm_instance->raw_data[icm_accel_z_index] = 0;
    icm_instance->raw_data[icm_temperature_index] = 0;
}

static uint16_t icm_check_gyro_parameters( icm20600 *icm_instance )
{
    uint16_t error_code = 0;

    if (icm_instance->gyro_averaging_setup > icm_gyro_128_samples_averaging )
    {
        icm_instance->gyro_averaging_setup = icm_gyro_128_samples_averaging;
        error_code = ICM20600_WRONG_GYRO_AVERAGING_Err;
    }
    if(icm_instance->gyro_averaging_setup < icm_gyro_1_sample_averaging )
    {
        icm_instance->gyro_averaging_setup = icm_gyro_1_sample_averaging;
        error_code = ICM20600_WRONG_GYRO_AVERAGING_Err;
    }

    if(icm_instance->gyro_filter_bw > icm_gyro_5_hz)
    {
        icm_instance->gyro_filter_bw = icm_gyro_5_hz;
        error_code = ICM20600_WRONG_GYRO_FILTER_BW_Err;
    }
    if(icm_instance->gyro_filter_bw < icm_gyro_250_hz)
    {
        icm_instance->gyro_filter_bw = icm_gyro_250_hz;
        error_code = ICM20600_WRONG_GYRO_FILTER_BW_Err;
    }

    if(icm_instance->gyro_scale_setup > icm_gyro_2000dps_scale)
    {
        icm_instance->gyro_scale_setup = icm_gyro_2000dps_scale;
        error_code = ICM20600_WRONG_GYRO_SCALE_Err;
    }
    if(icm_instance->gyro_scale_setup < icm_gyro_250dps_scale)
    {
        icm_instance->gyro_scale_setup = icm_gyro_250dps_scale;
        error_code = ICM20600_WRONG_GYRO_SCALE_Err;
    }

    return error_code;
}

static uint16_t icm_check_accel_parameters( icm20600 *icm_instance )
{
    uint16_t error_code = 0;

    if(icm_instance->accel_averaging_setup > icm_accel_32_samples_averaging)
    {
        icm_instance->accel_averaging_setup = icm_accel_32_samples_averaging;
        error_code = ICM20600_WRONG_ACCEL_AVERAGING_Err;
    }
    if(icm_instance->accel_averaging_setup < icm_accel_4_samples_averaging)
    {
        icm_instance->accel_averaging_setup = icm_accel_4_samples_averaging;
        error_code = ICM20600_WRONG_ACCEL_AVERAGING_Err;
    }

    if(icm_instance->accel_filter_bw > icm_accel_5_hz)
    {
        icm_instance->accel_filter_bw = icm_accel_5_hz;
        error_code = ICM20600_WRONG_ACCEL_FILTER_BW_Err;
    }
    if(icm_instance->accel_filter_bw < icm_accel_218_hz)
    {
        icm_instance->accel_filter_bw = icm_accel_218_hz;
        error_code = ICM20600_WRONG_ACCEL_FILTER_BW_Err;
    }

    if(icm_instance->accel_scale_setup > icm_accel_16g_scale)
    {
        icm_instance->accel_scale_setup = icm_accel_16g_scale;
        error_code = ICM20600_WRONG_ACCEL_SCALE_Err;
    }
    if(icm_instance->accel_scale_setup < icm_accel_2g_scale)
    {
        icm_instance->accel_scale_setup = icm_accel_2g_scale;
        error_code = ICM20600_WRONG_ACCEL_SCALE_Err;
    }

    return error_code;
}

static uint16_t icm_setup_gyro_and_accel( icm20600 *icm_instance )
{
    uint16_t error_code = icm_check_gyro_parameters( icm_instance );

    if ( error_code == 0 )
    {
        error_code = icm_check_accel_parameters( icm_instance );
    }

    icm_start_writing_sequence(icm_instance, ICM20600_SMPLRT_DIV, icm_instance->sample_rate_divider); // SMPLRT_DIV setup;
    icm_write_next_sequenced_byte( icm_instance, icm_instance->gyro_filter_bw << ICM20600_CONFIG_DLPF_CFG_Pos ); // CONFIG setup.
    icm_write_next_sequenced_byte( icm_instance, icm_instance->gyro_scale_setup << ICM20600_GYRO_CONFIG_FS_SEL_Pos ); // GYRO_CONFIG setup.
    icm_write_next_sequenced_byte( icm_instance, icm_instance->accel_scale_setup << ICM20600_ACCEL_CONFIG_ACCEL_FS_SEL_Pos ); // ACCEL_CONFIG setup.

    uint8_t icm20600_accel_congig2_setup = icm_instance->accel_averaging_setup << ICM20600_ACCEL_CONFIG2_DEC2_CFG_Pos
                                           | icm_instance->accel_filter_bw << ICM20600_ACCEL_CONFIG2_A_DLPF_CFG_Pos;
    icm_write_next_sequenced_byte( icm_instance, icm20600_accel_congig2_setup ); // ACCEL_CONFIG2 setup.

    icm_finish_writing_sequence(icm_instance, icm_instance->gyro_averaging_setup << ICM20600_LP_MODE_CFG_G_AVGCFG_Pos); // LP_MODE_CFG setup.

    return error_code;
}

static void icm_check_raw_data_for_faults( icm20600 *icm_instance )
{
    if(icm_instance->raw_data[icm_gyro_x_index] == 32767 || icm_instance->raw_data[icm_gyro_x_index] == -32768)
    {
        icm_instance->gyro_faults[icm_x_axis_index] = 1;
    }

    if(icm_instance->raw_data[icm_gyro_y_index] == 32767 || icm_instance->raw_data[icm_gyro_y_index] == -32768)
    {
        icm_instance->gyro_faults[icm_y_axis_index] = 1;
    }

    if(icm_instance->raw_data[icm_gyro_z_index] == 32767 || icm_instance->raw_data[icm_gyro_z_index] == -32768)
    {
        icm_instance->gyro_faults[icm_z_axis_index] = 1;
    }
}

//! Resets sleep bit to zero and disables temperature sensor if needed.
static void icm_wakeup(icm20600 *icm_instance)
{
    if ( icm_instance->enable_temperature_sensor )
    {
        icm_write_register(icm_instance, ICM20600_PWR_MGMT_1, 1 << ICM20600_PWR_MGMT_1_CLKSEL_Pos);
    }
    else
    {
        icm_write_register(icm_instance, ICM20600_PWR_MGMT_1, ICM20600_PWR_MGMT_1_TEMP_DIS | 1 << ICM20600_PWR_MGMT_1_CLKSEL_Pos);
    }
}

/****************************************************************************************/
/*                                                                                      */
/*                          Global functions implementations                            */
/*                                                                                      */
/****************************************************************************************/
/*                                                                                      */
/*                                   Initialization                                     */
/*                                                                                      */
/****************************************************************************************/

uint16_t icm20600_init(icm20600 *icm_instance)
{
    uint32_t error_code = icm_check_declarations(icm_instance);
    if ( error_code != 0 )
    {
        return error_code;
    }

    icm_instance->set_cs_high(); // SPI must start with CS fall, so to get fall before first data transfer CS must be set high.

    error_code = icm20600_check_if_alive( icm_instance );
    if ( error_code != 0 )
    {
        return error_code;
    }

    error_code = icm20600_check_parameters( icm_instance ); // All possible mistakes are tolerable, so there is no need to return immediately.

    icm_write_register( icm_instance, ICM20600_I2C_IF, ICM20600_I2C_IF_I2C_IF_DIS ); // Disable I2C
    error_code = icm_setup_gyro_and_accel(icm_instance);

    icm_write_register(icm_instance, ICM20600_ACCEL_INTEL_CTRL, ICM20600_ACCEL_INTEL_CTRL_OUTPUT_LIMIT);

    icm_wakeup(icm_instance);

    icm_reset_variable_parameters(icm_instance); // In case user understood something wrong.
    icm_instance->was_initialized = 1;



    icm20600_set_calibration_values(icm_instance);

    return error_code;
}

uint16_t icm20600_reset_all_registeres( icm20600 *icm_instance )
{
    uint32_t error_code = icm_check_declarations(icm_instance);
    if ( error_code != 0 )
    {
        return error_code;
    }

    icm_instance->set_cs_high(); // SPI must start with CS fall, so to get fall before first data transfer CS must be set high.

    error_code = icm20600_check_if_alive( icm_instance );
    if ( error_code != 0 )
    {
        return error_code;
    }

    icm_reset_all_registers(icm_instance);

    return 0;
}

uint16_t icm20600_disable_gyro( icm20600 *icm_instance )
{
    if ( icm_instance->was_initialized == 0 )
    {
        return ICM20600_WAS_NOT_INITIALIZED_Err;
    }

    uint8_t answer = icm_read_register( icm_instance, ICM20600_PWR_MGMT_2 );

    answer |= ICM20600_PWR_MGMT_2_STBY_XG | ICM20600_PWR_MGMT_2_STBY_YG | ICM20600_PWR_MGMT_2_STBY_ZG;

    icm_write_register( icm_instance, ICM20600_PWR_MGMT_2, answer );

    return 0;
}

uint16_t icm20600_disable_accel( icm20600 *icm_instance )
{
    if ( icm_instance->was_initialized == 0 )
    {
        return ICM20600_WAS_NOT_INITIALIZED_Err;
    }

    uint8_t answer = icm_read_register( icm_instance, ICM20600_PWR_MGMT_2 );

    answer |= ICM20600_PWR_MGMT_2_STBY_XA | ICM20600_PWR_MGMT_2_STBY_YA | ICM20600_PWR_MGMT_2_STBY_ZA;

    icm_write_register( icm_instance, ICM20600_PWR_MGMT_2, answer );

    return 0;
}

uint16_t icm20600_disable_one_gyro_channel( icm20600 *icm_instance, icm_axes_indexes channel_index )
{
    if ( icm_instance->was_initialized == 0 )
    {
        return ICM20600_WAS_NOT_INITIALIZED_Err;
    }

    if ( channel_index < icm_x_axis_index || channel_index > icm_z_axis_index )
    {
        return ICM20600_WRONG_GYRO_INDEX_Err;
    }

    uint8_t answer = icm_read_register( icm_instance, ICM20600_PWR_MGMT_2 );

    answer |= 1 << (2 - (uint8_t)(channel_index));
    icm_write_register( icm_instance, ICM20600_PWR_MGMT_2, answer );

    return 0;
}

uint16_t icm20600_disable_one_accel_channel( icm20600 *icm_instance, icm_axes_indexes channel_index)
{
    if ( icm_instance->was_initialized == 0 )
    {
        return ICM20600_WAS_NOT_INITIALIZED_Err;
    }

    if ( channel_index < icm_x_axis_index || channel_index > icm_z_axis_index )
    {
        return ICM20600_WRONG_ACCEL_INDEX_Err;
    }

    uint8_t answer = icm_read_register( icm_instance, ICM20600_PWR_MGMT_2 );

    answer |= 1 << (5 - (uint8_t)(channel_index));
    icm_write_register( icm_instance, ICM20600_PWR_MGMT_2, answer );

    return 0;
}

uint16_t icm20600_set_calibration_values( icm20600 *icm_instance )
{
    if (icm_instance->was_initialized == 0 )
    {
        return ICM20600_WAS_NOT_INITIALIZED_Err;
    }

    // Set gyro x axis calibration coefficient.
    icm_start_writing_sequence( icm_instance, ICM20600_XG_OFFS_USRH, (icm_instance->gyro_calibration_coefficients[icm_x_axis_index] >> 8) & 0xFF );
    icm_write_next_sequenced_byte( icm_instance, icm_instance->gyro_calibration_coefficients[icm_x_axis_index] & 0xFF );

    // Set gyto y axis calibratiiin coefficient.
    icm_write_next_sequenced_byte( icm_instance, (icm_instance->gyro_calibration_coefficients[icm_y_axis_index] >> 8) & 0xFF );
    icm_write_next_sequenced_byte( icm_instance, icm_instance->gyro_calibration_coefficients[icm_y_axis_index] & 0xFF );

    // Set gyto z axis calibratiiin coefficient.
    icm_write_next_sequenced_byte( icm_instance, (icm_instance->gyro_calibration_coefficients[icm_z_axis_index] >> 8) & 0xFF );
    icm_finish_writing_sequence( icm_instance, icm_instance->gyro_calibration_coefficients[icm_z_axis_index] & 0xFF );

    return 0;
}

/****************************************************************************************/
/*                                                                                      */
/*                              Testing and calibrations                                */
/*                                                                                      */
/****************************************************************************************/

/*!
 * Default value should be 0x11. Function checks for the 0 or 0xff, both of them will mean that there is a connection problem.
 */
uint16_t icm20600_check_if_alive( icm20600 *icm_instance )
{
    uint8_t icm_who_am_i_value = icm_read_register( icm_instance, ICM20600_WHO_AM_I );

    if ( icm_who_am_i_value == 0 || icm_who_am_i_value == 0xFF )
    {
        return ICM20600_IS_NOT_CONNECTED_Err;
    }
    else
    {
        return 0;
    }
}

/****************************************************************************************/
/*                                                                                      */
/*                                  Raw data handling                                   */
/*                                                                                      */
/****************************************************************************************/

uint16_t icm20600_get_raw_data(icm20600 *icm_instance)
{
    if (icm_instance->was_initialized == 0)
    {
        return ICM20600_WAS_NOT_INITIALIZED_Err;
    }

    // Get accelerometer data.
    icm_instance->raw_data[icm_accel_x_index] = icm_start_reading_sequence( icm_instance, ICM20600_ACCEL_XOUT_H ) << 8;
    icm_instance->raw_data[icm_accel_x_index] |= icm_read_next_sequenced_byte( icm_instance );
    icm_instance->raw_data[icm_accel_y_index] = icm_read_next_sequenced_byte( icm_instance ) << 8 | icm_read_next_sequenced_byte( icm_instance );
    icm_instance->raw_data[icm_accel_z_index] = icm_read_next_sequenced_byte( icm_instance ) << 8 | icm_read_next_sequenced_byte( icm_instance );

    // Get temperature data.
    icm_instance->raw_data[icm_temperature_index] = icm_read_next_sequenced_byte( icm_instance ) << 8 | icm_read_next_sequenced_byte( icm_instance );

    // Get gyroscope data.
    icm_instance->raw_data[icm_gyro_x_index] = icm_read_next_sequenced_byte( icm_instance ) << 8 | icm_read_next_sequenced_byte( icm_instance );
    icm_instance->raw_data[icm_gyro_y_index] = icm_read_next_sequenced_byte( icm_instance ) << 8 | icm_read_next_sequenced_byte( icm_instance );
    icm_instance->raw_data[icm_gyro_z_index] = icm_read_next_sequenced_byte( icm_instance ) << 8 | icm_finish_reading_sequence(icm_instance);

    icm_check_raw_data_for_faults(icm_instance);

    return 0;
}

uint16_t icm20600_process_raw_data( icm20600 *icm_instance, float *processed_output_array )
{
    if ( icm_instance->was_initialized == 0 )
    {
        return ICM20600_WAS_NOT_INITIALIZED_Err;
    }

    float gyro_sensitivity = icm20600_GYRO_BASIC_SENSITIVITY / (float)(1 << icm_instance->gyro_scale_setup);
    float accel_sensitivity = (uint32_t)(icm20600_ACC_BASIC_SENSITIVITY >> (icm_instance->accel_scale_setup));

    processed_output_array[icm_accel_x_index] = (float)(icm_instance->raw_data[icm_accel_x_index]) / accel_sensitivity;
    processed_output_array[icm_accel_y_index] = (float)(icm_instance->raw_data[icm_accel_y_index]) / accel_sensitivity;
    processed_output_array[icm_accel_z_index] = (float)(icm_instance->raw_data[icm_accel_z_index]) / accel_sensitivity;

    processed_output_array[icm_gyro_x_index] = (float)(icm_instance->raw_data[icm_gyro_x_index]) / gyro_sensitivity;
    processed_output_array[icm_gyro_y_index] = (float)(icm_instance->raw_data[icm_gyro_y_index]) / gyro_sensitivity;
    processed_output_array[icm_gyro_z_index] = (float)(icm_instance->raw_data[icm_gyro_z_index]) / gyro_sensitivity;

    if ( icm_instance->enable_temperature_sensor )
    {
        processed_output_array[icm_temperature_index] = (float)(icm_instance->raw_data[icm_temperature_index]) / ICM20600_TEMPERATURE_SENSITIVITY;
        processed_output_array[icm_temperature_index] += ICM20600_TEMPERATURE_OFFSET;
    }

    return 0;
}

