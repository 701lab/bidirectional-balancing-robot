/*!
 * @file icm20600_itf_in.h
 */

#ifndef ICM20600_ITF_IN_H_
#define ICM20600_ITF_IN_H_

#include "interfacing.h"
#include "icm20600_error_codes.h"
#include "icm20600_registers.h"
#include "math.h"

/****************************************************************************************/
/*                                                                                      */
/*                                        ENUMs                                         */
/*                                                                                      */
/****************************************************************************************/

typedef enum icm_data_order_indexes
{
    icm_accel_x_index = 0,
    icm_accel_y_index = 1,
    icm_accel_z_index = 2,
    icm_gyro_x_index = 3,
    icm_gyro_y_index = 4,
    icm_gyro_z_index = 5,
    icm_temperature_index = 6
} icm_data_order_indexes;

typedef enum icm_axes_indexes
{
    icm_x_axis_index = 0,
    icm_y_axis_index = 1,
    icm_z_axis_index = 2,
} icm_axes_indexes;

typedef enum icm_gyro_scale_variants
{
    icm_gyro_250dps_scale = 0,
    icm_gyro_500dps_scale = 1,
    icm_gyro_1000dps_scale = 2,
    icm_gyro_2000dps_scale = 3
} icm_gyro_scale_variants;

typedef enum icm_accel_scale_variants
{
    icm_accel_2g_scale = 0,
    icm_accel_4g_scale = 1,
    icm_accel_8g_scale = 2,
    icm_accel_16g_scale = 3
} icm_accel_scale_variants;

typedef enum icm_gyro_filter_bandwidth
{
    icm_gyro_250_hz = 0,
    icm_gyro_176_hz = 1,
    icm_gyro_92_hz = 2,
    icm_gyro_41_hz = 3,
    icm_gyro_20_hz = 4,
    icm_gyro_10_hz = 5,
    icm_gyro_5_hz = 6,
} icm_gyro_filter_bandwidth;

typedef enum icm_accel_filter_bandwidth
{
    icm_accel_218_hz = 1,
    icm_accel_99_hz = 2,
    icm_accel_44_hz = 3,
    icm_accel_21_hz = 4,
    icm_accel_10_hz = 5,
    icm_accel_5_hz = 6,
} icm_accel_filter_bandwidth;

typedef enum icm_accel_averaging_filter_setup
{
    icm_accel_4_samples_averaging = 0,
    icm_accel_8_samples_averaging = 1,
    icm_accel_16_samples_averaging = 2,
    icm_accel_32_samples_averaging = 3,
} icm_accel_averaging_filter_setup;

typedef enum icm_gyro_averaging_filter_setup
{
    icm_gyro_1_sample_averaging = 0,
    icm_gyro_2_samples_averaging = 1,
    icm_gyro_4_samples_averaging = 2,
    icm_gyro_8_samples_averaging = 3,
    icm_gyro_16_samples_averaging = 4,
    icm_gyro_32_samples_averaging = 5,
    icm_gyro_64_samples_averaging = 6,
    icm_gyro_128_samples_averaging = 7,
} icm_gyro_averaging_filter_setup;

/****************************************************************************************/
/*                                                                                      */
/*                                     Structures                                       */
/*                                                                                      */
/****************************************************************************************/

typedef struct
{
    // @brief Chip select high. Used in SPI communication to indicate the end of the transmission.
    //      Function must set MCU pin connected to the device CS pin logic low.
    const void (*set_cs_high)( void );

    // @brief Chip select low. Used in SPI communication to indicate start of the transmission.
    //      Function must set MCU pin connected to the device CS pin logic high.
    const void (*set_cs_low)( void );

    // @brief Sends single byte through desired SPI.
    //      Function must send input byte with the SPI connected to the desired icm-20600 device in single-byte mode. Must return byte received by SPI during transmission.
    const uint8_t (*spi_write_byte)( uint8_t byte_to_be_sent );

    float previous_gyro_values[3];
    float complementary_filter_coef; // Maybe make sense to change for an array.

    icm_gyro_scale_variants gyro_scale_setup;
    icm_gyro_filter_bandwidth gyro_filter_bw; // filter bandwidth
    icm_gyro_averaging_filter_setup gyro_averaging_setup;

    icm_accel_scale_variants accel_scale_setup;
    icm_accel_filter_bandwidth accel_filter_bw;
    icm_accel_averaging_filter_setup accel_averaging_setup;

    int16_t raw_data[7];
    int16_t gyro_calibration_coefficients[3];

    uint8_t gyro_faults[3];
    uint8_t sample_rate_divider; // output data rate = 1000/ (sample_rate_divider + 1)
    uint8_t enable_temperature_sensor; // 1 or 0

    uint8_t was_initialized;

} icm20600;

/****************************************************************************************/
/*                                                                                      */
/*                                   Initialization                                     */
/*                                                                                      */
/****************************************************************************************/

uint16_t icm20600_init( icm20600 *icm_instance ); // [V]

uint16_t icm20600_reset_all_registeres( icm20600 *icm_instance ); // [V]

uint16_t icm20600_disable_gyro( icm20600 *icm_instance ); // [V]
uint16_t icm20600_disable_accel( icm20600 *icm_instance ); // [V]

uint16_t icm20600_disable_one_gyro_channel( icm20600 *icm_instance, icm_axes_indexes channel_index ); // [V]
uint16_t icm20600_disable_one_accel_channel( icm20600 *icm_instance, icm_axes_indexes channel_index ); // [V]

uint16_t icm20600_set_calibration_values( icm20600 *icm_instance );

/****************************************************************************************/
/*                                                                                      */
/*                              Testing and calibrations                                */
/*                                                                                      */
/****************************************************************************************/

uint16_t icm20600_check_if_alive( icm20600 *icm_instance ); // [V]

/****************************************************************************************/
/*                                                                                      */
/*                                  Raw data handling                                   */
/*                                                                                      */
/****************************************************************************************/

uint16_t icm20600_get_raw_data( icm20600 *icm_instance ); // [V]

uint16_t icm20600_process_raw_data( icm20600 *icm_instance, float *processed_output_array );

/****************************************************************************************/
/*                                                                                      */
/*                                    Definitions                                       */
/*                                                                                      */
/****************************************************************************************/

//! 2^16/500 where 500 is in dps - full scale of +-250 dps gyroscope setup - the most precise one. All other scales can be based on this one.
#define icm20600_GYRO_BASIC_SENSITIVITY             (131.072f)

#define icm20600_ACC_BASIC_SENSITIVITY              (16384)       // 2^16/4 where 4 in g - full scale of +-2g accelerometer setup - the most precise one. All other scales can be based on this one.

#define ICM20600_TEMPERATURE_SENSITIVITY            (340.0f)
#define ICM20600_TEMPERATURE_OFFSET                 (36.53f)      // in degrees Celsius


#endif /* ICM20600_ITF_IN_H_ */
