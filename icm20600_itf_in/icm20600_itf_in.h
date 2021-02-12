/*!
 * @file icm20600_itf_in.h
 */

#ifndef ICM20600_ITF_IN_H_
#define ICM20600_ITF_IN_H_

#include "interfacing.h"
#include "icm20600_error_codes.h"
#include "icm20600_registers.h"
#include "math.h"

typedef struct{

    // @brief Chip select high. Used in SPI communication to indicate the end of the transmission.
    //      Function must set MCU pin connected to the device CS pin logic low.
    const void (*set_cs_high)(void);

    // @brief Chip select low. Used in SPI communication to indicate start of the transmission.
    //      Function must set MCU pin connected to the device CS pin logic high.
    const void (*set_cs_low)(void);

    // @brief Sends single byte through desired SPI.
    //      Function must send input byte with the SPI connected to the desired icm-20600 device in single-byte mode. Must return byte received by SPI during transmission.
    const uint8_t (*spi_write_byte)(uint8_t byte_to_be_sent);

    float previous_gyro_x;
    float previous_gyro_y;
    float previous_gyro_z;

    float complementary_filter_coef;

    uint16_t raw_data[7];
    uint16_t gyro_calibration_coefficients[3];

    uint8_t gyro_scale_setup;
    uint8_t accel_scale_setup;

    uint8_t was_initialized;

    // Enables temperature sensor if 1
    uint8_t enable_temperature_sensor;

} icm20600;

#define icm20600_GYRO_BASIC_SENSITIVITY            131.072f    // 2^16/500 where 500 is in dps - full scale of +-250 dps gyroscope setup - the most precise one. All other scales can be based on this one.
#define icm20600_ACC_BASIC_SENSITIVITY             16384       // 2^16/4 where 4 in g - full scale of +-2g accelerometer setup - the most precise one. All other scales can be based on this one.
#define icm20600_TEMPERATURE_SENSITIVITY           340.0f
#define icm20600_TEMPERATURE_OFFSET                36.53f      // in degrees Celsius

//****** ICM functions ******//

uint16_t icm20600_init(icm20600 *icm_instance);

uint16_t icm20600_get_raw_data(icm20600 *icm_instance, int16_t *data_storage_array);

uint16_t icm20600_procces_raw_data(icm20600 *icm_instance, int16_t *raw_input_array, float * processed_output_array);

uint16_t icm20600_get_proccesed_data(icm20600 *icm_instance, float *processed_output_array);


// @brief send basic read message onto register with enable bit. If answer is 0 there is problems with connection to the device, or device didn't start.
uint16_t icm20600_check_if_alive(icm20600 * icm_instance);


// Должны будут в дальнейшем быть пересмотрены и по возможности улучшены
uint32_t icm20600_calculate_all_angles(icm20600 *icm_instance, float angles_storage[3], float integration_period);

// Вычисляет только нужный угол из трех. Может вызываться без вызова каких либо других функций, так как будет считывать значения датчика самостоятельно
uint32_t icm20600_calculate_z_x_angle(icm20600 *icm_instance, float *calculated_angle, float integration_period);
uint32_t icm20600_calculate_y_z_angle(icm20600 *icm_instance, float *calculated_angle, float integration_period);
uint32_t icm20600_calculate_x_y_angle(icm20600 *icm_instance, float *calculated_angle, float integration_period);


// ****** Note implemented yet
//void icm20600_registers_reset(icm20600 *icm_instance);
//
//void icm20600_disable_gyro(icm20600 *icm_instance);
//
//void icm20600_disable_accel(icm20600 *icm_instance);
//
//void icm20600_disable_one_gyro_channel(icm20600 *icm_instance, uint32_t number_of_channel_to_desable);
//
//void icm20600_disable_one_accel_channel(icm20600 *icm_instance, uint32_t number_of_channel_to_desable);


//****** ICM enums ******//

// @brief Used for icm-20600 data pointing in arrays
enum icm_data_order_indexes
{
    icm_accel_x_index = 0,
    icm_accel_y_index = 1,
    icm_accel_z_index = 2,
    icm_gyro_x_index = 3,
    icm_gyro_y_index = 4,
    icm_gyro_z_index = 5,
    icm_temperature_index = 6
};

enum icm_axises_indexes
{
    icm_x_axis_index = 0,
    icm_y_axis_index = 1,
    icm_z_axis_index = 2
};

enum icm_angles_indexes
{
    z_x_angle_index = 0,
    y_z_angle_index = 0,
    x_y_angle_index = 0
};

// @brief Gyroscope configuration parameters
enum icm_gyro_scale_variants
{
    icm_gyro_250dps_scale = 0,
    icm_gyro_500dps_scale = 1,
    icm_gyro_1000dps_scale = 2,
    icm_gyro_2000dps_scale = 3
};

// @brief Gyroscope configuration parameters
enum icm_accel_scale_variants
{
    icm_accel_2g_scale = 0,
    icm_accel_4g_scale = 1,
    icm_accel_8g_scale = 2,
    icm_accel_16g_scale = 3
};

#endif /* ICM20600_ITF_IN_H_ */
