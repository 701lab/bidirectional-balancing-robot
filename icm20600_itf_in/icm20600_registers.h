/*!
 * @file icm20600_registers.h
 */

#ifndef ICM20600_REGISTERS_H_
#define ICM20600_REGISTERS_H_

/****************************************************************************************/
/*                                                                                      */
/*                              Communication definition                                */
/*                                                                                      */
/****************************************************************************************/

#define ICM20600_READ_REG                0x80        //!< Write mask for ICM20600 register address.
#define ICM20600_READ_REG                0x00        //!< Read mask for ICM20600 register address.

#define ICM20600_ADDRESS_MASK            0x7F        //!< Address mask.

/****************************************************************************************/
/*                                                                                      */
/*                                Registers definition                                  */
/*                                                                                      */
/****************************************************************************************/

// @todo Проверить имена и расположения всех добавленных регистров.
// @todo Обернуть все адреса в (uint8_t)(0xYY) формате
// @todo Добавить недостаяющие регистры.

#define ICM20600_XG_OFFS_TC_H                0x04
#define ICM20600_XG_OFFS_TC_L                0x05
#define ICM20600_YG_OFFS_TC_H                0x07
#define ICM20600_YG_OFFS_TC_L                0x08
#define ICM20600_ZG_OFFS_TC_H                0x0A
#define ICM20600_ZG_OFFS_TC_L                0x0B

#define ICM20600_SELF_TEST_X_ACCEL           0x0D
#define ICM20600_SELF_TEST_Y_ACCEL           0x0E
#define ICM20600_SELF_TEST_Z_ACCEL           0x0F

#define ICM20600_XG_OFFS_USRH                0x13
#define ICM20600_XG_OFFS_USRL                0x14
#define ICM20600_YG_OFFS_USRH                0x15
#define ICM20600_YG_OFFS_USRL                0x16
#define ICM20600_ZG_OFFS_USRH                0x17
#define ICM20600_ZG_OFFS_USRL                0x18
#define ICM20600_SMPLRT_DIV                  0x19
#define ICM20600_CONFIG                      0x1A
#define ICM20600_GYRO_CONFIG                 0x1B
#define ICM20600_ACCEL_CONFIG_1              0x1C
#define ICM20600_ACCEL_CONFIG_2              0x1D
#define ICM20600_LP_MODE_CFG                 0x1E
#define ICM20600_ACCEL_WOM_X_THR             0x20
#define ICM20600_ACCEL_WOM_Y_THR             0x21
#define ICM20600_ACCEL_WOM_Z_THR             0x22
#define ICM20600_FIFO_EN                     0x23

#define ICM20600_FSYNC_INT                   0x36
#define ICM20600_INT_PIN_CFG                 0x37
#define ICM20600_INT_ENABLE                  0x38
#define ICM20600_FIFO_WM_INT_STATUS          0x39
#define ICM20600_INT_STATUS                  0x3A
#define ICM20600_ACCEL_XOUT_H                0x3B
#define ICM20600_ACCEL_XOUT_L                0x3C
#define ICM20600_ACCEL_YOUT_H                0x3D
#define ICM20600_ACCEL_YOUT_L                0x3E
#define ICM20600_ACCEL_ZOUT_H                0x3F
#define ICM20600_ACCEL_ZOUT_L                0x40
#define ICM20600_TEMP_OUT_H                  0x41
#define ICM20600_TEMP_OUT_L                  0x42
#define ICM20600_GYRO_XOUT_H                 0x43
#define ICM20600_GYRO_XOUT_L                 0x44
#define ICM20600_GYRO_YOUT_H                 0x45
#define ICM20600_GYRO_YOUT_L                 0x46
#define ICM20600_GYRO_ZOUT_H                 0x47
#define ICM20600_GYRO_ZOUT_L                 0x48

#define ICM20600_SELF_TEST_X_GYRO            0x50
#define ICM20600_SELF_TEST_Y_GYRO            0x51
#define ICM20600_SELF_TEST_Z_GYRO            0x52

#define ICM20600_FIFO_WM_TH1                (uint8_t)(0x60)
#define ICM20600_FIFO_WM_TH2                (uint8_t)(0x61)

#define ICM20600_SIGNAL_PATH_RESET          (uint8_t)(0x68)
#define ICM20600_ACCEL_INTEL_CTRL           (uint8_t)(0x69)
#define ICM20600_USER_CTRL                  (uint8_t)(0x6A)
#define ICM20600_PWR_MGMT_1                 (uint8_t)(0x6B)
#define ICM20600_PWR_MGMT_2                 (uint8_t)(0x6C)

#define ICM20600_I2C_IF                     (uint8_t)(0x70)

#define ICM20600_FIFO_COUNTH                (uint8_t)(0x72)
#define ICM20600_FIFO_COUNTL                (uint8_t)(0x73)
#define ICM20600_FIFO_R_W                    0x74
#define ICM20600_WHO_AM_I                    0x75

#define ICM20600_XA_OFFSET_H                 0x77
#define ICM20600_XA_OFFSET_L                 0x78

#define ICM20600_YA_OFFSET_H                 0x7A
#define ICM20600_YA_OFFSET_L                 0x7B

#define ICM20600_ZA_OFFSET_H                 0x7D
#define ICM20600_ZA_OFFSET_L                 0x7E

// Default Register values. All other values are 0 by default.
#define ICM20600_DEFAULT_CONFIG             (uint8_t)(0x80)
#define ICM20600_DEFAULT_PWR_MGMT_1         (uint8_t)(0x41)
#define TMC5130_DEFAULT_CHOPCONF            (uint8_t)(0x11)

/****************************************************************************************/
/*                                                                                      */
/*                                   Bit definitions                                    */
/*                                                                                      */
/****************************************************************************************/

// XG_OFFS_TC_H - 0x04 - RW
#define ICM20600_XG_OFFS_TC_H_XG_OFFS_TC_H_Pos      (0U)
#define ICM20600_XG_OFFS_TC_H_XG_OFFS_TC_H_Msk      (uint8_t)(0x03 << ICM20600_XG_OFFS_TC_H_XG_OFFS_TC_H_Pos)
#define ICM20600_XG_OFFS_TC_H_XG_OFFS_LP_Pos        (2U)
#define ICM20600_XG_OFFS_TC_H_XG_OFFS_LP_Msk        (uint8_t)(0x3F << ICM20600_XG_OFFS_TC_H_XG_OFFS_LP_Pos)

// XG_OFFS_TC_L - 0x05 - RW
#define ICM20600_XG_OFFS_TC_L_XG_OFFS_TC_L_Pos      (0U)
#define ICM20600_XG_OFFS_TC_L_XG_OFFS_TC_L_Msk      (uint8_t)(0xFF << ICM20600_XG_OFFS_TC_L_XG_OFFS_TC_L_Pos)




#endif /* ICM20600_REGISTERS_H_ */
