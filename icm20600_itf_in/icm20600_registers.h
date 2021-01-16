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
// @todo Добавить недостающие регистры.

#define ICM20600_XG_OFFS_TC_H                (uint8_t)(0x04)
#define ICM20600_XG_OFFS_TC_L                (uint8_t)(0x05)

#define ICM20600_YG_OFFS_TC_H                (uint8_t)(0x07)
#define ICM20600_YG_OFFS_TC_L                (uint8_t)(0x08)

#define ICM20600_ZG_OFFS_TC_H                (uint8_t)(0x0A)
#define ICM20600_ZG_OFFS_TC_L                (uint8_t)(0x0B)

#define ICM20600_SELF_TEST_X_ACCEL           (uint8_t)(0x0D)
#define ICM20600_SELF_TEST_Y_ACCEL           (uint8_t)(0x0E)
#define ICM20600_SELF_TEST_Z_ACCEL           (uint8_t)(0x0F)

#define ICM20600_XG_OFFS_USRH                (uint8_t)(0x13)
#define ICM20600_XG_OFFS_USRL                (uint8_t)(0x14)
#define ICM20600_YG_OFFS_USRH                (uint8_t)(0x15)
#define ICM20600_YG_OFFS_USRL                (uint8_t)(0x16)
#define ICM20600_ZG_OFFS_USRH                (uint8_t)(0x17)
#define ICM20600_ZG_OFFS_USRL                (uint8_t)(0x18)
#define ICM20600_SMPLRT_DIV                  (uint8_t)(0x19)
#define ICM20600_CONFIG                      (uint8_t)(0x1A)
#define ICM20600_GYRO_CONFIG                 (uint8_t)(0x1B)
#define ICM20600_ACCEL_CONFIG                (uint8_t)(0x1C)
#define ICM20600_ACCEL_CONFIG_2              (uint8_t)(0x1D)
#define ICM20600_LP_MODE_CFG                 (uint8_t)(0x1E)
#define ICM20600_ACCEL_WOM_X_THR             (uint8_t)(0x20)
#define ICM20600_ACCEL_WOM_Y_THR             (uint8_t)(0x21)
#define ICM20600_ACCEL_WOM_Z_THR             (uint8_t)(0x22)
#define ICM20600_FIFO_EN                     (uint8_t)(0x23)

#define ICM20600_FSYNC_INT                   (uint8_t)(0x36)
#define ICM20600_INT_PIN_CFG                 (uint8_t)(0x37)
#define ICM20600_INT_ENABLE                  (uint8_t)(0x38)
#define ICM20600_FIFO_WM_INT_STATUS          (uint8_t)(0x39)
#define ICM20600_INT_STATUS                  (uint8_t)(0x3A)
#define ICM20600_ACCEL_XOUT_H                (uint8_t)(0x3B)
#define ICM20600_ACCEL_XOUT_L                (uint8_t)(0x3C)
#define ICM20600_ACCEL_YOUT_H                (uint8_t)(0x3D)
#define ICM20600_ACCEL_YOUT_L                (uint8_t)(0x3E)
#define ICM20600_ACCEL_ZOUT_H                (uint8_t)(0x3F)
#define ICM20600_ACCEL_ZOUT_L                (uint8_t)(0x40)
#define ICM20600_TEMP_OUT_H                  (uint8_t)(0x41)
#define ICM20600_TEMP_OUT_L                  (uint8_t)(0x42)
#define ICM20600_GYRO_XOUT_H                 (uint8_t)(0x43)
#define ICM20600_GYRO_XOUT_L                 (uint8_t)(0x44)
#define ICM20600_GYRO_YOUT_H                 (uint8_t)(0x45)
#define ICM20600_GYRO_YOUT_L                 (uint8_t)(0x46)
#define ICM20600_GYRO_ZOUT_H                 (uint8_t)(0x47)
#define ICM20600_GYRO_ZOUT_L                 (uint8_t)(0x48)

#define ICM20600_SELF_TEST_X_GYRO            (uint8_t)(0x50)
#define ICM20600_SELF_TEST_Y_GYRO            (uint8_t)(0x51)
#define ICM20600_SELF_TEST_Z_GYRO            (uint8_t)(0x52)

#define ICM20600_FIFO_WM_TH1                 (uint8_t)(0x60)
#define ICM20600_FIFO_WM_TH2                 (uint8_t)(0x61)

#define ICM20600_SIGNAL_PATH_RESET           (uint8_t)(0x68)
#define ICM20600_ACCEL_INTEL_CTRL            (uint8_t)(0x69)
#define ICM20600_USER_CTRL                   (uint8_t)(0x6A)
#define ICM20600_PWR_MGMT_1                  (uint8_t)(0x6B)
#define ICM20600_PWR_MGMT_2                  (uint8_t)(0x6C)

#define ICM20600_I2C_IF                      (uint8_t)(0x70)

#define ICM20600_FIFO_COUNTH                 (uint8_t)(0x72)
#define ICM20600_FIFO_COUNTL                 (uint8_t)(0x73)
#define ICM20600_FIFO_R_W                    (uint8_t)(0x74)
#define ICM20600_WHO_AM_I                    (uint8_t)(0x75)

#define ICM20600_XA_OFFSET_H                 (uint8_t)(0x77)
#define ICM20600_XA_OFFSET_L                 (uint8_t)(0x78)

#define ICM20600_YA_OFFSET_H                 (uint8_t)(0x7A)
#define ICM20600_YA_OFFSET_L                 (uint8_t)(0x7B)

#define ICM20600_ZA_OFFSET_H                 (uint8_t)(0x7D)
#define ICM20600_ZA_OFFSET_L                 (uint8_t)(0x7E)

// Default Register values. All other values are 0 by default.
#define ICM20600_DEFAULT_CONFIG              (uint8_t)(0x80)
#define ICM20600_DEFAULT_PWR_MGMT_1          (uint8_t)(0x41)
#define ICM20600_DEFAULT_WHO_AM_I            (uint8_t)(0x11)

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
#define ICM20600_XG_OFFS_TC_L_Pos                   (0U)
#define ICM20600_XG_OFFS_TC_L_Msk                   (uint8_t)(0xFF << ICM20600_XG_OFFS_TC_L_Pos)

// YG_OFFS_TC_H - 0x07 - RW
#define ICM20600_YG_OFFS_TC_H_YG_OFFS_TC_H_Pos      (0U)
#define ICM20600_YG_OFFS_TC_H_YG_OFFS_TC_H_Msk      (uint8_t)(0x03 << ICM20600_YG_OFFS_TC_H_YG_OFFS_TC_H_Pos)
#define ICM20600_YG_OFFS_TC_H_YG_OFFS_LP_Pos        (2U)
#define ICM20600_YG_OFFS_TC_H_YG_OFFS_LP_Msk        (uint8_t)(0x3F << ICM20600_YG_OFFS_TC_H_YG_OFFS_LP_Pos)

// YG_OFFS_TC_L - 0x08 - RW
#define ICM20600_YG_OFFS_TC_L_Pos                   (0U)
#define ICM20600_YG_OFFS_TC_L_Msk                   (uint8_t)(0xFF << ICM20600_YG_OFFS_TC_L_Pos)

// ZG_OFFS_TC_H - 0x0A - RW
#define ICM20600_ZG_OFFS_TC_H_ZG_OFFS_TC_H_Pos      (0U)
#define ICM20600_ZG_OFFS_TC_H_ZG_OFFS_TC_H_Msk      (uint8_t)(0x03 << ICM20600_ZG_OFFS_TC_H_ZG_OFFS_TC_H_Pos)
#define ICM20600_ZG_OFFS_TC_H_ZG_OFFS_LP_Pos        (2U)
#define ICM20600_ZG_OFFS_TC_H_ZG_OFFS_LP_Msk        (uint8_t)(0x3F << ICM20600_ZG_OFFS_TC_H_ZG_OFFS_LP_Pos)

// ZG_OFFS_TC_L - 0x0B - RW
#define ICM20600_ZG_OFFS_TC_L_Pos                   (0U)
#define ICM20600_ZG_OFFS_TC_L_Msk                   (uint8_t)(0xFF << ICM20600_ZG_OFFS_TC_L_Pos)

// SELF_TEST_X_ACCEL - 0x0D - RW
#define ICM20600_SELF_TEST_X_ACCEL_Pos              (0U)
#define ICM20600_SELF_TEST_X_ACCEL_Msk              (uint8_t)(0xFF << ICM20600_SELF_TEST_X_ACCEL_Pos)

// SELF_TEST_X_ACCEL - 0x0E - RW
#define ICM20600_SELF_TEST_Y_ACCEL_Pos              (0U)
#define ICM20600_SELF_TEST_Y_ACCEL_Msk              (uint8_t)(0xFF << ICM20600_SELF_TEST_Y_ACCEL_Pos)

// SELF_TEST_X_ACCEL - 0x0F - RW
#define ICM20600_SELF_TEST_Z_ACCEL_Pos              (0U)
#define ICM20600_SELF_TEST_Z_ACCEL_Msk              (uint8_t)(0xFF << ICM20600_SELF_TEST_Z_ACCEL_Pos)

// XG_OFFS_USRH - 0x13 - RW
#define ICM20600_XG_OFFS_USRH_Pos                   (0U)
#define ICM20600_XG_OFFS_USRH_Msk                   (uint8_t)(0xFF << ICM20600_XG_OFFS_USRH_Pos)

// XG_OFFS_USRL - 0x14 - RW
#define ICM20600_XG_OFFS_USRL_Pos                   (0U)
#define ICM20600_XG_OFFS_USRL_Msk                   (uint8_t)(0xFF << ICM20600_XG_OFFS_USRL_Pos)

// YG_OFFS_USRH - 0x15 - RW
#define ICM20600_YG_OFFS_USRH_Pos                   (0U)
#define ICM20600_YG_OFFS_USRH_Msk                   (uint8_t)(0xFF << ICM20600_YG_OFFS_USRH_Pos)

// YG_OFFS_USRL - 0x16 - RW
#define ICM20600_YG_OFFS_USRL_Pos                   (0U)
#define ICM20600_YG_OFFS_USRL_Msk                   (uint8_t)(0xFF << ICM20600_YG_OFFS_USRL_Pos)

// ZG_OFFS_USRH - 0x17 - RW
#define ICM20600_ZG_OFFS_USRH_Pos                   (0U)
#define ICM20600_ZG_OFFS_USRH_Msk                   (uint8_t)(0xFF << ICM20600_ZG_OFFS_USRH_Pos)

// ZG_OFFS_USRL - 0x18 - RW
#define ICM20600_ZG_OFFS_USRL_Pos                   (0U)
#define ICM20600_ZG_OFFS_USRL_Msk                   (uint8_t)(0xFF << ICM20600_ZG_OFFS_USRL_Pos)

// SMPLRT_DIV - 0x19 - RW
#define ICM20600_SMPLRT_DIV_Pos                     (0U)
#define ICM20600_SMPLRT_DIV_Msk                     (uint8_t)(0xFF << ICM20600_SMPLRT_DIV_Pos)

// COINFIG - 0x1A - RW
#define ICM20600_COINFIG_DLPF_CFG_Pos               (0U)
#define ICM20600_COINFIG_DLPF_CFG_Msk               (uint8_t)(0x07 << ICM20600_COINFIG_DLPF_CFG_Pos)
#define ICM20600_COINFIG_EXT_SYNC_SET_Pos           (3U)
#define ICM20600_COINFIG_EXT_SYNC_SET_Msk           (uint8_t)(0x07 << ICM20600_COINFIG_EXT_SYNC_SET_Pos)
#define ICM20600_COINFIG_FIFO_MODE_Pos              (6U)
#define ICM20600_COINFIG_FIFO_MODE_Msk              (uint8_t)(0x01 << CM20600_COINFIG_FIFO_MODE_Pos)
#define ICM20600_COINFIG_FIFO_MODE                  CM20600_COINFIG_FIFO_MODE_Msk


// GYRO_CONFIG - 0x1B - RW
#define ICM20600_GYRO_CONFIG_FCHOICE_B_Pos          (0U)
#define ICM20600_GYRO_CONFIG_FCHOICE_B_Msk          (uint8_t)(0x03 << ICM20600_GYRO_CONFIG_FCHOICE_B_Pos)
#define ICM20600_GYRO_CONFIG_FS_SEL_Pos             (3U)
#define ICM20600_GYRO_CONFIG_FS_SEL_Msk             (uint8_t)(0x03 << ICM20600_GYRO_CONFIG_FS_SEL_Pos)
#define ICM20600_GYRO_CONFIG_ZG_ST_Pos              (5U)
#define ICM20600_GYRO_CONFIG_ZG_ST_Msk              (uint8_t)(0x01 << ICM20600_GYRO_CONFIG_ZG_ST_Pos)
#define ICM20600_GYRO_CONFIG_ZG_ST                  ICM20600_GYRO_CONFIG_ZG_ST_Msk
#define ICM20600_GYRO_CONFIG_YG_ST_Pos              (6U)
#define ICM20600_GYRO_CONFIG_YG_ST_Msk              (uint8_t)(0x01 << ICM20600_GYRO_CONFIG_YG_ST_Pos)
#define ICM20600_GYRO_CONFIG_YG_ST                  ICM20600_GYRO_CONFIG_YG_ST_Msk
#define ICM20600_GYRO_CONFIG_XG_ST_Pos              (7U)
#define ICM20600_GYRO_CONFIG_XG_ST_Msk              (uint8_t)(0x01 << ICM20600_GYRO_CONFIG_XG_ST_Pos)
#define ICM20600_GYRO_CONFIG_XG_ST                  ICM20600_GYRO_CONFIG_XG_ST_Msk

// ACCEL_CONFIG - 0x1C - RW
#define ICM20600_ACCEL_CONFIG_ACCEL_FS_SEL_Pos      (3U)
#define ICM20600_ACCEL_CONFIG_ACCEL_FS_SEL_Msk      (uint8_t)(0x03 << ICM20600_ACCEL_CONFIG_ACCEL_FS_SEL_Pos)
#define ICM20600_ACCEL_CONFIG_ZA_ST_Pos             (5U)
#define ICM20600_ACCEL_CONFIG_ZA_ST_Msk             (uint8_t)(0x01 << ICM20600_ACCEL_CONFIG_ZA_ST_Pos)
#define ICM20600_ACCEL_CONFIG_ZA_ST                 ICM20600_ACCEL_CONFIG_ZA_ST_Msk
#define ICM20600_ACCEL_CONFIG_YA_ST_Pos             (6U)
#define ICM20600_ACCEL_CONFIG_YA_ST_Msk             (uint8_t)(0x01 << ICM20600_ACCEL_CONFIG_YA_ST_Pos)
#define ICM20600_ACCEL_CONFIG_YA_ST                 ICM20600_ACCEL_CONFIG_YA_ST_Msk
#define ICM20600_ACCEL_CONFIG_XA_ST_Pos             (7U)
#define ICM20600_ACCEL_CONFIG_XA_ST_Msk             (uint8_t)(0x01 << ICM20600_ACCEL_CONFIG_XA_ST_Pos)
#define ICM20600_ACCEL_CONFIG_XA_ST                 ICM20600_ACCEL_CONFIG_XA_ST_Msk

// ACCEL_CONFIG2 - 0x1D - RW
#define ICM20600_ACCEL_CONFIG2_A_DLPF_CFG_Pos       (0U)
#define ICM20600_ACCEL_CONFIG2_A_DLPF_CFG_Msk       (uint8_t)(0x07 << ICM20600_ACCEL_CONFIG2_A_DLPF_CFG_Pos)
#define ICM20600_ACCEL_CONFIG2_ACCEL_FCHOICE_B_Pos  (3U)
#define ICM20600_ACCEL_CONFIG2_ACCEL_FCHOICE_B_Msk  (uint8_t)(0x01 << ICM20600_ACCEL_CONFIG2_ACCEL_FCHOICE_B_Pos)
#define ICM20600_ACCEL_CONFIG2_ACCEL_FCHOICE_B      ICM20600_ACCEL_CONFIG2_ACCEL_FCHOICE_B_Msk
#define ICM20600_ACCEL_CONFIG2_DEC2_CFG_Pos         (4U)
#define ICM20600_ACCEL_CONFIG2_DEC2_CFG_Msk         (uint8_t)(0x03 << ICM20600_ACCEL_CONFIG2_DEC2_CFG_Pos)

// LP_MODE_CFG - 0x1E - RW
#define ICM20600_LP_MODE_CFG_G_AVGCFG_Pos           (4U)
#define ICM20600_LP_MODE_CFG_G_AVGCFG_Msk           (uint8_t)(0x07 << ICM20600_LP_MODE_CFG_G_AVGCFG_Pos)
#define ICM20600_LP_MODE_CFG_GYRO_CYCLE_Pos         (7U)
#define ICM20600_LP_MODE_CFG_GYRO_CYCLE_Msk         (uint8_t)(0x01 << ICM20600_LP_MODE_CFG_GYRO_CYCLE_Pos)
#define ICM20600_LP_MODE_CFG_GYRO_CYCLE             ICM20600_LP_MODE_CFG_GYRO_CYCLE_Msk

// ACCEL_WOM_X_THR - 0x20 -RW
#define ICM20600_ACCEL_WOM_X_THR_Pos                (0U)
#define ICM20600_ACCEL_WOM_X_THR_Msk                (uint8_t)(0xFF << ICM20600_ACCEL_WOM_X_THR_Pos)

// ACCEL_WOM_Y_THR - 0x21 -RW
#define ICM20600_ACCEL_WOM_Y_THR_Pos                (0U)
#define ICM20600_ACCEL_WOM_Y_THR_Msk                (uint8_t)(0xFF << ICM20600_ACCEL_WOM_Y_THR_Pos)

// ACCEL_WOM_Z_THR - 0x22 -RW
#define ICM20600_ACCEL_WOM_Z_THR_Pos                (0U)
#define ICM20600_ACCEL_WOM_Z_THR_Msk                (uint8_t)(0xFF << ICM20600_ACCEL_WOM_Z_THR_Pos)

// FIFO_EN - 0x23 - RW
#define ICM20600_FIFO_EN_ACCEL_FIFO_EN_Pos          (3U)
#define ICM20600_FIFO_EN_ACCEL_FIFO_EN_Msk          (uint8_t)(0x01 << ICM20600_FIFO_EN_ACCEL_FIFO_EN_Pos)
#define ICM20600_FIFO_EN_ACCEL_FIFO_EN              ICM20600_FIFO_EN_ACCEL_FIFO_EN_Msk
#define ICM20600_FIFO_EN_GYRO_FIFO_EN_Pos           (4U)
#define ICM20600_FIFO_EN_GYRO_FIFO_EN_Pos           (uint8_t)(0x01 << ICM20600_FIFO_EN_GYRO_FIFO_EN_Pos)
#define ICM20600_FIFO_EN_GYRO_FIFO_EN               ICM20600_FIFO_EN_GYRO_FIFO_EN_Msk

// FSYNC_INT - 0x36 - RC
#define ICM20600_FSYNC_INT_Pos                      (0U)
#define ICM20600_FSYNC_INT_Msk                      (uint8_t)(0xFF << ICM20600_FSYNC_INT_Pos)

// INT_PIN_CFG - 0x37 - RW
#define ICM20600_INT_PIN_CFG_INT2_EN_Pos            (0U)
#define ICM20600_INT_PIN_CFG_INT2_EN_Msk            (uint8_t)(0x01 << ICM20600_INT_PIN_CFG_INT2_EN_Pos)
#define ICM20600_INT_PIN_CFG_INT2_EN                ICM20600_INT_PIN_CFG_INT2_EN_Msk

#define ICM20600_INT_PIN_CFG_FSYNC_INT_MODE_EN_Pos  (2U)
#define ICM20600_INT_PIN_CFG_FSYNC_INT_MODE_EN_Msk  (uint8_t)(0x01 << ICM20600_INT_PIN_CFG_FSYNC_INT_MODE_EN_Pos)
#define ICM20600_INT_PIN_CFG_FSYNC_INT_MODE_EN      ICM20600_INT_PIN_CFG_FSYNC_INT_MODE_EN_Msk
#define ICM20600_INT_PIN_CFG_FSYNC_INT_LEVEL_Pos    (3U)
#define ICM20600_INT_PIN_CFG_FSYNC_INT_LEVEL_Pos    (uint8_t)(0x01 << ICM20600_INT_PIN_CFG_FSYNC_INT_LEVEL_Pos)
#define ICM20600_INT_PIN_CFG_FSYNC_INT_LEVEL        ICM20600_INT_PIN_CFG_FSYNC_INT_LEVEL_Msk
#define ICM20600_INT_PIN_CFG_INT_RD_CLEAR_Pos       (4U)
#define ICM20600_INT_PIN_CFG_INT_RD_CLEAR_Msk       (uint8_t)(0x01 << ICM20600_INT_PIN_CFG_INT_RD_CLEAR_Pos)
#define ICM20600_INT_PIN_CFG_INT_RD_CLEAR           ICM20600_INT_PIN_CFG_INT_RD_CLEAR_Msk
#define ICM20600_INT_PIN_CFG_LATCH_INT_EN_Pos       (5U)
#define ICM20600_INT_PIN_CFG_LATCH_INT_EN_Msk       (uint8_t)(0x01 << ICM20600_INT_PIN_CFG_LATCH_INT_EN_Pos)
#define ICM20600_INT_PIN_CFG_LATCH_INT_EN           ICM20600_INT_PIN_CFG_LATCH_INT_EN_Msk
#define ICM20600_INT_PIN_CFG_INT_OPEN_Pos           (6U)
#define ICM20600_INT_PIN_CFG_INT_OPEN_Msk           (uint8_t)(0x01 << ICM20600_INT_PIN_CFG_INT_OPEN_Pos)
#define ICM20600_INT_PIN_CFG_INT_OPEN               ICM20600_INT_PIN_CFG_INT_OPEN_Msk
#define ICM20600_INT_PIN_CFG_INT_LEVEL_Pos          (7U)
#define ICM20600_INT_PIN_CFG_INT_LEVEL_Msk          (uint8_t)(0x01 << ICM20600_INT_PIN_CFG_INT_LEVEL_Pos)
#define ICM20600_INT_PIN_CFG_INT_LEVEL              ICM20600_INT_PIN_CFG_INT_LEVEL_Msk

// FIFO_WM_INT_STATUS - 0x39 - RC
#define ICM20600_FIFO_WM_INT_STATUS_Pos             (0U)
#define ICM20600_FIFO_WM_INT_STATUS_Msk             (uint8_t)(0x7F << ICM20600_FIFO_WM_INT_STATUS_Pos)

// INT_STATUS - 0x3A - RC
#define ICM20600_INT_STATUS_DATA_RDY_INT_Pos        (0U)
#define ICM20600_INT_STATUS_DATA_RDY_INT_Msk        (uint8_t)(0x01 << ICM20600_INT_STATUS_DATA_RDY_INT_Pos)
#define ICM20600_INT_STATUS_DATA_RDY_INT            ICM20600_INT_STATUS_DATA_RDY_INT_Msk
#define ICM20600_INT_STATUS_GDRIVE_INT_Pos          (2U)
#define ICM20600_INT_STATUS_GDRIVE_INT_Msk          (uint8_t)(0x01 << ICM20600_INT_STATUS_GDRIVE_INT_Pos)
#define ICM20600_INT_STATUS_GDRIVE_INT              ICM20600_INT_STATUS_GDRIVE_INT_Msk
#define ICM20600_INT_STATUS_FIFO_OFLOW_INT_Pos      (4U)
#define ICM20600_INT_STATUS_FIFO_OFLOW_INT_Msk      (uint8_t)(0x01 << ICM20600_INT_STATUS_FIFO_OFLOW_INT_Pos)
#define ICM20600_INT_STATUS_FIFO_OFLOW_INT          ICM20600_INT_STATUS_FIFO_OFLOW_INT_Msk
#define ICM20600_INT_STATUS_WOM_Z_INT_Pos           (5U)
#define ICM20600_INT_STATUS_WOM_Z_INT_Msk           (uint8_t)(0x01 << ICM20600_INT_STATUS_WOM_Z_INT_Pos)
#define ICM20600_INT_STATUS_WOM_Z_INT               ICM20600_INT_STATUS_WOM_Z_INT_Msk
#define ICM20600_INT_STATUS_WOM_Y_INT_Pos           (6U)
#define ICM20600_INT_STATUS_WOM_Y_INT_Msk           (uint8_t)(0x01 << ICM20600_INT_STATUS_WOM_Y_INT_Pos)
#define ICM20600_INT_STATUS_WOM_Y_INT               ICM20600_INT_STATUS_WOM_Y_INT_Msk
#define ICM20600_INT_STATUS_WOM_X_INT_Pos           (7U)
#define ICM20600_INT_STATUS_WOM_X_INT_Msk           (uint8_t)(0x01 << ICM20600_INT_STATUS_WOM_X_INT_Pos)
#define ICM20600_INT_STATUS_WOM_X_INT               ICM20600_INT_STATUS_WOM_X_INT_Msk


// ACCEL_XOUT_H - 0x3B - R
#define ICM20600_ACCEL_XOUT_H_Pos                   (0U)
#define ICM20600_ACCEL_XOUT_H_Msk                   (uint8_t)(0xFF << ICM20600_ACCEL_XOUT_H_Pos)

// ACCEL_XOUT_L - 0x3C - R
#define ICM20600_ACCEL_XOUT_L_Pos                   (0U)
#define ICM20600_ACCEL_XOUT_L_Msk                   (uint8_t)(0xFF << ICM20600_ACCEL_XOUT_L_Pos)

// ACCEL_YOUT_H - 0x3D - R
#define ICM20600_ACCEL_YOUT_H_Pos                   (0U)
#define ICM20600_ACCEL_YOUT_H_Msk                   (uint8_t)(0xFF << ICM20600_ACCEL_YOUT_H_Pos)

// ACCEL_YOUT_L - 0x3E - R
#define ICM20600_ACCEL_YOUT_L_Pos                   (0U)
#define ICM20600_ACCEL_YOUT_L_Msk                   (uint8_t)(0xFF << ICM20600_ACCEL_YOUT_L_Pos)

// ACCEL_ZOUT_H - 0x3F - R
#define ICM20600_ACCEL_ZOUT_H_Pos                   (0U)
#define ICM20600_ACCEL_ZOUT_H_Msk                   (uint8_t)(0xFF << ICM20600_ACCEL_ZOUT_H_Pos)

// ACCEL_ZOUT_L - 0x40 - R
#define ICM20600_ACCEL_ZOUT_L_Pos                   (0U)
#define ICM20600_ACCEL_ZOUT_L_Msk                   (uint8_t)(0xFF << ICM20600_ACCEL_ZOUT_L_Pos)

// TEMP_OUT_H - 0x41 - R
#define ICM20600_TEMP_OUT_H_Pos                     (0U)
#define ICM20600_TEMP_OUT_H_Msk                     (uint8_t)(0xFF << ICM20600_TEMP_OUT_H_Pos)

// TEMP_OUT_L - 0x42 - R
#define ICM20600_TEMP_OUT_L_Pos                     (0U)
#define ICM20600_TEMP_OUT_L_Msk                     (uint8_t)(0xFF << ICM20600_TEMP_OUT_L_Pos)

// GYRO_XOUT_H - 0x43 - R
#define ICM20600_GYRO_XOUT_H_Pos                    (0U)
#define ICM20600_GYRO_XOUT_H_Msk                    (uint8_t)(0xFF << ICM20600_GYRO_XOUT_H_Pos)

// GYRO_XOUT_L - 0x44 - R
#define ICM20600_GYRO_XOUT_L_Pos                    (0U)
#define ICM20600_GYRO_XOUT_L_Msk                    (uint8_t)(0xFF << ICM20600_GYRO_XOUT_L_Pos)

// GYRO_YOUT_H - 0x45 - R
#define ICM20600_GYRO_YOUT_H_Pos                    (0U)
#define ICM20600_GYRO_YOUT_H_Msk                    (uint8_t)(0xFF << ICM20600_GYRO_YOUT_H_Pos)

// GYRO_YOUT_L - 0x46 - R
#define ICM20600_GYRO_YOUT_L_Pos                    (0U)
#define ICM20600_GYRO_YOUT_L_Msk                    (uint8_t)(0xFF << ICM20600_GYRO_YOUT_L_Pos)

// GYRO_ZOUT_H - 0x47 - R
#define ICM20600_GYRO_ZOUT_H_Pos                    (0U)
#define ICM20600_GYRO_ZOUT_H_Msk                    (uint8_t)(0xFF << ICM20600_GYRO_ZOUT_H_Pos)

// GYRO_ZOUT_L - 0x48 - R
#define ICM20600_GYRO_ZOUT_L_Pos                    (0U)
#define ICM20600_GYRO_ZOUT_L_Msk                    (uint8_t)(0xFF << ICM20600_GYRO_ZOUT_L_Pos)

// SELF_TEST_X_GYRO - 0x50 - RW
#define ICM20600_SELF_TEST_X_GYRO_Pos               (0U)
#define ICM20600_SELF_TEST_X_GYRO_Msk               (uint8_t)(0xFF << ICM20600_SELF_TEST_X_GYRO_Pos)

// SELF_TEST_Y_GYRO - 0x51 - RW
#define ICM20600_SELF_TEST_Y_GYRO_Pos               (0U)
#define ICM20600_SELF_TEST_Y_GYRO_Msk               (uint8_t)(0xFF << ICM20600_SELF_TEST_Y_GYRO_Pos)

// SELF_TEST_Z_GYRO - 0x52 - RW
#define ICM20600_SELF_TEST_Z_GYRO_Pos               (0U)
#define ICM20600_SELF_TEST_Z_GYRO_Msk               (uint8_t)(0xFF << ICM20600_SELF_TEST_Z_GYRO_Pos)

// FIFO_WM_TH1 - 0x60 - RW
#define ICM20600_FIFO_WM_TH1_Pos                    (0U)
#define ICM20600_FIFO_WM_TH1_Msk                    (uint8_t)(0x03 << ICM20600_FIFO_WM_TH1_Pos)

// FIFO_WM_TH2 - 0x61 - RW
#define ICM20600_FIFO_WM_TH2_Pos                    (0U)
#define ICM20600_FIFO_WM_TH2_Msk                    (uint8_t)(0xFF << ICM20600_FIFO_WM_TH2_Pos)

// SIGNAL_PATH_RESET - 0x68 - RW
#define ICM20600_SIGNAL_PATH_RESET_TEMP_RST_Pos     (0U)
#define ICM20600_SIGNAL_PATH_RESET_TEMP_RST_Msk     (uint8_t)(0x01 << ICM20600_SIGNAL_PATH_RESET_TEMP_RST_Pos)
#define ICM20600_SIGNAL_PATH_RESET_TEMP_RST         ICM20600_SIGNAL_PATH_RESET_TEMP_RST_Msk
#define ICM20600_SIGNAL_PATH_RESET_ACCEL_RST_Pos    (1U)
#define ICM20600_SIGNAL_PATH_RESET_ACCEL_RST_Msk    (uint8_t)(0x01 << ICM20600_SIGNAL_PATH_RESET_ACCEL_RST_Pos)
#define ICM20600_SIGNAL_PATH_RESET_ACCEL_RST        ICM20600_SIGNAL_PATH_RESET_ACCEL_RST_Msk

// ACCEL_INTEL_CTRL - 0x69 - RW
#define ICM20600_ACCEL_INTEL_CTRL_WOM_TH_MODE_Pos   (0U)
#define ICM20600_ACCEL_INTEL_CTRL_WOM_TH_MODE_Msk   (uint8_t)(0x01 << ICM20600_ACCEL_INTEL_CTRL_WOM_TH_MODE_Pos)
#define ICM20600_ACCEL_INTEL_CTRL_WOM_TH_MODE       ICM20600_ACCEL_INTEL_CTRL_WOM_TH_MODE_Msk
#define ICM20600_ACCEL_INTEL_CTRL_OUTPUT_LIMIT_Pos  (1U)
#define ICM20600_ACCEL_INTEL_CTRL_OUTPUT_LIMIT_Msk  (uint8_t)(0x01 << ICM20600_ACCEL_INTEL_CTRL_OUTPUT_LIMIT_Pos)
#define ICM20600_ACCEL_INTEL_CTRL_OUTPUT_LIMIT      ICM20600_ACCEL_INTEL_CTRL_OUTPUT_LIMIT_Msk
#define ICM20600_ACCEL_INTEL_CTRL_ACCEL_INTEL_MODE_Pos  (6U)
#define ICM20600_ACCEL_INTEL_CTRL_ACCEL_INTEL_MODE_Msk  (uint8_t)(0x01 << ICM20600_ACCEL_INTEL_CTRL_ACCEL_INTEL_MODE_Pos)
#define ICM20600_ACCEL_INTEL_CTRL_ACCEL_INTEL_MODE      ICM20600_ACCEL_INTEL_CTRL_ACCEL_INTEL_MODE_Msk
#define ICM20600_ACCEL_INTEL_CTRL_ACCEL_INTEL_EN_Pos    (7U)
#define ICM20600_ACCEL_INTEL_CTRL_ACCEL_INTEL_EN_Msk    (uint8_t)(0x01 << ICM20600_ACCEL_INTEL_CTRL_ACCEL_INTEL_EN_Pos)
#define ICM20600_ACCEL_INTEL_CTRL_ACCEL_INTEL_EN        ICM20600_ACCEL_INTEL_CTRL_ACCEL_INTEL_EN_Msk

// USER_CTRL - 0x6A - RW
#define ICM20600_USER_CTRL_SIG_COND_RST_Pos         (0U)
#define ICM20600_USER_CTRL_SIG_COND_RST_Msk         (uint8_t)(0x01 << ICM20600_USER_CTRL_SIG_COND_RST_Pos)
#define ICM20600_USER_CTRL_SIG_COND_RST             ICM20600_USER_CTRL_SIG_COND_RST_Msk
#define ICM20600_USER_CTRL_FIFO_RST_Pos             (2U)
#define ICM20600_USER_CTRL_FIFO_RST_Msk             (uint8_t)(0x01 << ICM20600_USER_CTRL_FIFO_RST_Pos)
#define ICM20600_USER_CTRL_FIFO_RST                 ICM20600_USER_CTRL_FIFO_RST_Msk
#define ICM20600_USER_CTRL_FIFO_EN_Pos              (6U)
#define ICM20600_USER_CTRL_FIFO_EN_Msk              (uint8_t)(0x01 << ICM20600_USER_CTRL_FIFO_EN_Pos)
#define ICM20600_USER_CTRL_FIFO_WN                  ICM20600_USER_CTRL_FIFO_EN_Msk

// PWR_MGMT_1 - 0x6B - RW
#define ICM20600_PWR_MGMT_1_CLKSEL_Pos              (0U)
#define ICM20600_PWR_MGMT_1_CLKSEL_Msk              (uint8_t)(0x07 << ICM20600_PWR_MGMT_1_CLKSEL_Pos)
#define ICM20600_PWR_MGMT_1_TEMP_DIS_Pos            (3U)
#define ICM20600_PWR_MGMT_1_TEMP_DIS_Msk            (uint8_t)(0x01 << ICM20600_PWR_MGMT_1_TEMP_DIS_Pos)
#define ICM20600_PWR_MGMT_1_TEMP_DIS                ICM20600_PWR_MGMT_1_TEMP_DIS_Msk
#define ICM20600_PWR_MGMT_1_GYRO_STANDBY_Pos        (4U)
#define ICM20600_PWR_MGMT_1_GYRO_STANDBY_Msk        (uint8_t)(0x01 << ICM20600_PWR_MGMT_1_GYRO_STANDBY_Pos)
#define ICM20600_PWR_MGMT_1_GYRO_STANDBY            ICM20600_PWR_MGMT_1_GYRO_STANDBY_Msk
#define ICM20600_PWR_MGMT_1_CYCLE_Pos               (5U)
#define ICM20600_PWR_MGMT_1_CYCLE_Msk               (uint8_t)(0x01 << ICM20600_PWR_MGMT_1_CYCLE_Pos)
#define ICM20600_PWR_MGMT_1_CYCLE                   ICM20600_PWR_MGMT_1_CYCLE_Msk
#define ICM20600_PWR_MGMT_1_SLEEP_Pos               (6U)
#define ICM20600_PWR_MGMT_1_SLEEP_Msk               (uint8_t)(0x01 << ICM20600_PWR_MGMT_1_SLEEP_Pos)
#define ICM20600_PWR_MGMT_1_SLEEP                   ICM20600_PWR_MGMT_1_SLEEP_Msk
#define ICM20600_PWR_MGMT_1_DEVICE_RESET_Pos        (7U)
#define ICM20600_PWR_MGMT_1_DEVICE_RESET_Msk        (uint8_t)(0x01 << ICM20600_PWR_MGMT_1_DEVICE_RESET_Pos)
#define ICM20600_PWR_MGMT_1_DEVICE_RESET            ICM20600_PWR_MGMT_1_DEVICE_RESET_Msk

// PWR_MGMT_2 - 0x6C - RW
#define ICM20600_PWR_MGMT_2_STBY_ZG_Pos             (0U)
#define ICM20600_PWR_MGMT_2_STBY_ZG_Msk             (uint8_t)(0x01 << ICM20600_PWR_MGMT_2_STBY_ZG_Pos)
#define ICM20600_PWR_MGMT_2_STBY_ZG                 ICM20600_PWR_MGMT_2_STBY_ZG_Msk
#define ICM20600_PWR_MGMT_2_STBY_YG_Pos             (1U)
#define ICM20600_PWR_MGMT_2_STBY_YG_Msk             (uint8_t)(0x01 << ICM20600_PWR_MGMT_2_STBY_YG_Pos)
#define ICM20600_PWR_MGMT_2_STBY_YG                 ICM20600_PWR_MGMT_2_STBY_YG_Msk
#define ICM20600_PWR_MGMT_2_STBY_XG_Pos             (2U)
#define ICM20600_PWR_MGMT_2_STBY_XG_Msk             (uint8_t)(0x01 << ICM20600_PWR_MGMT_2_STBY_XG_Pos)
#define ICM20600_PWR_MGMT_2_STBY_XG                 ICM20600_PWR_MGMT_2_STBY_XG_Msk
#define ICM20600_PWR_MGMT_2_STBY_ZA_Pos             (3U)
#define ICM20600_PWR_MGMT_2_STBY_ZA_Msk             (uint8_t)(0x01 << ICM20600_PWR_MGMT_2_STBY_ZA_Pos)
#define ICM20600_PWR_MGMT_2_STBY_ZA                 ICM20600_PWR_MGMT_2_STBY_ZA_Msk
#define ICM20600_PWR_MGMT_2_STBY_YA_Pos             (4U)
#define ICM20600_PWR_MGMT_2_STBY_YA_Msk             (uint8_t)(0x01 << ICM20600_PWR_MGMT_2_STBY_YA_Pos)
#define ICM20600_PWR_MGMT_2_STBY_YA                 ICM20600_PWR_MGMT_2_STBY_YA_Msk
#define ICM20600_PWR_MGMT_2_STBY_XA_Pos             (5U)
#define ICM20600_PWR_MGMT_2_STBY_XA_Msk             (uint8_t)(0x01 << ICM20600_PWR_MGMT_2_STBY_XA_Pos)
#define ICM20600_PWR_MGMT_2_STBY_XA                 ICM20600_PWR_MGMT_2_STBY_XA_Msk

// I2C_IF - 0x70 - RW
#define ICM20600_I2C_IF_I2C_IF_DIS_Pos              (6U)
#define ICM20600_I2C_IF_I2C_IF_DIS_Msk              (uint8_t)(0x01 << ICM20600_I2C_IF_I2C_IF_DIS_Pos)
#define ICM20600_I2C_IF_I2C_IF_DIS                  ICM20600_I2C_IF_I2C_IF_DIS_Msk

// FIFO_COUNTH - 0x72 - R
#define ICM20600_FIFO_COUNTH_Pos                    (0U)
#define ICM20600_FIFO_COUNTH_Msk                    (uint8_t)(0xFF << ICM20600_FIFO_COUNTH_Pos)

// FIFO_COUNTL - 0x73 - R
#define ICM20600_FIFO_COUNTL_Pos                    (0U)
#define ICM20600_FIFO_COUNTH_Msk                    (uint8_t)(0xFF << ICM20600_FIFO_COUNTH_Pos)

// FIFO_R_W - 0x74 - RW
#define ICM20600_FIFO_R_W_Pos                       (0U)
#define ICM20600_FIFO_R_W_Msk                       (uint8_t)(0xFF << ICM20600_FIFO_R_W_Pos)

// WHO_AM_I - 0x75 - R
#define ICM20600_WHO_AM_I_Pos                       (0U)
#define ICM20600_WHO_AM_I_Msk                       (uint8_t)(0xFF << ICM20600_WHO_AM_I_Pos)

// XA_OFFSET_H - 0x77 - RW
#define ICM20600_XA_OFFSET_H_Pos                    (0U)
#define ICM20600_XA_OFFSET_H_Msk                    (uint8_t)(0xFF << ICM20600_XA_OFFSET_H_Pos)

// XA_OFFSET_L - 0x78 - RW
#define ICM20600_XA_OFFSET_L_Pos                    (1U)
#define ICM20600_XA_OFFSET_L_Msk                    (uint8_t)(0x7F << ICM20600_XA_OFFSET_L_Pos)

// YA_OFFSET_H - 0x7A - RW
#define ICM20600_YA_OFFSET_H_Pos                    (0U)
#define ICM20600_YA_OFFSET_H_Msk                    (uint8_t)(0xFF << ICM20600_YA_OFFSET_H_Pos)

// YA_OFFSET_L - 0x7B - RW
#define ICM20600_YA_OFFSET_L_Pos                    (1U)
#define ICM20600_YA_OFFSET_L_Msk                    (uint8_t)(0x7F << ICM20600_YA_OFFSET_L_Pos)

// ZA_OFFSET_H - 0x7D - RW
#define ICM20600_ZA_OFFSET_H_Pos                    (0U)
#define ICM20600_ZA_OFFSET_H_Msk                    (uint8_t)(0xFF << ICM20600_ZA_OFFSET_H_Pos)

// ZA_OFFSET_L - 0x7E - RW
#define ICM20600_ZA_OFFSET_L_Pos                    (1U)
#define ICM20600_ZA_OFFSET_L_Msk                    (uint8_t)(0x7F << ICM20600_ZA_OFFSET_L_Pos)

#endif /* ICM20600_REGISTERS_H_ */
