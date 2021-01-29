/*!
 * @file itf_icm20600.c
 *
 * Robots board v2.0 uses STM32G474RBT6 MCU.
 *  PCB contains ICM-20600 3 axis gyroscope and accelerometer.
 *
 * ICM-20600:
 *  PB13-PC15 - SCK, MISO, MOSI respectively (SPI2, alternate function 5);
 *  PC6 - ICM_CS (digital output) - ICM-20600 chip selected;
 *  PB11 - ICM_INT2 (external interrupt input) - ICM-20600 interrupt output 2;
 *  PB12 - ICM_INT1 (external interrupt input) - ICM-20600 interrupt output 1;
 */

#include "itf_icm20600.h"


