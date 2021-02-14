/*!
 * @file device.h
 *
 * Contains the main, system related definitions and function declarations.
 *  Unlike interfacing libraries, this file can be freely updated and changed in order to get the desired behavior.
 */

#ifndef DEVICE_H_
#define DEVICE_H_

// Interfacing includes
#include "itf_gpio.h"
#include "itf_clocks.h"
#include "itf_drv8701.h"
#include "itf_icm20600.h"
#include "itf_nrf24l01p.h"

// Hardware libraries includes
#include "icm20600_itf_in.h"
#include "drv8701_itf_in.h"
#include "nrf24l01p_impi.h" // Old Fashioned library.
#include "motors.h" // Super old fashioned library.

// Other includes
#include "math.h"


//! Sets up all project related peripherals.
void setup_device(void);

void dummy_delay(uint32_t duration);

void calibrate_icm20600_gyro( icm20600 *icm_instance, uint8_t calibration_coef, uint32_t cycle_length );

#endif /* DEVICE_H_ */
