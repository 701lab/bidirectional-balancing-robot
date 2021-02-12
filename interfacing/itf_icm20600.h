/*
 * @file itf_icm20600.h
 */

#ifndef ITF_ICM20600_H_
#define ITF_ICM20600_H_

#include "interfacing.h"
#include "itf_debug.h"

void setup_icm20600_peripherals( void );

uint8_t spi2_write_single_byte(const uint8_t byte_to_send);

void set_icm20600_cs_high( void );
void set_icm20600_cs_low( void );

#endif /* ITF_ICM20600_H_ */
