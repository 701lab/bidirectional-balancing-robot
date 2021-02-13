/*!
 * @file itf_drv8701.h
 */

#ifndef ITF_DRV8701_H_
#define ITF_DRV8701_H_

#include "interfacing.h"
#include "itf_debug.h"

void setup_drv8701_1( void );
void setup_drv8701_2( void );

void setup_motor1( void );
void setup_motor2( void );

void setup_encoder1( void );
void setup_encoder2( void );

int16_t get_encoder1_value( void );
int16_t get_encoder2_value( void );

void set_drv8701_enable_high( void );
void set_drv8701_enable_low( void );

uint32_t set_drv8701_1_duty_cycle( int32_t required_duty_cycle );
uint32_t set_drv8701_2_duty_cycle( int32_t required_duty_cycle );

#endif /* ITF_DRV8701_H_ */
