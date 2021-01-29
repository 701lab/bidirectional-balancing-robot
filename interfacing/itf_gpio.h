/*!
 * @file itf_gpio.h
 */

#ifndef ITF_GPIO_H_
#define ITF_GPIO_H_

#include "interfacing.h"

void setup_leds( void );

void toggle_d2_led( void );
void turn_on_d2_led( void );
void turn_off_d2_led( void );

void toggle_d3_led( void );
void turn_on_d3_led( void );
void turn_off_d3_led( void );

void toggle_d4_led( void );
void turn_on_d4_led( void );
void turn_off_d4_led( void );

void toggle_all_leds( void );
void turn_off_all_leds( void );
void turn_on_all_leds( void );

#endif /* ITF_GPIO_H_ */
