/*!
 * @file itf_timers.h
 *
 * Timer 6 is used as milliseconds counter.
 * Timer
 */

#ifndef ITF_TIMERS_H_
#define ITF_TIMERS_H_

#include "interfacing.h"

// Value represents current millisecond of the system time
#define CURRENT_MILLISECOND_Val         (TIM6->CNT)

void setup_timers( void );

void setup_system_timer( void );

void delay_in_milliseconds( const uint16_t time_in_millisecond );

//! Updates seconds from startup.
void TIM6_DACUNDER_IRQHandler( void );

void TIM7_IRQHandler( void );


#endif /* ITF_TIMERS_H_ */
