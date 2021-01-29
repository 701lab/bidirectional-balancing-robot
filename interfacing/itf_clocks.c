/*!
 * @file itf_clocks.c
 *
 * Robots board v2.0 uses STM32G474RBT6 MCU.
 *
 * Interfacing library uses multiple different timers for different time-based operations:
 *  * System timer - used for control loop interrupt calls;
 *  * Timer 6 - counts milliseconds for error log;
 *  * Timer 7 - counts milliseconds for delay function.
 *
 */

#include "itf_clocks.h"


