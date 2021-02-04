/*
 * @file itf_clocks.h
 */

#ifndef ITF_CLOCKS_H_
#define ITF_CLOCKS_H_

#include "interfacing.h"
#include "itf_debug.h"

//! Sets up SYSCLK to SYSTEM_MAIN_FREQUENCY.
void setup_system_clock(void);

//! Sets up PLL with a given input source.
uint32_t setup_pll_as_sysclk_source(mcu_pll_source frequency_source);

//! Handles HSE fail interrupt.
void NMI_Handler(void);

#endif /* ITF_CLOCKS_H_ */
