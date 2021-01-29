/*!
 * @file itf_drv8701.c
 *
 * Robots board v2.0 uses STM32G474RBT6 MCU.
 *  PCB contains two DRV8701 DC-motor drivers and two encoders. Encoders are not related to a particular motor driver,
 * but to make things easier it is better to use Encoder1 with Motor1 and Encoder2 with Motor2, thus they will be set up together.
 *
 * Motor driver 1:
 *  PC0, PC1 - M1_IN1, M1_IN2 respectively (TIM1 CH1 and CH2 respectively, alternate function 2, PWM output mode) - Driver input signals;
 *  PB9 - M_NSLEEP (digital output) - DRV8701 sleep pin: 0 on pin - driver is disabled, 1 - enabled. The same pin for both drivers;
 *  PC13 - M1_NFAULT (external interrupt).
 *
 * Motor driver 2:
 *  PC2, PC3 - M2_IN1, M2_IN2 respectively (TIM1 CH3 and CH4 respectively, alternate function 2, PWM output mode) - Driver input signals;
 *  PB9 - M_NSLEEP (digital output) - DRV8701 sleep pin: 0 on pin - driver is disabled, 1 - enabled. The same pin for both drivers;
 *  PC15 - M2_NFAULT (external interrupt).
 *
 * Encoder 1:
 *  PB4, PB5 - encoder1 inputs 1 and 2 respectively (TIM3 CH1 and CH2 respectively, alternate function 2, ENCODER counter mode).
 *
 * Encoder 2:
 *  PB6, PB7 - encoder2 inputs 1 and 2 respectively (TIM4 CH1 and CH2 respectively, alternate function 2, ENCODER counter mode).
 */

#include "itf_drv8701.h"

