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
 *
 * @todo This code does not implements interrupt handling. It must be implemented in the future.
 *
 */

#include "itf_drv8701.h"


void setup_drv8701_1( void )
{

}

void setup_drv8701_2( void )
{

}

void setup_encoder1( void )
{
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM3EN;

    TIM3->PSC = 0;
    TIM3->ARR = 0xFFFF;      // encoder timer must be set up with maximum int16_t value in order to use uint16_t overflow in calculations of position and speed.
//    TIM3->CCER |= 0x02;     // Change counting direction. Should be uncommented if encoder direction reversal is needed
    TIM3->SMCR |= 0x03;     // Set encoder mode.
    TIM3->CNT = 0;          // Clear counter before start.
    TIM3->CR1 |= TIM_CR1_CEN;
}

void setup_encoder2( void )
{
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM4EN;

    TIM4->PSC = 0;
    TIM4->ARR = 0xFFFF;      // encoder timer must be set up with maximum int16_t value in order to use uint16_t overflow in calculations of position and speed.
//    TIM4->CCER |= 0x02;     // Change counting direction. Should be uncommented if encoder direction reversal is needed
    TIM4->SMCR |= 0x03;     // Set encoder mode.
    TIM4->CNT = 0;          // Clear counter before start.
    TIM4->CR1 |= TIM_CR1_CEN;
}

void setup_motor1( void )
{

}

void setup_motor2( void )
{

}

inline int16_t get_encoder1_value( void )
{
    return TIM3->CNT;
}

inline int16_t get_encoder2_value( void )
{
    return TIM4->CNT;
}
