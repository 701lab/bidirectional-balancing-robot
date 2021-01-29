/*!
 * @file itf_gpio.c
 *
 * Robots board v2.0 uses STM32G474RBT6 MCU.
 *  PCB contains 3 user LEDs.
 *
 * LEDs:
 *  PB0 - D4 LED (digital output);
 *  PB1 - D3 LED (digital output);
 *  PB2 - D2 LED (digital output).
 */

#include "itf_gpio.h"

void setup_leds( void )
{
    // Enable clocking of GPIOB.
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;

    // Reset needed pin modes.
    GPIOB->MODER &= ~(GPIO_MODER_Msk << GPIO_MODER_MODE0_Pos
                | GPIO_MODER_Msk << GPIO_MODER_MODE1_Pos
                | GPIO_MODER_Msk << GPIO_MODER_MODE2_Pos);

    // Set desired modes.
    GPIOB->MODER |= (GPIO_DIGITAL_OUT_Mode << GPIO_MODER_MODE0_Pos
                | GPIO_DIGITAL_OUT_Mode << GPIO_MODER_MODE1_Pos
                | GPIO_DIGITAL_OUT_Mode << GPIO_MODER_MODE2_Pos);
}

inline void toggle_d2_led( void )
{
    GPIOB->ODR ^= GPIO_ODR_OD2;
}

inline void turn_on_d2_led( void )
{
    GPIOB->BSRR = GPIO_BSRR_BS2;
}

inline void turn_off_d2_led( void )
{
    GPIOB->BSRR = GPIO_BSRR_BR2;
}

inline void toggle_d3_led( void )
{
    GPIOB->ODR ^= GPIO_ODR_OD1;
}

inline void turn_on_d3_led( void )
{

    GPIOB->BSRR = GPIO_BSRR_BS1;
}

inline void turn_off_d3_led( void )
{
    GPIOB->BSRR = GPIO_BSRR_BR1;
}


inline void toggle_d4_led( void )
{
    GPIOB->ODR ^= GPIO_ODR_OD0;
}

inline void turn_on_d4_led( void )
{
    GPIOB->BSRR = GPIO_BSRR_BS0;
}

inline void turn_off_d4_led( void )
{
    GPIOB->BSRR = GPIO_BSRR_BR0;
}

inline void toggle_all_leds( void )
{
    GPIOB->ODR ^= GPIO_ODR_OD0 | GPIO_ODR_OD1 | GPIO_ODR_OD2;
}

inline void turn_off_all_leds( void )
{
    GPIOB->BSRR = GPIO_BSRR_BR0 | GPIO_BSRR_BR1 | GPIO_BSRR_BR2;
}

inline void turn_on_all_leds( void )
{
    GPIOB->BSRR = GPIO_BSRR_BS0 | GPIO_BSRR_BS1 | GPIO_BSRR_BS2;
}

