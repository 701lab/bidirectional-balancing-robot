#define DECLARE_GLOBAL_VARIABLES

#include "interfacing.h"

void dummy_delay(uint32_t duration);

int main(void)
{

    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;

    GPIOB->MODER &= ~(GPIO_MODER_MODE0_Msk | GPIO_MODER_MODE1_Msk | GPIO_MODER_MODE2_Msk);

    // Set desired pin-modes.
    GPIOB->MODER |= (1 << GPIO_MODER_MODE0_Pos
                | 1 << GPIO_MODER_MODE1_Pos
                | 1 << GPIO_MODER_MODE2_Pos);

    while(1)
    {
        GPIOB->ODR ^= 0x07;
        dummy_delay(1000000);
    }
}

void dummy_delay(uint32_t duration)
{
    for(uint32_t i = 0; i < duration; ++i){}
}

