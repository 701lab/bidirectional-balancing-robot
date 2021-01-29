#define DECLARE_GLOBAL_VARIABLES

#include "interfacing.h"

void dummy_delay(uint32_t duration);

int main(void)
{

    // Включить HSE как и в прошлом примере
    RCC->CR |= RCC_CR_HSEON;
    while ( (RCC->CR & RCC_CR_HSERDY) != RCC_CR_HSERDY ) {}

    // Использовать PLL в качестве источника SYSCLK.
    RCC->CFGR |= RCC_CFGR_SW_Msk;
    RCC->CFGR &= ~(RCC_CFGR_SW_Msk ^ RCC_CFGR_SW_HSE);

    // Подождать, до момента, пока источник частоты помеяется
    while ( ( RCC->CFGR & RCC_CFGR_SWS_HSE) != RCC_CFGR_SWS_HSE ){}

    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM3EN;

    GPIOB->MODER &= ~( GPIO_MODER_MODE0_Msk | GPIO_MODER_MODE1_Msk );
    GPIOB->MODER |= ( 2 << GPIO_MODER_MODE0_Pos | 2 << GPIO_MODER_MODE1_Pos );
    GPIOB->AFR[0] |= 2 << GPIO_AFRL_AFSEL0_Pos | 2 << GPIO_AFRL_AFSEL1_Pos;

    // Настройка таймера на выход
    TIM3->PSC = 0;       // Преддилитель = 0, то есть каждый такт на шине тактирует таймер напрямую
    TIM3->ARR = 399;    // Частота = 20000 Гц, поэтому максимальная возможная точность - 400
    // Настроить таймер в режим ШИМ
    TIM3->CCMR2 |=  TIM_CCMR2_OC3PE | 6 << TIM_CCMR2_OC3M_Pos | TIM_CCMR2_OC4PE | 6 << TIM_CCMR2_OC4M_Pos;
    TIM3->CCER |= TIM_CCER_CC3E | TIM_CCER_CC4E; // включить канал 3 и 4.
    TIM3->CR1 |= TIM_CR1_ARPE;  // Разрешить автоматическую перегрузку счетчика
    TIM3->CR1 |= TIM_CR1_CEN; // Включить таймер
    TIM3->CCR3 = 250; // Установить желаемы коэффициент заполнения на канал 3 - 0.625
    TIM3->CCR4 = 100; // Установить желаемы коэффициент заполнения на канал 4 - 0.25

    while(1)
    {
    }
}

void dummy_delay(uint32_t duration)
{
    for(uint32_t i = 0; i < duration; ++i){}
}

void SysTick_Handler()
{
    GPIOB->ODR ^= 0x07;
}
