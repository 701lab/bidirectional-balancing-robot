/*!
 * @file itf_timers.c
 */

#include "itf_timers.h"

/*!
 * TIM14 - counts time with 1 ms precision for mistakes log
 *
 * TIM16 - timer for proper delay implementation
 */

/*!
 * @brief Set up timer 6 as milliseconds counter.
 *
 * The timer will count milliseсonds. Resets every second and call interrupt, that updates seconds counter, represented by a global variable.
 *
 * @todo Add DMA update of the seconds value not to call interrupt every time.
 */
void setup_timers( void )
{
    // Enable timers clocking.
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM6EN;

    // Set up prescaler and reload to count exactly 1000 pulses every second and overflow every second.
    TIM6->PSC |= (uint32_t)((SYSTEM_MAIN_FREQUENCY / 1000) - 1);
    TIM6->ARR = 1000 - 1;

    //Enable interrupt on update event.
    TIM6->DIER |= TIM_DIER_UIE;
    NVIC_SetPriority( TIM6_DAC_IRQn, 2 ); // Interrupt should not be too important.
    NVIC_EnableIRQ( TIM6_DAC_IRQn );

    // Generate update event to enable new register values and start timer.
    TIM6->EGR |= TIM_EGR_UG;
    TIM6->CR1 |= TIM_CR1_CEN;
}

/*!
 * @brief Updates seconds value on timer 15 update event interrupt.
 */
void TIM6_DACUNDER_IRQHandler()
{
    // Clear update event interrupt flag.
    TIM6->SR &= ~TIM_SR_UIF;

    // Update minutes value.
    seconds_from_setup += 1;
}

/*!
 * @brief Sets up SysTick timer interrupt with respect to the main system frequency.
 *
 * @note SysTick timer uses the default prescaler of 8 because it is a 24-bit timer that will reload on relatively
 *          small frequencies (high prescaler values). This can help to not overflow the counter on small
 *          SysTick frequencies and high system frequencies.
 */
void setup_system_timer( void )
{
    // Set system timer reload register.
    SysTick->LOAD = SYSTEM_MAIN_FREQUENCY / (8 * SYSTICK_INTERRUPT_FREQUENCY) - 1;

    // Reset counter value to 0 before start.
    SysTick->VAL = 0;

    // Enable interrupt.
    NVIC_EnableIRQ( SysTick_IRQn );

    // Start SysTick timer with default prescaler of 8 and enable interrupt.
    SysTick->CTRL |= 0x03;
}

