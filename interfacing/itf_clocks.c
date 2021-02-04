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


/****************************************************************************************/
/*                                                                                      */
/*                                   Clocking setup                                     */
/*                                                                                      */
/****************************************************************************************/

/*!
 * @brief Sets up SYSCLK to SYSTEM_MAIN_FREQUENCY.
 *
 * Tries to archive SYSTEM_MAIN_FREQUENCY with any possible way: first with HSE as a PLL source.
 *      if it not working - Oscillator absent or or can't start, starts PLL with HSI as a source.
 *      If this also fails keep system on the HSI. Add all possible mistakes to the log.
 *
 * @todo Update function to make a safe < 80 MHz startup.
 */
void setup_system_clock(void)
{

/*! Depending on the used SYSTEM_MAIN_FREQUENCY we need to change flash access latency. See RM chapter 3.3.3. */
#if (SYSTEM_MAIN_FREQUENCY > 120000000)
    FLASH->ACR &= ~FLASH_ACR_LATENCY_Msk;   //!< Clear latency setup.
    FLASH->ACR |= FLASH_ACR_LATENCY_4WS;    //!< 0x04 - 5 CPU cycles read latency (4 + 1).

#elif (SYSTEM_MAIN_FREQUENCY > 90000000)
    FLASH->ACR &= ~FLASH_ACR_LATENCY_Msk;   //!< Clear latency setup.
    FLASH->ACR |= FLASH_ACR_LATENCY_3WS;    //!< 0x03 - 4 CPU cycles read latency (3 + 1).

#elif (SYSTEM_MAIN_FREQUENCY > 60000000)
    FLASH->ACR &= ~FLASH_ACR_LATENCY_Msk;   //!< Clear latency setup.
    FLASH->ACR |= FLASH_ACR_LATENCY_2WS;    //!< 0x02 - 3 CPU cycles read latency (2 + 1).

#elif (SYSTEM_MAIN_FREQUENCY > 30000000)
    FLASH->ACR &= ~FLASH_ACR_LATENCY_Msk;   //!< Clear latency setup.
    FLASH->ACR |= FLASH_ACR_LATENCY_1WS;    //!< 0x01 - 2 CPU cycles read latency (1 + 1).

#else
    FLASH->ACR &= ~FLASH_ACR_LATENCY_Msk;   // Clear latency setup = 1 CPU cycle read latency
#endif

    // Start external oscillator.
    RCC->CR |= RCC_CR_HSEON;

    // Give HSE time to start up.
    for ( uint32_t i = 0; i < DUMMY_DELAY_VALUE; ++i ) // @todo fix delay to a flexible value that should be constant here, because at this point it works on fixed 16MHz HSI.
    {
        uint32_t hse_is_started = (RCC->CR & RCC_CR_HSERDY) == RCC_CR_HSERDY;
        if (hse_is_started)
        {
            // Enable clock security system. It will call the non-maskable interrupt if HSE fails.
            RCC->CR |= RCC_CR_CSSON;

            // Start PLL with HSE as a frequency source.
            uint32_t pll_startup_failed = setup_pll_as_sysclk_source(HSE_AS_PLL_SOURCE);
            if ( pll_startup_failed )
            {
                // HSE is working, but PLL is not - weird stuff. HSI will remain as a clock source.
                // Stop HSE so it won't consume power.
                RCC->CR &= ~RCC_CR_HSEON;

                // Reset FLASH latency because HSI is 16 MHz.
                FLASH->ACR &= ~FLASH_ACR_LATENCY_Msk;
            }
            else
            {
                /*
                 * This block should not be used in out setup because we use HSI as a clock source for the TMC5130. And i don't think we need to change it
                 * to have a guaranteed frequency on the TMC5130 even at the fault event on the oscillator.
                 */
                // Everything is fine. HSI will be stopped so it won't consume energy.
                // RCC->CR &= ~RCC_CR_HSION_Msk;
                return;
            }
        }
    }

    // HSE didn't start. Stop it so it won't consume power.
    RCC->CR &= ~RCC_CR_HSEON;
    add_mistake_to_the_log(MCU_FAILED_TO_LAUNCH_HSE_Err);

    // Try starting PLL with a HSI16 as a source
    setup_pll_as_sysclk_source(HSI16_AS_PLL_SOURCE);
}


/*!
 * @brief Sets up PLL with a given input source.
 *
 * The functions set up PLL with respect to a given source and with strictly defined dividers and multipliers coefficients.
 *
 */
 uint32_t setup_pll_as_sysclk_source(mcu_pll_source frequency_source)
{
    // Reset PLL register before setup.
    RCC->PLLCFGR = 0x00;

    // Set up PLL with respect to given frequency source
    if ( frequency_source == HSE_AS_PLL_SOURCE )
    {
        RCC->PLLCFGR |= PLLP_VALUE << RCC_PLLCFGR_PLLPDIV_Pos
                        | PLLR_VALUE << RCC_PLLCFGR_PLLR_Pos
                        | RCC_PLLCFGR_PLLREN
                        | PLLQ_VALUE << RCC_PLLCFGR_PLLQ_Pos
                        | PLLN_VALUE << RCC_PLLCFGR_PLLN_Pos
                        | PLLM_VALUE << RCC_PLLCFGR_PLLM_Pos
                        | RCC_PLLCFGR_PLLSRC_HSE;

        // @todo Add check that HSE was enabled and mistake output if not.

    }
    else if ( frequency_source == HSI16_AS_PLL_SOURCE )
    {
        RCC->PLLCFGR |= PLLP_VALUE << RCC_PLLCFGR_PLLPDIV_Pos
                        | PLLR_VALUE << RCC_PLLCFGR_PLLR_Pos
                        | RCC_PLLCFGR_PLLREN
                        | PLLQ_VALUE << RCC_PLLCFGR_PLLQ_Pos
                        | PLLN_VALUE << RCC_PLLCFGR_PLLN_Pos
                        | PLLM_VALUE_WITH_HSI << RCC_PLLCFGR_PLLM_Pos
                        | RCC_PLLCFGR_PLLSRC_HSI;

        // @todo Add check that HSI16 was enabled and mistake output if not.
    }
    else
    {
        add_mistake_to_the_log( MCU_PLL_WRONG_FREQUENCY_SOURCE_Err );
        return MCU_PLL_WRONG_FREQUENCY_SOURCE_Err;
    }

    // Start PLL.
    RCC->CR |= RCC_CR_PLLON;

    uint32_t safety_delay_counter = 0;

    // Give PLL time to start up.
    while ( ( RCC->CR & RCC_CR_PLLRDY ) != RCC_CR_PLLRDY ) // @todo This safety delay implemented differently than the system clock one, which should be fixed.
    {
        ++safety_delay_counter;
        if ( safety_delay_counter > DUMMY_DELAY_VALUE )
        {
            // Stop PLL so it won't consume power.
            RCC->CR &= ~RCC_CR_PLLON;
            add_mistake_to_the_log( MCU_PLL_FAILED_Err );
            return MCU_PLL_FAILED_Err;
        }
    }

    // At that point PLL is on and we can use it as a SYSCLK source.
    RCC->CFGR &= ~RCC_CFGR_SW_Msk;
    RCC->CFGR |= RCC_CFGR_SW_PLL;

    // Wait for System clock source swap.
    while ( ( RCC->CFGR & RCC_CFGR_SWS_PLL ) != RCC_CFGR_SWS_PLL );

    return 0;
}

/*!
 *
 * @brief Handles HSE fail interrupt.
 *
 * Non-maskable interrupt handler. Called when HSE fails. Tries to start PLL with HSI as a source
 *
 * When HSE fail is detected non-maskable interrupt is called if enabled in RCC_CR register.
 *      If fail happens, systems swap to the HSI automatically, even if it is disabled. So the
 *     function tries to start PLL with HSI as a source instead of HSE which is meant to be unfunctional.
 *
 * @todo It is possible, that system re-setup will be needed if clock source was lost. Should add such functionality in the future.
 * @todo Possibly system_clock_setup can be used instead of straight PLL setup to try to enable HSE one more time.
 */
void NMI_Handler()
{
    // Clear the clock security system interrupt flag.
    RCC->CICR |= RCC_CICR_CSSC;

    add_mistake_to_the_log(MCU_HSE_FAILED_WHILE_RUNNING_Err);

    // Wait until PLL is fully stopped.
    while ( (RCC->CR & RCC_CR_PLLRDY) == RCC_CR_PLLRDY ){}

    // Try starting PLL with HSI as a source.
    add_mistake_to_the_log(setup_pll_as_sysclk_source(HSI16_AS_PLL_SOURCE));

    //! @note Instead of using HSI as a clock source for PLL, it is possible to call system_clock_setup function
    //!     that will try to re-setup everything - HSE included, and if it won't start, it will just use HSI as PLL
    //!     source (the same result, but the possibility to start HSE once again. can be dangerous if HSE not dead, but lagging).
    //!     In the future there can be implemented a procedure, that will try to enable HSE back again a couple of times,
    //!     before completely  give up, cause HSE source is way better for real-time control systems.
//    system_clock_setup();
}

