/*!
 * @file interfacing.c
 */

#include "interfacing.h"

/****************************************************************************************/
/*                                                                                      */
/*                             MCU connections overview                                 */
/*                                                                                      */
/****************************************************************************************/


// @todo Clear interfacing library.
/*!
 *  LEDs:
 *      PA4, PA5 (digital outputs).
 *
 *  TMC5130 + AEAT-8800:
 *      PB3-PB5 - SCK, MISO, MOSI respectively (SPI3, alternate function 6);
 *      PB6 - TMC_EN (digital output) - TMC5130 enable pin;
 *      PA8 - ENC_CS (digital output) - AEAT-8800 SPI chip select;
 *      PA15 - TMC_CS (digital output) - TMC5130 SPI chip select;
 *      PA0, PA1 - MCU_ENC_A/_B (alternate function 1) - TIM2 incremental encoder inputs;
 *      PB9 - MCU_ENC_N (digital input) - incremental encoder N-channel;
 *      PC14, PC15 - DIAG1/0 (digital inputs with interrupt);
 *      PG10 - TMC_CLK (alternate function 0) - Clock data output for the TMC5130.
 *
 *  SPI2:
 *      PB13-PB15 - SCK, MISO, MOSI respectively (SPI2, alternate function 5);
 *      PB12 - CS_1 (digital output) - chip select;
 *      PC6 - CS_2 (digital output) - chip select.
 *
 *  I2C1:
 *      PB7, PB8 - SDA and SCL respectively (I2C1, alternate function 4).
 *
 *  UART1:
 *      PA9, PA10 - TX and RX respectively (USART1, alternate function 7).
 *
 *  UART3:
 *      PC10, PC11 - TX and RX respectively (USART3, alternate function 7).
 *
 *  ADC:
 *      PA2 - ADC1_IN3 (analog) - fast ADC input;
 *      PA3 - ADC1_IN4 (analog) - fast ADC input;
 *      PA6 - ADC2_IN3 (analog) - fast ADC input;
 *      PA7 - ADC2_IN4 (analog) - fast ADC input;
 *      PC4 - ADC2_IN5 (analog) - fast ADC input.
 *
 * Timers:
 *      TIM2 - AEAT-8800 incremental encoder counter;
 *      TIM15 - mistakes log clock timer.
 */

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

/*!
 * @brief Sets up PG10 as MCO pin from HSI16 source.
 *
 * @param frequency_source - desired frequency source for the MCO output. See rcc_mco_source for details.
 *
 * Function checks if PG10 is used as an NRST pin. If so changes the mode to GPIO and resets MCU.
 *
 * @todo This function can be further updated to setup both PG10 and PA8 and to choose MCO divider either automatically or manually.
 *          There can be a note, that will say exact MCO output frequency during compiling.
 */
void setup_master_clock_output (mcu_rcc_mco_source frequency_source)
{
    // Check if the PG10 pin is not in GPIO mode.
    if((FLASH->OPTR & FLASH_OPTR_NRST_MODE_Msk) != (NRST_GPIO_MODE << FLASH_OPTR_NRST_MODE_Pos))
    {
        // Set NRST to GPIO mode.
        set_nrst_pin_mode(NRST_GPIO_MODE);
    }

    // Enable GPIOG clocking.
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOGEN;

    // Configure PG10 as an alternate pin. Don't set up an alternate function cause we need 0 mode, which is the default.
    GPIOG->MODER &= ~GPIO_MODER_MODE10_Msk;
    GPIOG->MODER |= GPIO_ALTERNATE_Mode << GPIO_MODER_MODE10_Pos;

    // Set higher pin speed.
    GPIOG->OSPEEDR |= GPIO_OSPEED_LOW << GPIO_MODER_MODE10_Pos;

    // Set up MCO.
    RCC->CFGR |= frequency_source << RCC_CFGR_MCOSEL_Pos;
}


/****************************************************************************************/
/*                                                                                      */
/*                                    Debugging                                         */
/*                                                                                      */
/****************************************************************************************/

/*!
 * @brief Initializes all debug related stuff.
 *
 * If allowed, enables timers for mistake time logging.
 *
 * @todo Change function to enable re-setup. So that it can be used right from the device enable and then reconfigured after the clock is set up.
 */
uint32_t setup_debug_features(void)
{
    /*!
     * @brief Timer 15 setup as seconds counter clock
     *
     * Timer will count milliseсonds. Resets every second, so counts 60 seconds * 1000 milliseconds = 60000 before reset.
     *
     * @todo Add DMA update of the seconds value not to call interrupt every time.
     */
    // Enable timer 15 clocling.
    RCC->APB2ENR |= RCC_APB2ENR_TIM15EN;

    // Set up prescaler and reload registers.
    TIM15->PSC |= (uint32_t)((SYSTEM_MAIN_FREQUENCY / 1000) - 1);
    TIM15->ARR = 1000 - 1;

    //Enable interrupt on update event.
    TIM15->DIER |= TIM_DIER_UIE;
    NVIC_EnableIRQ(TIM1_BRK_TIM15_IRQn);

    // Generate update event to enable new rigister values and start timer.
    TIM15->EGR |= TIM_EGR_UG;
    TIM15->CR1 |= TIM_CR1_CEN;

    return 0;
}

/*!
 * @brief Updates seconds value on timer 15 update event interrupt.
 */
void TIM1_BRK_TIM15_IRQHandler()
{
    // Clear update event interrupt flag.
    TIM15->SR &= ~TIM_SR_UIF;

    // Update minutes value.
    seconds_from_setup += 1;
}


/*!
 * @brief Makes misteakes_log entry with given mistake code at the current time if time logging is enabled.
 *
 * @param mistake_code - code of the occurred mistake to add to the log.
 *
 * @return The mistake_code value.
 *
 * @todo Implement critical mistake handling.
 * @todo Add timers setup functon to be able to get time stamp for the mistakes even before general setup.
 *          And after setup the function should be called again to resetup to a current frequency.
 */
uint32_t add_mistake_to_the_log(uint32_t mistake_code)
{
    // Return zero if no mistake.
    if(mistake_code == 0)
    {
        return 0;
    }


    // Reset pointer if overflow.
    if (mistakes_log_pointer == MISTAKES_LOG_SIZE)
    {
        mistakes_log_pointer = 0;

        // Add log overflow mistake to the log.
        mistakes_log[MISTAKES_LOG_SIZE].mistake_code = MCU_ERROR_LOG_OVERFLOW_Err;

        // Add logging time to the log if enabled.
#ifdef MISTAKE_LOG_SHOULD_SAVE_TIME

        mistakes_log[MISTAKES_LOG_SIZE].mistake_time_in_seconds = seconds_from_setup;
        mistakes_log[MISTAKES_LOG_SIZE].mistake_time_in_milliseconds = TIM15->CNT;

#endif /* MISTAKE_LOG_SHOULD_SAVE_TIME */

    }

    // Add mistake to the log.
    mistakes_log[mistakes_log_pointer].mistake_code = mistake_code;

    // Add logging time to the log if enabled.
#ifdef MISTAKE_LOG_SHOULD_SAVE_TIME

    mistakes_log[mistakes_log_pointer].mistake_time_in_seconds = seconds_from_setup;
    mistakes_log[mistakes_log_pointer].mistake_time_in_milliseconds = TIM15->CNT;

#endif /* MISTAKE_LOG_SHOULD_SAVE_TIME */

    ++mistakes_log_pointer;


    // Return mistake code for further use.
    return mistake_code;
}


/****************************************************************************************/
/*                                                                                      */
/*                              General setup functions                                 */
/*                                                                                      */
/****************************************************************************************/
/*!
 * @note for future. Inline functions will not be allowed into any other setup field than general
 *          due to the flexible structure of interfacing independent libraries.
 */

/*!
 * @brief Sets up all needed FLASH option bytes.
 *
 * If needed, disables the boot0 pin, so that it can't be accessed from the outside and PB8 can be used as GPIO.
 *      Also, sets NRST pin to GPIO mode for MCO generation.
 * in the future).
 *
 * @todo update the function to a final state.
 */
void setup_flash_option_bytes()
{
    // Check nSWBOOT0 bit state to know if an update is needed.
   if((FLASH->OPTR & FLASH_OPTR_nSWBOOT0_Msk))
   {
       // Wait until flash is available to access.
       while(FLASH->SR & FLASH_SR_BSY){}

       // Unlock flash.
       FLASH->KEYR = 0x45670123;
       FLASH->KEYR = 0xCDEF89AB;

       // Unlock option bytes.
       FLASH->OPTKEYR = 0x08192A3B;
       FLASH->OPTKEYR = 0x4C5D6E7F;

       // Reset nSWBOOT0 bit and NRST mode.
       FLASH->OPTR &= ~(FLASH_OPTR_nSWBOOT0_Msk | FLASH_OPTR_NRST_MODE_Msk);

       // Set NRST pin mode to GPIO.
       FLASH->OPTR |= NRST_GPIO_MODE << FLASH_OPTR_NRST_MODE_Pos;

       // Enable updates.
       while(FLASH->SR & FLASH_SR_BSY){}
       FLASH->CR |= FLASH_CR_OPTSTRT;

       // Reset MCU to load updated option bytes.
       while(FLASH->SR & FLASH_SR_BSY){}
       FLASH->CR |= FLASH_CR_OBL_LAUNCH;
   }
}

/*!
 * @brief Sets up the reset pin mode.
 */
void set_nrst_pin_mode(mcu_nrst_mode desired_mode)
{
    // Check if the PG10 pin is in NRST input/output mode
    if((FLASH->OPTR & FLASH_OPTR_NRST_MODE_Msk) != (desired_mode << FLASH_OPTR_NRST_MODE_Pos))
    {
        // Wait until flash is available to access.
        while(FLASH->SR & FLASH_SR_BSY){}

        // Unlock flash.
        FLASH->KEYR = 0x45670123;
        FLASH->KEYR = 0xCDEF89AB;

        // Unlock option bytes.
        FLASH->OPTKEYR = 0x08192A3B;
        FLASH->OPTKEYR = 0x4C5D6E7F;

        // Change NRST mode.
        FLASH->OPTR &= ~FLASH_OPTR_NRST_MODE_Msk;
        FLASH->OPTR |= desired_mode << FLASH_OPTR_NRST_MODE_Pos;

        // Enable updates.
        while(FLASH->SR & FLASH_SR_BSY){}
        FLASH->CR |= FLASH_CR_OPTSTRT;

        // Reset MCU to load updated option bytes.
        while(FLASH->SR & FLASH_SR_BSY){}
        FLASH->CR |= FLASH_CR_OBL_LAUNCH;
    }
}

/*!
 * @brief Sets up LEDs.
 */
void setup_leds(void)
{
    // Enable clocking of GPIOA.
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;

    // Reset needed pin modes/
    GPIOA->MODER &= ~(GPIO_MODER_Msk << GPIO_MODER_MODE4_Pos
                | GPIO_MODER_Msk << GPIO_MODER_MODE5_Pos);

    // Set desired modes.
    GPIOA->MODER |= (GPIO_DIGITAL_OUT_Mode << GPIO_MODER_MODE4_Pos
                | GPIO_DIGITAL_OUT_Mode << GPIO_MODER_MODE5_Pos);
}

/*!
 * @brief Toggles pa4 pin high - lights up the pa4 led.
 */
inline void light_up_blue_led(void)
{
    GPIOA->BSRR = GPIO_BSRR_BS4;
}

/*!
 * @brief Toggles pa4 pin low - puts down the pa4 led.
 */
inline void put_down_blue_led(void)
{
    GPIOA->BSRR = GPIO_BSRR_BR4;
}

/*!
 * @brief Toggles pa4 pin. Toggles led state
 */
inline void toggle_blue_led(void)
{
    GPIOA->ODR ^= GPIO_ODR_OD4;
}

/*!
 * @brief Toggles pa5 pin high - lights up the led.
 */
inline void light_up_red_led(void)
{
    GPIOA->BSRR = GPIO_BSRR_BS5;
}

/*!
 * @brief Toggles pa5 pin low - puts down the pa5 led.
 */
inline void put_down_red_led(void)
{
    GPIOA->BSRR = GPIO_BSRR_BR5;
}

/*!
 * @brief Toggles pa4 pin. Toggles led state
 */
inline void toggle_red_led(void)
{
    GPIOA->ODR ^= GPIO_ODR_OD5;
}

/*!
 * @brief Sets up SysTick timer and interrupt.
 *
 * @note SysTick timer uses the default prescaler of 8 because it is a 24-bit timer that will reload on relatively
 *          small frequencies (high prescaler values). This can help to not overflow the counter on small
 *          SysTick frequencies and high system frequencies.
 */
void setup_system_timer(void)
{
    // Enable System timer for the control loop interrupt without using STM timers.
    // Set up prescaler with respect to given system frequency and SYSTICK interrupt frequency.
    SysTick->LOAD = SYSTEM_MAIN_FREQUENCY / (8 * SYSTICK_INTERRUPT_FREQUENCY) - 1;

    // Reset value to 0.
    SysTick->VAL = 0;

    // Enable interrupt.
    NVIC_EnableIRQ(SysTick_IRQn);

    // Start SysTick timer with default prescaler of 8 and enable interrupt.
    SysTick->CTRL |= 0x03;
}

/*!
 * @brief Sets up only features related to the main board functionality.
 */
void setup_base_peripherals(void)
{
    // Enable FPU.
    SCB->CPACR |= 0x3 << 20 ;

    /*! Flash setup should always run first, cause it can call full reset, thus the sooner it's called the lower the initialization delay. */
    setup_flash_option_bytes();

    /*! Every single time related parameter is based on SYSTEM_MAIN_FREQUENCY, thus clock be set up a soon as possible. */
    setup_system_clock();

    setup_debug_features();

    setup_leds();
    setup_tmc5130_aeat8800_peripherals();

    setup_master_clock_output(SYSCLK_AS_MCO_SOURCE);

    /*
     * System timer should be set up last, so that control loop won't start working before MCU setup is finished.
     *
     * @todo In future this setup should be called after every other device (TMC5130 and encoder) were set up.
     *        Because otherwise it is possible that control loop will ask for data before setup was finished.
     */
    setup_system_timer();

    // @todo Add some kind of control loop zeroing.
    // @todo If possible call first SysTick interrupt immediately to enable everything.

}

/*!
 * @brief Sets up all board's features.
 */
void setup_all_peripherals(void)
{
    // Enable full FPU.
    SCB->CPACR |= 0x3 << 20 ;

    setup_base_peripherals();

    setup_uart1(115200);

    setup_spi2();
    // @todo setup of spi2, uart1, uart3, adc, and all related GPIO.

}

/*!
 * @brief Sets SPI2 with a frequency stated in SPI2_DESIRED_FREQUENCY definition.
 */
void setup_spi2()
{
    // Enable SPI clocking.
    RCC->APB1ENR1 |= RCC_APB1ENR1_SPI2EN;

    // Baud rate prescaler choosing algorithm.
    uint32_t baud_rate_divider = 2;
    uint32_t baud_rate = 0;

    for ( int i = 0; i < 8; ++i )
    {
        if ((SYSTEM_MAIN_FREQUENCY / baud_rate_divider) < SPI2_DESIRED_FREQUENCY)
        {
            break;
        }
        ++baud_rate;
        baud_rate_divider *= 2;
    }

    // If achieved frequency is still higher than the desired one.
    if ((SYSTEM_MAIN_FREQUENCY / baud_rate_divider) > SPI2_DESIRED_FREQUENCY)
    {
        add_mistake_to_the_log(MCU_SPI2_BAUD_RATE_SETUP_FAILED_Err);
    }

    // setup SPI as master with disabled hardware CS.
    SPI2->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI | (baud_rate << SPI_CR1_BR_Pos) | SPI_CR1_MSTR;
    SPI2->CR2 |= SPI_CR2_FRXTH;
    SPI2->CR1 |= SPI_CR1_SPE;
}

/*!
 * @brief Transmits and receives a single byte via SPI2.
 *
 * @param byte_to_be_sent - byte that will be sent via SPI.
 *
 * @return Byte received during transmission.
 *
 * @todo Update function body.
 * @todo Make the transmission fail counter dependent on the system frequency.
 */
uint8_t spi2_write_single_byte(const uint8_t byte_to_send)
{
    uint32_t safety_delay_counter = 0;

    // Wait until transmission buffer is empty.
    while((SPI2->SR & SPI_SR_TXE) != SPI_SR_TXE)
    {
        ++safety_delay_counter;

        // If transmission buffer is still not empty after delay.
        if ( safety_delay_counter > DUMMY_DELAY_VALUE )
        {
            add_mistake_to_the_log(MCU_SPI2_TRANSMISSION_FAILED_Err);
            return 0;
        }
    }

    // Write single byte into the Data Register with single byte access to send it.
    *((volatile uint8_t *)&SPI2->DR) = byte_to_send;

    // Wait until answer will appear in RX buffer.
    while(((SPI2->SR & SPI_SR_RXNE) != SPI_SR_RXNE)){}

    // Return value from RX buffer.
    return (SPI2->DR);
}



/****************************************************************************************/
/*                                                                                      */
/*                                 TMC5130 + AEAT-8800                                  */
/*                                                                                      */
/****************************************************************************************/

/*!
 * @brief Sets SPI3 with given frequency.
 *
 * In STM32 SPI can use SYSCLK frequency divided by 2 to the power from 1 to 8: 2, 4, 8 ... 256.
 *      Thus we need to set up SPI with the biggest possible frequency lower than the desired one.
 *      To do this, the algorithm iteratively checks whether the current prescaler matches the given criterion.
 *      If not, the prescaler is increased until the desired result is obtained.
 */
void setup_spi3()
{
    // Enable SPI clocking.
    RCC->APB1ENR1 |= RCC_APB1ENR1_SPI3EN;

    // Baud rate prescaler choosing algorithm.
    uint32_t baud_rate_divider = 2;
    uint32_t baud_rate = 0;

    for ( int i = 0; i < 8; ++i )
    {
        if ((SYSTEM_MAIN_FREQUENCY / baud_rate_divider) <= SPI3_DESIRED_FREQUENCY)
        {
            break;
        }
        ++baud_rate;
        baud_rate_divider *= 2;
    }

    // If achieved frequency is still higher than the desired one.
    if ((SYSTEM_MAIN_FREQUENCY / baud_rate_divider) > SPI3_DESIRED_FREQUENCY)
    {
        add_mistake_to_the_log(MCU_SPI3_BAUD_RATE_SETUP_FAILED_Err);
    }

    // setup SPI3 as master with disabled hardware CS.
    SPI3->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI | (baud_rate << SPI_CR1_BR_Pos) | SPI_CR1_MSTR;
    SPI3->CR2 |= SPI_CR2_FRXTH;
    SPI3->CR1 |= SPI_CR1_SPE;
}


/*!
 * @brief Transmits and receives a single byte via SPI3.
 *
 * @param byte_to_be_sent - byte that will be sent via SPI3.
 *
 * @return Byte received during transmission.
 *
 * @todo Update function body.
 * @todo Make the transmission fail counter dependent on the system frequency.
 */
uint8_t spi3_write_single_byte(const uint8_t byte_to_send)
{
    uint32_t safety_delay_counter = 0;

    // Wait until transmission buffer is empty.
    while((SPI3->SR & SPI_SR_TXE) != SPI_SR_TXE)
    {
        ++safety_delay_counter;

        // If transmission buffer is still not empty after delay.
        if ( safety_delay_counter > DUMMY_DELAY_VALUE )
        {
            add_mistake_to_the_log(MCU_SPI3_TRANSMISSION_FAILED_Err);
            return 0;
        }
    }

    // Write single byte into the Data Register with single byte access to send it.
    *((volatile uint8_t *)&SPI3->DR) = byte_to_send;

    // Wait until answer will appear in RX buffer.
    while(((SPI3->SR & SPI_SR_RXNE) != SPI_SR_RXNE)){}

    // Return value from RX buffer.
    return (SPI3->DR);
}


//! This function is needed in particular for AEAT8800.
//! @todo Deside how we will use AEAT8800 in future versions. Update this function.
void spi3_set_mode(uint8_t mode)
{
    switch (mode) {
        case 0:
            {
                SPI3->CR1 &= ~(SPI_CR1_CPHA | SPI_CR1_CPOL);
                return;
            }
        case 1:
            {
                SPI3->CR1 &= ~(SPI_CR1_CPHA | SPI_CR1_CPOL);
                SPI3->CR1 |= (SPI_CR1_CPHA);
                return;
            }
        case 2:
        {
            SPI3->CR1 &= ~(SPI_CR1_CPHA | SPI_CR1_CPOL);
            SPI3->CR1 |= (SPI_CR1_CPOL);
            return;
        }
        case 3:
        {
            SPI3->CR1 |= (SPI_CR1_CPHA | SPI_CR1_CPOL);
            return;
        }

            break;
        default:
            return; // Here should be the mistake code of wrong mode input.
    }
}


/*!
 * @brief Sets up all needed peripherals for TMC5130 and AEAT-8800.
 *
 * TMC5130 and AEAT-8800 use:
 *      PA0, PA1 - TIM2 incremental encoder inputs;
 *      PA8 - AEAT-8800 chip select;
 *      PA15 - TMC5130 chip select;
 *      PB3-PB5 - SCK, MISO, MOSI respectively (SPI3);
 *      PB6 - TMC5130 enable;
 *      PB9 - AEAT-8800 N-channel input;
 *      PC14 - TMC5130 DIAG1 input;
 *      PC15 - TMC5130 DIAG0 input;
 */
void setup_tmc5130_aeat8800_peripherals()
{
    // Enable clocking of GPIOA, GPIOB and GPIOC.
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_GPIOCEN;

    /*! GPIOA setup */
    // Reset pin-modes.
    GPIOA->MODER &= ~(GPIO_MODER_Msk << GPIO_MODER_MODE0_Pos
                | GPIO_MODER_Msk << GPIO_MODER_MODE1_Pos
                | GPIO_MODER_Msk << GPIO_MODER_MODE8_Pos
                | GPIO_MODER_Msk << GPIO_MODER_MODE15_Pos);

    // Set desired pin-modes.
    GPIOA->MODER |= (GPIO_ALTERNATE_Mode << GPIO_MODER_MODE0_Pos
                | GPIO_ALTERNATE_Mode << GPIO_MODER_MODE1_Pos
                | GPIO_DIGITAL_OUT_Mode << GPIO_MODER_MODE8_Pos
                | GPIO_DIGITAL_OUT_Mode << GPIO_MODER_MODE15_Pos);

    // Set desired alternate functions.
    GPIOA->AFR[0] |= (ALTERNATE_FUNCTION_1 << GPIO_AFRL_AFSEL0_Pos
                | ALTERNATE_FUNCTION_1 << GPIO_AFRL_AFSEL1_Pos);

    /*! GPIOB setup */
    // Reset pin-modes.
    GPIOB->MODER &= ~(GPIO_MODER_Msk << GPIO_MODER_MODE3_Pos
                | GPIO_MODER_Msk << GPIO_MODER_MODE4_Pos
                | GPIO_MODER_Msk << GPIO_MODER_MODE5_Pos
                | GPIO_MODER_Msk << GPIO_MODER_MODE6_Pos
                | GPIO_MODER_Msk << GPIO_MODER_MODE9_Pos);

    // Set desired pin-modes.
    GPIOB->MODER |= (GPIO_ALTERNATE_Mode << GPIO_MODER_MODE3_Pos
                | GPIO_ALTERNATE_Mode << GPIO_MODER_MODE4_Pos
                | GPIO_ALTERNATE_Mode << GPIO_MODER_MODE5_Pos
                | GPIO_DIGITAL_OUT_Mode << GPIO_MODER_MODE6_Pos
                | GPIO_DIGITAL_OUT_Mode << GPIO_MODER_MODE9_Pos);

    // Set desired alternate functions.
    GPIOB->AFR[0] |= (ALTERNATE_FUNCTION_6 << GPIO_AFRL_AFSEL3_Pos
                | ALTERNATE_FUNCTION_6 << GPIO_AFRL_AFSEL4_Pos
                | ALTERNATE_FUNCTION_6 << GPIO_AFRL_AFSEL5_Pos);

    // @todo Implement interrupt setup for PB9.

    /*! GPIOC setup */
    // Reset pin-modes.
    GPIOC->MODER &= ~(GPIO_MODER_Msk << GPIO_MODER_MODE14_Pos
                | GPIO_MODER_Msk << GPIO_MODER_MODE15_Pos);

    // Set desired pin-modes.
    GPIOC->MODER |= (GPIO_DIGITAL_IN_Mode << GPIO_MODER_MODE14_Pos
                | GPIO_DIGITAL_IN_Mode << GPIO_MODER_MODE15_Pos);

    // @todo Implement interrupt setup for PC14 and PC15.

    /*! Setup timer 2 */
    // Enable timer clocking.
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;

    // Set maximum posible aouto reload value - 2^32-1 = 0xFFFFFFFF for this register.
    TIM2->ARR = 0xFFFFFFFF;

    // Set up timer into incremental encoder mode.
//  TIM2->CCER |= 0x02; // Should be uncommented if encoder direction reversal is needed.
    TIM2->SMCR |= 0x03;

    // Generate update event to update ARR value and start timer.
    TIM2->EGR |= TIM_CR1_CEN;
    TIM2->CR1 |= TIM_CR1_CEN;

    setup_spi3();
}

/*!
 * @brief Toggle the PA8 pin high.
 */
void set_pa8_high(void)
{
    GPIOA->BSRR = GPIO_BSRR_BS8;
}


/*!
 * @brief Toggle the PA8 pin low.
 */
void set_pa8_low(void)
{
    GPIOA->BSRR = GPIO_BSRR_BR8;
}


/*!
 * @brief Toggle the PA15 pin high.
 */
void set_pa15_high(void)
{
    GPIOA->BSRR = GPIO_BSRR_BS15;
}


/*!
 * @brief Toggle the PA15 pin low.
 */
void set_pa15_low(void)
{
    GPIOA->BSRR = GPIO_BSRR_BR15;
}


/*!
 * @brief Toggles the PB6 pin high. Disables TMC5130.
 */
void set_pb6_high(void)
{
    GPIOB->BSRR = GPIO_BSRR_BS6;
}


/*!
 * @brief Toggles the PB6 pin low. Enables TMC5130.
 */
void set_pb6_low(void)
{
    GPIOB->BSRR = GPIO_BSRR_BR6;
}


/****************************************************************************************/
/*                                                                                      */
/*                                 ESP32 communication                                  */
/*                                                                                      */
/****************************************************************************************/

/*!
 * @brief Enables UART1 with a given data rate.
 */
void setup_uart1(uint32_t desired_data_rate_in_bauds)
{
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;

    GPIOA->MODER &= ~(GPIO_MODER_Msk << GPIO_MODER_MODE9_Pos
                | GPIO_MODER_Msk << GPIO_MODER_MODE10_Pos);

    // Set desired pin-modes.
    GPIOA->MODER |= (GPIO_ALTERNATE_Mode << GPIO_MODER_MODE9_Pos
                | GPIO_ALTERNATE_Mode << GPIO_MODER_MODE10_Pos);

    GPIOA->AFR[1] &= ~(ALTERNATE_FUNCTION_Msk << GPIO_AFRH_AFSEL9_Pos
                | ALTERNATE_FUNCTION_Msk << GPIO_AFRH_AFSEL10_Pos);

    // Set desired alternate functions.
    GPIOA->AFR[1] |= (ALTERNATE_FUNCTION_7 << GPIO_AFRH_AFSEL9_Pos
                | ALTERNATE_FUNCTION_7 << GPIO_AFRH_AFSEL10_Pos);


    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

    // Set desired data rate.
    USART1->BRR = SYSTEM_MAIN_FREQUENCY/desired_data_rate_in_bauds;

    // Enable TX and RX channels and UART iteslf.
    USART1->CR1 |= USART_CR1_UE;
    USART1->CR1 |= USART_CR1_TE | USART_CR1_RE;
}

/*!
 * @brief Sends a single byte via UART1.
 *
 * @todo update function because right now it is just a dummy function without any control and mistake handling.
 */
uint32_t uart1_send_single_byte(uint8_t byte_to_send)
{
    while((USART1->ISR & USART_ISR_TC) != USART_ISR_TC){}
    while((USART1->ISR & USART_ISR_TXE_TXFNF) != USART_ISR_TXE_TXFNF) {}
    USART1->TDR = byte_to_send;

    return 0;
}

/*!
 * @brief Reads single byte from the UART1.
 *
 * @todo Implement the function.
 */
uint8_t uart1_read_single_byte(void);

/*!
 * @brief Sends multiple bytes via UART1.
 *
 * @todo Implement the function.
 */
void uart1_send_array(uint32_t number_of_bytes, uint8_t data_to_send[]);


/****************************************************************************************/
/*                                                                                      */
/*                                    Other stuff                                       */
/*                                                                                      */
/****************************************************************************************/



