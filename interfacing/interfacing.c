/*!
 * @file interfacing.c
 */

#include "interfacing.h"





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



