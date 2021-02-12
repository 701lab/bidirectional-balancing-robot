/*!
 * @file itf_nrf24l01p.c
 *
 * Robots board v2.0 uses STM32G474RBT6 MCU.
 *  PCB contains NRF24L01+ radio module.
 *
 * NRF24l01+:
 *  PC10-PC12 - SCK, MISO, MOSI respectively (SPI3, alternate function 6);
 *  PD2 - NRF_CS (digital output) NRF24L01+ chip selected;
 *  PB3 - NRF_CE (digital output) NRF24L01+ chip enable;
 *  PA15 - NRF_IRQ (external interrupt input) NRF24L01+ interrupt request.
 */

#include "itf_nrf24l01p.h"

/****************************************************************************************/
/*                                                                                      */
/*                            Static functions declarations                             */
/*                                                                                      */
/****************************************************************************************/

static void setup_nrf24l01p_gpio ( void );

static inline void setup_spi3( void );

/****************************************************************************************/
/*                                                                                      */
/*                          Static functions implementations                            */
/*                                                                                      */
/****************************************************************************************/

static inline void setup_nrf24l01p_gpio ( void )
{
    // Enable clocking
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_GPIOCEN | RCC_AHB2ENR_GPIODEN;

    /*! GPIOA setup */
    GPIOA->MODER &= ~( GPIO_MODER_Msk << GPIO_MODER_MODE15_Pos );
    GPIOA->MODER |= ( GPIO_DIGITAL_IN_Mode << GPIO_MODER_MODE15_Pos);

    // Place for PA15 interrupt setup if needed

    /*! GPIOB setup */
    GPIOB->MODER &= ~(GPIO_MODER_Msk << GPIO_MODER_MODE3_Pos);
    GPIOB->MODER |= (GPIO_DIGITAL_OUT_Mode << GPIO_MODER_MODE3_Pos);

    /*! GPIOC setup */
    GPIOC->MODER &= ~(GPIO_MODER_Msk << GPIO_MODER_MODE10_Pos
                      | GPIO_MODER_Msk << GPIO_MODER_MODE11_Pos
                      | GPIO_MODER_Msk << GPIO_MODER_MODE12_Pos);
    GPIOC->MODER |= (GPIO_ALTERNATE_Mode << GPIO_MODER_MODE10_Pos
                     | GPIO_ALTERNATE_Mode << GPIO_MODER_MODE11_Pos
                     | GPIO_ALTERNATE_Mode << GPIO_MODER_MODE12_Pos);

    GPIOC->AFR[1] |= 6 << GPIO_AFRH_AFSEL10_Pos
                     | 6 << GPIO_AFRH_AFSEL11_Pos
                     | 6 << GPIO_AFRH_AFSEL12_Pos;

    /*! GPIOD setup */
    GPIOD->MODER &= ~( GPIO_MODER_Msk << GPIO_MODER_MODE2_Pos);
    GPIOD->MODER |= GPIO_DIGITAL_OUT_Mode << GPIO_MODER_MODE2_Pos;

    nrf24_set_cs_high();
}

/*!
 * @brief Sets SPI3 with given frequency.
 *
 * In STM32 SPI can use SYSCLK frequency divided by 2 to the power from 1 to 8: 2, 4, 8 ... 256.
 *      Thus we need to set up SPI with the biggest possible frequency lower than the desired one.
 *      To do this, the algorithm iteratively checks whether the current prescaler matches the given criterion.
 *      If not, the prescaler is increased until the desired result is obtained.
 */
static inline void setup_spi3()
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


/****************************************************************************************/
/*                                                                                      */
/*                          Global functions implementations                            */
/*                                                                                      */
/****************************************************************************************/

void setup_nrf24l01p_peripherals( void )
{
    setup_nrf24l01p_gpio();

    setup_spi3();
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


void nrf24_set_cs_high( void )
{
    GPIOD->BSRR = GPIO_BSRR_BS2;
}

void nrf24_set_cs_low( void )
{
    GPIOD->BSRR = GPIO_BSRR_BR2;
}

void nrf24_set_ce_high( void )
{
    GPIOB->BSRR = GPIO_BSRR_BS3;
}

void nrf24_set_ce_low( void )
{
    GPIOB->BSRR = GPIO_BSRR_BR3;
}
