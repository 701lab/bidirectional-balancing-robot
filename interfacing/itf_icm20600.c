/*!
 * @file itf_icm20600.c
 *
 * Robots board v2.0 uses STM32G474RBT6 MCU.
 *  PCB contains ICM-20600 3 axis gyroscope and accelerometer.
 *
 *  ICM-20600:
 *      PB13-PB15 - SCK, MISO, MOSI respectively (SPI2, alternate function 5);
 *      PC6 - ICM_CS (digital output) - ICM-20600 chip selected;
 *      PB11 - ICM_INT2 (external interrupt input) - ICM-20600 interrupt output 2;
 *      PB12 - ICM_INT1 (external interrupt input) - ICM-20600 interrupt output 1;
 */

#include "itf_icm20600.h"


/****************************************************************************************/
/*                                                                                      */
/*                            Static functions declarations                             */
/*                                                                                      */
/****************************************************************************************/

static void setup_icm20600_gpio ( void );

static void setup_spi2( void );

/****************************************************************************************/
/*                                                                                      */
/*                          Static functions implementations                            */
/*                                                                                      */
/****************************************************************************************/

static inline void setup_icm20600_gpio ( void )
{
    // Enable clocking
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_GPIOCEN;

    /*! GPIOB setup */
    GPIOB->MODER &= ~(GPIO_MODER_Msk << GPIO_MODER_MODE11_Pos
                      | GPIO_MODER_Msk << GPIO_MODER_MODE12_Pos
                      | GPIO_MODER_Msk << GPIO_MODER_MODE13_Pos
                      | GPIO_MODER_Msk << GPIO_MODER_MODE14_Pos
                      | GPIO_MODER_Msk << GPIO_MODER_MODE15_Pos);
    GPIOB->MODER |= (GPIO_DIGITAL_IN_Mode << GPIO_MODER_MODE11_Pos
                     | GPIO_DIGITAL_IN_Mode << GPIO_MODER_MODE12_Pos
                     | GPIO_ALTERNATE_Mode << GPIO_MODER_MODE13_Pos
                     | GPIO_ALTERNATE_Mode << GPIO_MODER_MODE14_Pos
                     | GPIO_ALTERNATE_Mode << GPIO_MODER_MODE15_Pos);

    GPIOB->AFR[1] |= 5 << GPIO_AFRH_AFSEL13_Pos
                     | 5 << GPIO_AFRH_AFSEL14_Pos
                     | 5 << GPIO_AFRH_AFSEL15_Pos;

    /*! GPIOC setup */
    GPIOC->MODER &= ~(GPIO_MODER_Msk << GPIO_MODER_MODE6_Pos);
    GPIOC->MODER |= (GPIO_DIGITAL_OUT_Mode << GPIO_MODER_MODE6_Pos);

    set_icm20600_cs_high();
}

/*!
 * @brief Sets SPI2 with given frequency.
 *
 * In STM32 SPI can use SYSCLK frequency divided by 2 to the power from 1 to 8: 2, 4, 8 ... 256.
 *      Thus we need to set up SPI with the biggest possible frequency lower than the desired one.
 *      To do this, the algorithm iteratively checks whether the current prescaler matches the given criterion.
 *      If not, the prescaler is increased until the desired result is obtained.
 */
static inline void setup_spi2()
{
    // Enable SPI clocking.
    RCC->APB1ENR1 |= RCC_APB1ENR1_SPI2EN;

    // Baud rate prescaler choosing algorithm.
    uint32_t baud_rate_divider = 2;
    uint32_t baud_rate = 0;

    for ( int i = 0; i < 8; ++i )
    {
        if ((SYSTEM_MAIN_FREQUENCY / baud_rate_divider) <= SPI2_DESIRED_FREQUENCY)
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

    // setup SPI2 as master with disabled hardware CS.
    SPI2->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI | (baud_rate << SPI_CR1_BR_Pos) | SPI_CR1_MSTR;
    SPI2->CR2 |= SPI_CR2_FRXTH;
    SPI2->CR1 |= SPI_CR1_SPE;
}

/****************************************************************************************/
/*                                                                                      */
/*                          Global functions implementations                            */
/*                                                                                      */
/****************************************************************************************/

void setup_icm20600_peripherals( void )
{
    setup_icm20600_gpio();

    setup_spi2();
}

/*!
 * @brief Transmits and receives a single byte via SPI2.
 *
 * @param byte_to_be_sent - byte that will be sent via SPI2.
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


void set_icm20600_cs_high( void )
{
    GPIOC->BSRR = GPIO_BSRR_BS6;
}

void set_icm20600_cs_low( void )
{
    GPIOC->BSRR = GPIO_BSRR_BR6;
}
