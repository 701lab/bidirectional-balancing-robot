/*!
 * @file itf_drv8701.c
 *
 * Robots board v2.0 uses STM32G474RBT6 MCU.
 *  PCB contains two DRV8701 DC-motor drivers and two encoders. Encoders are not related to a particular motor driver,
 * but to make things easier it is better to use Encoder1 with Motor1 and Encoder2 with Motor2, thus they will be set up together.
 *
 *  Motor driver 1:
 *      PC0, PC1 - M1_IN1, M1_IN2 respectively (TIM1 CH1 and CH2 respectively, alternate function 2, PWM output mode) - Driver input signals;
 *      PB9 - M_NSLEEP (digital output) - DRV8701 sleep pin: 0 on pin - driver is disabled, 1 - enabled. The same pin for both drivers;
 *      PC13 - M1_NFAULT (external interrupt).
 *
 *  Motor driver 2:
 *      PC2, PC3 - M2_IN1, M2_IN2 respectively (TIM1 CH3 and CH4 respectively, alternate function 2, PWM output mode) - Driver input signals;
 *      PB9 - M_NSLEEP (digital output) - DRV8701 sleep pin: 0 on pin - driver is disabled, 1 - enabled. The same pin for both drivers;
 *      PC15 - M2_NFAULT (external interrupt).
 *
 *  Encoder 1:
 *      PB4, PB5 - encoder1 inputs 1 and 2 respectively (TIM3 CH1 and CH2 respectively, alternate function 2, ENCODER counter mode).
 *
 *  Encoder 2:
 *      PB6, PB7 - encoder2 inputs 1 and 2 respectively (TIM4 CH1 and CH2 respectively, alternate function 2, ENCODER counter mode).
 *
 *  Current sense 1:
 *      PA0 - motor 1 current sense (analog mode) - INA240A1 OUT.
 *
 *  Current sense 2:
 *      PA1 - motor 2 current sense (analog mode) - INA240A1 OUT.
 *
 *  @todo This code does not implements interrupt handling. It must be implemented in the future.
 */

#include "itf_drv8701.h"

/****************************************************************************************/
/*                                                                                      */
/*                            Static functions declarations                             */
/*                                                                                      */
/****************************************************************************************/

static void setup_drv8701_1_gpio( void );
static void setup_drv8701_2_gpio( void );

static void setup_drv8701_1_timers( void );
static void setup_drv8701_2_timers( void );

//static void setup_drv8701_1_interrupt( void );
//static void setup_drv8701_2_interrupt( void );

/****************************************************************************************/
/*                                                                                      */
/*                          Static functions implementations                            */
/*                                                                                      */
/****************************************************************************************/

static void setup_drv8701_1_gpio( void )
{
    // Enable clocking
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_GPIOCEN;

    /*! GPIOA setup */
    GPIOA->MODER &= ~( GPIO_MODER_Msk << GPIO_MODER_MODE0_Pos);
    GPIOA->MODER |= ( GPIO_ANALOG_IN_Mode << GPIO_MODER_MODE0_Pos);

    /*! GPIOB setup */
    uint32_t pb9_is_not_setup = (GPIOB->MODER & GPIO_MODER_MODE9_Msk) == GPIO_MODER_MODE9_Msk;
    if ( pb9_is_not_setup )
    {
        GPIOB->MODER &= ~(GPIO_MODER_Msk << GPIO_MODER_MODE9_Pos);
        GPIOB->MODER |= (GPIO_DIGITAL_OUT_Mode << GPIO_MODER_MODE9_Pos);
        set_drv8701_enable_low(); // disable driver by default
    }

    /*! GPIOC setup */
    GPIOC->MODER &= ~(GPIO_MODER_Msk << GPIO_MODER_MODE0_Pos
                      | GPIO_MODER_Msk << GPIO_MODER_MODE1_Pos
                      | GPIO_MODER_Msk << GPIO_MODER_MODE13_Pos);
    GPIOC->MODER |= (GPIO_ALTERNATE_Mode << GPIO_MODER_MODE0_Pos
                     | GPIO_ALTERNATE_Mode << GPIO_MODER_MODE1_Pos
                     | GPIO_DIGITAL_IN_Mode << GPIO_MODER_MODE13_Pos);

    GPIOC->AFR[0] |= 2 << GPIO_AFRL_AFSEL0_Pos
                     | 2 << GPIO_AFRL_AFSEL1_Pos;
}

static void setup_drv8701_2_gpio( void )
{
    // Enable clocking
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_GPIOCEN;

    /*! GPIOA setup */
    GPIOA->MODER &= ~( GPIO_MODER_Msk << GPIO_MODER_MODE1_Pos);
    GPIOA->MODER |= ( GPIO_ANALOG_IN_Mode << GPIO_MODER_MODE1_Pos);

    /*! GPIOB setup */
    uint32_t pb9_is_not_setup = (GPIOB->MODER & GPIO_MODER_MODE9_Msk) == GPIO_MODER_MODE9_Msk;
    if ( pb9_is_not_setup )
    {
        GPIOB->MODER &= ~(GPIO_MODER_Msk << GPIO_MODER_MODE9_Pos);
        GPIOB->MODER |= (GPIO_DIGITAL_OUT_Mode << GPIO_MODER_MODE9_Pos);
        set_drv8701_enable_low(); // disable driver by default
    }

    /*! GPIOC setup */
    GPIOC->MODER &= ~(GPIO_MODER_Msk << GPIO_MODER_MODE2_Pos
                      | GPIO_MODER_Msk << GPIO_MODER_MODE3_Pos
                      | GPIO_MODER_Msk << GPIO_MODER_MODE15_Pos);
    GPIOC->MODER |= (GPIO_ALTERNATE_Mode << GPIO_MODER_MODE2_Pos
                     | GPIO_ALTERNATE_Mode << GPIO_MODER_MODE3_Pos
                     | GPIO_DIGITAL_IN_Mode << GPIO_MODER_MODE15_Pos);

    GPIOC->AFR[0] |= 2 << GPIO_AFRL_AFSEL2_Pos
                     | 2 << GPIO_AFRL_AFSEL3_Pos;
}

static void setup_drv8701_1_timers( void )
{
    // Enable clocking.
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

    // Disable timer in case was enabled previously.
    TIM1->CR1 &= ~TIM_CR1_CEN;

    TIM1->PSC = 0;              // Timer speed = bus speed
    TIM1->ARR = PWM_PRECISION;

    TIM1->CCMR1 |= 0x6868;
    TIM1->CCER |= 0x0011;   // Enable CH1-2

    TIM1->CR1 |= TIM_CR1_ARPE;  // Enable Auto reload preload
    TIM1->BDTR |= TIM_BDTR_MOE; // Main output enable

    // Zero velocity by default.
    TIM1->CCR1 = PWM_PRECISION;
    TIM1->CCR2 = PWM_PRECISION;

    TIM1->CR1 |= TIM_CR1_CEN; // Enable timer
}

static void setup_drv8701_2_timers( void )
{
    // Enable clocking.
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

    // Disable timer in case was enabled previously.
    TIM1->CR1 &= ~TIM_CR1_CEN; // Enable timer

    TIM1->PSC = 0;              // Timer speed = bus speed
    TIM1->ARR = PWM_PRECISION;

    TIM1->CCMR2 |= 0x6868;
    TIM1->CCER |= 0x1100;   // Enable CH3-4

    TIM1->CR1 |= TIM_CR1_ARPE;  // Enable Auto reload preload
    TIM1->BDTR |= TIM_BDTR_MOE; // Main output enable

    // Zero velocity by default.
    TIM1->CCR3 = PWM_PRECISION;
    TIM1->CCR4 = PWM_PRECISION;

    TIM1->CR1 |= TIM_CR1_CEN; // Enable timer
}

/****************************************************************************************/
/*                                                                                      */
/*                          Global functions implementations                            */
/*                                                                                      */
/****************************************************************************************/

void setup_drv8701_1( void )
{
    setup_drv8701_1_gpio();

    setup_drv8701_1_timers();
}

void setup_drv8701_2( void )
{
    setup_drv8701_2_gpio();

    setup_drv8701_2_timers();
}

void setup_encoder1( void )
{
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;

    GPIOB->MODER &= ~(GPIO_MODER_Msk << GPIO_MODER_MODE4_Pos
                | GPIO_MODER_Msk << GPIO_MODER_MODE5_Pos);
    GPIOB->MODER |= (GPIO_ALTERNATE_Mode << GPIO_MODER_MODE4_Pos
                | GPIO_ALTERNATE_Mode << GPIO_MODER_MODE5_Pos);

    GPIOB->AFR[0] |= 2 << GPIO_AFRL_AFSEL4_Pos | 2 << GPIO_AFRL_AFSEL5_Pos;

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
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;

    GPIOB->MODER &= ~(GPIO_MODER_Msk << GPIO_MODER_MODE6_Pos
                | GPIO_MODER_Msk << GPIO_MODER_MODE7_Pos);
    GPIOB->MODER |= (GPIO_ALTERNATE_Mode << GPIO_MODER_MODE6_Pos
                | GPIO_ALTERNATE_Mode << GPIO_MODER_MODE7_Pos);

    GPIOB->AFR[0] |= 2 << GPIO_AFRL_AFSEL6_Pos | 2 << GPIO_AFRL_AFSEL7_Pos;

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
    setup_drv8701_1();
    
    setup_encoder1();
}

void setup_motor2( void )
{
    setup_drv8701_2();
    
    setup_encoder2();
}

int16_t get_encoder1_value( void )
{
    return TIM3->CNT;
}

int16_t get_encoder2_value( void )
{
    return TIM4->CNT;
}

void set_drv8701_enable_high(void)
{
    GPIOB->BSRR = GPIO_BSRR_BS9;
}

void set_drv8701_enable_low(void)
{
    GPIOB->BSRR = GPIO_BSRR_BR9;
}

uint32_t set_drv8701_1_duty_cycle( int32_t required_duty_cycle )
{
    uint32_t max_duty_cycle = PWM_PRECISION;

    if ( required_duty_cycle < 0 )
    {
        if ( required_duty_cycle < -max_duty_cycle ) // PWM task negative but higher than maximum -> set maximum PWM in reverse direction
        {
            TIM1->CCR1 = max_duty_cycle;
            TIM1->CCR2 = 0;

            return add_mistake_to_the_log( MCU_M1_PWM_TASK_LOWER_THAN_MINIMUM_Err );
        }
        else // PWM task is negative and less than maximum -> set task PWM in reverse direction
        {
            TIM1->CCR1 = max_duty_cycle;
            TIM1->CCR2 = max_duty_cycle + required_duty_cycle;
        }
    }
    else
    { /* required_duty_cycle_coefficient >= 0 */
        if ( required_duty_cycle > max_duty_cycle ) // PWM task is positive but higher than maximum -> set maximum PWM in forward direction
        {
            TIM1->CCR1 = 0;
            TIM1->CCR2 = max_duty_cycle;

            return add_mistake_to_the_log( MCU_M1_PWM_TASK_HIGHER_THAN_MAXIMUM_Err );
        }
        else // PWM task is positive and less than maximum -> set maximum PWM in forward direction
        {
            TIM1->CCR1 = max_duty_cycle - required_duty_cycle;
            TIM1->CCR2 = max_duty_cycle;
        }
    } /* required_duty_cycle_coefficient >= 0 */

    return 0;
}

uint32_t set_drv8701_2_duty_cycle( int32_t required_duty_cycle )
{
    uint32_t max_duty_cycle = PWM_PRECISION;

    if ( required_duty_cycle < 0 )
    {
        if ( required_duty_cycle < -max_duty_cycle ) // PWM task negative but higher than maximum -> set maximum PWM in reverse direction
        {
            TIM1->CCR3 = max_duty_cycle;
            TIM1->CCR4 = 0;

            return add_mistake_to_the_log( MCU_M2_PWM_TASK_LOWER_THAN_MINIMUM_Err );
        }
        else // PWM task is negative and less than maximum -> set task PWM in reverse direction
        {
            TIM1->CCR3 = max_duty_cycle;
            TIM1->CCR4 = max_duty_cycle + required_duty_cycle;
        }
    }
    else
    { /* required_duty_cycle_coefficient >= 0 */
        if ( required_duty_cycle > max_duty_cycle ) // PWM task is positive but higher than maximum -> set maximum PWM in forward direction
        {
            TIM1->CCR3 = 0;
            TIM1->CCR4 = max_duty_cycle;

            return add_mistake_to_the_log( MCU_M2_PWM_TASK_HIGHER_THAN_MAXIMUM_Err );
        }
        else // PWM task is positive and less than maximum -> set maximum PWM in forward direction
        {
            TIM1->CCR3 = max_duty_cycle - required_duty_cycle;
            TIM1->CCR4 = max_duty_cycle;
        }
    } /* required_duty_cycle_coefficient >= 0 */

    return 0;
}
