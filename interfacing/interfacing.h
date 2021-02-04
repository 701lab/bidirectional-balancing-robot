/*!
 * @file interfacing.h
 *
 * @version 0.1.0
 */

#ifndef INTERFACING_H_
#define INTERFACING_H_

#include "stm32g474xx.h"
#include "itf_error_codes.h"

/****************************************************************************************/
/*                                                                                      */
/*                             User-adjustable definitions                              */
/*                                                                                      */
/****************************************************************************************/

/*!
 * @brief Allows mistake log to save time of the mistake.
 *
 * In order to log time mistakes log needs to have time clocking timer that will count seconds and milliseconds.
 *      In general, it can be any up-counting timer with a global variable that will contain the amount of seconds passed.
 */
#define MISTAKE_LOG_SHOULD_SAVE_TIME

/*!
 * @brief Desired SYSCLK and HCLK frequencies.
 *
 * Used in all setup functions: for SYSCLK setup, timers setup, PWM setup - everything, that may need to know real system frequency.
 *
 * @note Minimum frequency allowed - 8 MHz, maximum - 150 MHz.
 *
 * @note For lower than 32 MHz frequencies setup 0.5 MHz step is used:
 *          8000000, 8500000, 9000000 ... 20500000, 21000000 ... 31000000, 31500000.
 *
 * @note For equal or higher than 32 MHz frequencies setup step 2 MHz is used:
 *          32000000, 34000000 ... 62000000, 64000000 ... 148000000, 15000000.
 */
#ifndef SYSTEM_MAIN_FREQUENCY
    #define SYSTEM_MAIN_FREQUENCY               32000000 // Hz
#endif /* SYSTEM_MAIN_FREQUENCY */

/*!
 * Frequency of the system timer based interrupt.
 *
 * @note Can be changed by user.
 */
#ifndef SYSTICK_INTERRUPT_FREQUENCY
    #define SYSTICK_INTERRUPT_FREQUENCY         10 // Hz
#endif /* SYSTICK_INTERRUPT_FREQUENCY */

#ifndef MISTAKES_LOG_SIZE
    #define MISTAKES_LOG_SIZE                   10 // Records
#endif /* MISTAKES_LOG_SIZE */

//! TMC5130 and AEAT8800 SPI work frequency.
#ifndef SPI3_DESIRED_FREQUENCY
    #define SPI3_DESIRED_FREQUENCY              1000000 // Hz
#endif /* SPI3_WORKING_FREQUENCY */

//! SPI2 work frequency.
#ifndef SPI2_DESIRED_FREQUENCY
    #define SPI2_DESIRED_FREQUENCY              8000000 // Hz
#endif /* SPI2_WORKING_FREQUENCY */


#ifndef PWM_FREQUENCY
    #define PWM_FREQUENCY                   20000   // Hz = 20 Khz
    #define PWM_PRECISION                   ( SYSCLK_FREQUENCY / PWM_FREQUENCY - 1 )    // -1 is needed for proper timers setup. Equation shouldn't be changed
#endif /* PWM_FREQUENCY */


/****************************************************************************************/
/*                                                                                      */
/*                                        ENUMs                                         */
/*                                                                                      */
/****************************************************************************************/

/*! Logic questions answer for more clear code. */
enum question_answers
{
    NO = 0UL,
    YES = 1UL
};

/*! FLASH_OTPTR register NRST_MODE field options for error checking with static analyzer. */
typedef enum mcu_nrst_mode
{
    NRST_INPUT_ONLY_MODE = 1U,      // 0b01
    NRST_GPIO_MODE = 2U,            // 0b10
    NRST_INPUT_OUTPUT_MODE = 3U,    // 0b11 (default)
} mcu_nrst_mode;

/*! RCC_PLLCFGR PLL source values for error checking with static analyzer. */
typedef enum mcu_pll_source
{
    HSI16_AS_PLL_SOURCE = RCC_PLLCFGR_PLLSRC_HSI,
    HSE_AS_PLL_SOURCE = RCC_PLLCFGR_PLLSRC_HSE,
} mcu_pll_source;

/*! RCC_CFGR MCO source options for error checking with static analyzer. */
typedef enum mcu_rcc_mco_source
{
    MCO_DISABLED = 0U,              // 0b0000
    SYSCLK_AS_MCO_SOURCE = 1U,      // 0b0001
    HSI16_AS_MCO_SOURCE = 3U,       // 0b0011
    HSE_AS_MCO_SOURCE = 4U,         // 0b0100
    PLL_AS_MCO_SOURCE = 5U,         // 0b0101
    LSI_AS_MCO_SOURCE = 6U,         // 0b0110
    LSE_AS_MCO_SOURCE = 7U,         // 0b0111
    HSI48_AS_MCO_SOURCE = 8U,       // 0b1000
} mcu_rcc_mco_source;

/****************************************************************************************/
/*                                                                                      */
/*                                     Structures                                       */
/*                                                                                      */
/****************************************************************************************/

/*!
 * Mistakes log element definition.
 *
 * Defining MISTAKE_LOG_SHOULD_SAVE_TIME enables saving time stamps of the occurred mistakes.
 *      In this case, the clocking timer should be set up and the add_mistake_to_the_log function should be properly implemented
 *      with respect to this timer. Otherwise, all time stamps will be 0.
 *
 * Not defining MISTAKE_LOG_SHOULD_SAVE_TIME disables time stamps saving, which lowers SRAM memory consumption by 4*MISTAKES_LOG_SIZE bytes.
 */
typedef struct{
    uint16_t mistake_code; //!< Code of the occurred mistake. Used always.

#ifdef MISTAKE_LOG_SHOULD_SAVE_TIME
    uint16_t mistake_time_in_seconds; //!< Second of mistake occurrence from timers setup moment. Used when enabled.
    uint16_t mistake_time_in_milliseconds; //!< Millisecond of mistake occurrence. Used when enabled.
#endif /* MISTAKE_LOG_SHOULD_SAVE_TIME */

} mistake;



/****************************************************************************************/
/*                                                                                      */
/*                               General setup functions                                */
/*                                                                                      */
/****************************************************************************************/

void setup_flash_option_bytes(void);

void set_nrst_pin_mode(mcu_nrst_mode desired_mode);

// Toggles PA4 pin high.
void light_up_blue_led(void);

// Toggles PA4 pin low.
void put_down_blue_led(void);

//! Toggles PA4 pin.
void toggle_blue_led(void);

//! Toggles PA5 pin high.
void light_up_red_led(void);

//! Toggles PA5 pin low.
void put_down_red_led(void);

//! Toggles PA5 pin.
void toggle_red_led(void);

//! Sets up SysTick timer and interrupt.
void setup_system_timer(void);

//! Sets up only features related to the main board functionality.
void setup_base_peripherals(void);

//! Sets SPI2 with a frequency stated in SPI2_DESIRED_FREQUENCY definition.
void setup_spi2();

//! Transmits and receives a single byte via SPI2.
uint8_t spi2_write_single_byte(const uint8_t byte_to_send);

//! Sets up all board's features.
void setup_all_peripherals(void);

// ========================================================================//
// =                                                                     = //
// =                  =====  Not for this version =====                  = //
// =                                                                     = //
// ========================================================================//

//
//void adcs_setup(void);
//
//void uart3_setup(uint32_t data_rate);
//
//uint32_t uart3_send_single_byte(uint8_t byte_to_send);
//
//uint8_t uart3_read_single_byte(void);
//
//void uart3_send_array(uint32_t number_of_bytes, uint8_t data_to_send[]);

/****************************************************************************************/
/*                                                                                      */
/*                                 TMC5130 + AEAT-8800                                  */
/*                                                                                      */
/****************************************************************************************/

void setup_spi3();

//! Transmits and receives a single byte via SPI3.
uint8_t spi3_write_single_byte(const uint8_t byte_to_send);

//! This function is needed in particular for AEAT8800
void spi3_set_mode(uint8_t mode);

void setup_tmc5130_aeat8800_peripherals();

void set_pa8_high(void);
void set_pa8_low(void);

void set_pa15_high(void);
void set_pa15_low(void);

//! Toggles the PB6 pin high.
void disable_tmc5130(void);

//! Toggles the PB6 pin low.
void enable_tmc5130(void);

/****************************************************************************************/
/*                                                                                      */
/*                                 ESP32 communication                                  */
/*                                                                                      */
/****************************************************************************************/

void setup_uart1(uint32_t desired_data_rate_in_bauds);

uint32_t uart1_send_single_byte(uint8_t byte_to_send);

/*!
 * @brief Reads a single byte from the UART1.
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
/*                            Global variables declaration                              */
/*                                                                                      */
/****************************************************************************************/

/*!
 * @brief Declares global variables in all files that includes this one.
 *
 * If there is DECLARE_GLOBAL_VARIABLES #define in file - declares and initialize variables (should be only one such file - most of the time main.c).
 *
 * If there is no DECLARE_GLOBAL_VARIABLES #define in file - declares all variables as extern and doesn't initialize them.
 *          So for all libraries that will use interfacing.h file no additional defines needed.
 */
#ifndef DEFINE_GLOBAL_VARIABLES //!< This define is used for protection of redefinition of the variables,
                                //!< in case this file was included more than once.
    #define DEFINE_GLOBAL_VARIABLES 1

    #ifndef DECLARE_GLOBAL_VARIABLES    //!< If not defined, declare global variables as extern and doesn't initializes them.
        # define _DECL extern   // extern prefix.
        # define _INIT(x)       // no postfix = no initialization.
    #else                               //!< Else declare and initialize global variables.
        # define _DECL          // no prefix
        # define _INIT(x)  = x  // postix with variable initialization value.
    #endif /* DECLARE_GLOBAL_VARIABLES */

    /*
     * All global variables, that should be used outside of interfacing.h/.c files should be declared below.
     *
     * Declaration notation: "_DECL [standard C-variable declaration] _INIT(x);", where x - is initialization value.
     *      If there is no need for variable initialization postfix can be omitted: "_DECL [standard C-variable declaration];".
     *      But declraration without initialization is not MISRA-C compilant and must not be used in embedded systems!
     */

    _DECL mistake mistakes_log[MISTAKES_LOG_SIZE + 1]; //!< Declare mistakes log array with given size + 1 element. This additional last element will contain mistakes log overflow error.
    _DECL uint16_t mistakes_log_pointer _INIT(0); //!< Pointer to the current position of the mistake log to write to.
    _DECL uint16_t critical_mistake_detected _INIT(0); //!< Used for detection of critical mistakes. Handling of critical mistakes is not specified.

    /*!
     * Amount of seconds passed from the debug initialization. Needs to be -1 at initialization because after timer initialization
     *      update event is called to update timer parameters thus interrupt is called and at time 0 value will be incremented by 1, so it will start from 0.
     */
    _DECL uint32_t seconds_from_setup _INIT((uint32_t)(-1));

#endif /* DEFINE_GLOBAL_VARIABLES */

/****************************************************************************************/
/*                                                                                      */
/*                           Non-user-adjustable definitions                            */
/*                                                                                      */
/****************************************************************************************/

/*! Proper definitions of GPIO setup. */
#define GPIO_MODER_Msk              (3U)    // 0b11
#define GPIO_DIGITAL_IN_Mode        (0U)    // 0b00
#define GPIO_DIGITAL_OUT_Mode       (1U)    // 0b01
#define GPIO_ALTERNATE_Mode         (2U)    // 0b10
#define GPIO_ANALOG_IN_Mode         (3U)    // 0b11

/*! Proper definitions of OSPEED setup. */
#define GPIO_OSPEED_VERY_LOW        (0U)    // 0b00 - Fmax - 10 MHz at 3.3V
#define GPIO_OSPEED_LOW             (1U)    // 0b01 - Fmax - 50 MHz at 3.3V
#define GPIO_OSPEED_HIGH            (2U)    // 0b10 - Fmax - 100 MHz at 3.3V
#define GPIO_OSPEED_VERY_HIGH       (3U)    // 0b11 - Fmax - 180 MHz at 3.3V
#define GPIO_OSPEED_Msk             (3U)    // 0b11

/*! Proper definitions of alternate functions setup. */
#define ALTERNATE_FUNCTION_0        (0U)    // 0b0000
#define ALTERNATE_FUNCTION_1        (1U)    // 0b0001
#define ALTERNATE_FUNCTION_2        (2U)    // 0b0010
#define ALTERNATE_FUNCTION_3        (3U)    // 0b0011
#define ALTERNATE_FUNCTION_4        (4U)    // 0b0100
#define ALTERNATE_FUNCTION_5        (5U)    // 0b0101
#define ALTERNATE_FUNCTION_6        (6U)    // 0b0110
#define ALTERNATE_FUNCTION_7        (7U)    // 0b0111
#define ALTERNATE_FUNCTION_8        (8U)    // 0b1000
#define ALTERNATE_FUNCTION_9        (9U)    // 0b1001
#define ALTERNATE_FUNCTION_10       (10U)   // 0b1010
#define ALTERNATE_FUNCTION_11       (11U)   // 0b1011
#define ALTERNATE_FUNCTION_12       (12U)   // 0b1100
#define ALTERNATE_FUNCTION_13       (13U)   // 0b1101
#define ALTERNATE_FUNCTION_14       (14U)   // 0b1110
#define ALTERNATE_FUNCTION_15       (15U)   // 0b1111
#define ALTERNATE_FUNCTION_Msk      (15U)   // 0b1111

/*! System clock prescalers setup */
#if (SYSTEM_MAIN_FREQUENCY >= 32000000) // For frequencies equal or higher than 32 MHz we use the 2 MHz step.
    #define PLLN_VALUE              (SYSTEM_MAIN_FREQUENCY / 2000000)
    #define PLLR_VALUE              (0U) // PLLR is dividing by 2.
#endif

#if (SYSTEM_MAIN_FREQUENCY < 32000000) // For frequencies lower than 32 MHz we use the 2 MHz step.
    #define PLLN_VALUE              (SYSTEM_MAIN_FREQUENCY / 500000)
    #define PLLR_VALUE              (3U) // PLLR is dividing by 8.
#endif


#define PLLM_VALUE                  (1U) // PLLM is dividing by 2 because HSE frequency is 8 MHz.
#define PLLM_VALUE_WITH_HSI         (3U) // PLLM is dividing by 4 because HSI frequency is 16 MHz.
#define PLLQ_VALUE                  (3U) // PLLQ is dividing by 8.
#define PLLP_VALUE                  (8U) // PLLP current divider value.


/****************************************************************************************/
/*                                                                                      */
/*                                    Other stuff                                       */
/*                                                                                      */
/****************************************************************************************/

/*** Completely random value to determine the waiting-state length ***/
// @todo change this value to allow proper setups
#define DUMMY_DELAY_VALUE       20000

/*!
 * @brief
 *
 * @todo add description and usage
 */
#define SAFETY_DELAY_COEFFICIENT     42

/****************************************************************************************/
/*                                                                                      */
/*                                   Error checking                                     */
/*                                                                                      */
/****************************************************************************************/

//! Checks if system clock frequency was defined properly.
#if (SYSTEM_MAIN_FREQUENCY > 150000000)   //!< Absolute maximum value.
    #error "Clock speed define is higher than the maximum possible value of 150 MHz!"

#elif (SYSTEM_MAIN_FREQUENCY < 8000000) //!< Comfortable to use minimum value.
    #error "Clock speed define is less than the minimum possible value of 8 MHz!"

#elif (SYSTEM_MAIN_FREQUENCY >= 32000000 && SYSTEM_MAIN_FREQUENCY % 2000000 != 0)
    #error "Clock speeds above 32MHz must be multiple of 2 MHz!"

#elif (SYSTEM_MAIN_FREQUENCY < 32000000 && SYSTEM_MAIN_FREQUENCY % 500000 != 0)
    #error "Clock speed below 32Mhz must be multiple of 0.5 MHz!"
#endif


//! Checks if SysTick interrupt frequency was defined properly.
#if (SYSTEM_MAIN_FREQUENCY % 8 != 0) //!< These fluctuations should not be that significant.
    #warning "System main frequency is not multiple of 8, which will lead to SysTick interrupt frequency fluctuation!"

#elif (SYSTEM_MAIN_FREQUENCY % (SYSTICK_INTERRUPT_FREQUENCY)) //!< These fluctuations can be significant.
    #warning "System main frequency is not multiple of SYSTICK_INTERRUPT_FREQUENCY, which will lead to SysTick interrupt frequency fluctuation!"

#elif ((16777216 * 8 * SYSTICK_INTERRUPT_FREQUENCY)/(SYSTEM_MAIN_FREQUENCY) < 1) //!< 16777216 - 2^24 - maximum value of the SysTick prescalser.
    #error "SysTick interrupt frequency is so slow, that it does not feet into the SysTick prescaler. Suggestion: Make SysTick frequency higher!"

#endif

//! Checks if SPI working frequencies were set up properly.
#if (SPI2_DESIRED_FREQUENCY > 10000000)
    #error "SPI2 frequency is bigger than maximum allowed 10 MHz!"
#endif

#if (SPI3_DESIRED_FREQUENCY > 4000000)
    #error "SPI3 frequency is bigger than maximum allowed for TMC5130 4 MHz!"
#endif

/****************************************************************************************/
/*                                                                                      */
/*                              For the future updates                                  */
/*                                                                                      */
/****************************************************************************************/
/*
 * @todo Update system clock handling in a way, that it will work without mistakes without PLL - on HSE. This will make PLL fails a tolerable mistake.
 *
 * @todo Implement control system snippet storing functionality. It should monitor some parameters and add them to the log continuously. If something will
 *          go wrong it will store a half array of what was before a "wrong" event, and a half array of what was after.
 *
 * @todo Update PLL frequency setup to make it more flexible - allow for additional
 *
 * @todo There should be a rool, that all interfacing functions adds to mistake log only mistakes they found themselves, not mistakes from the functions they call.
 *          If needed functions can also return mistake code, but this return value should not be placed into the mistakes log by the calling function.
 *
 * @todo Add some kind of a setup status array, that will contain all possible setups functions flags and will show what functions was and wasn't setted up. Can
 *         be super useful to track what is on, at what is not. Also can help with understanding of what
 *
 * @todo Learn how to use and properly access MPU and trigonometry accelerator.
 *
 * @todo Refactor the code into different files.
 */


#endif /* INTERFACING_H_ */
