/*!
 * @file itf_error_codes.h
 *
 * @brief This file contains all error codes for the interfacing library.
 *
 * All error codes are divided into tolerable and critical. Tolerable errors can be fully handled by the system and just warn users
 *  about occurred problems. Critical mistakes can lead to unknown behavior of the system, which can cause harm and must be handled properly.
 *
 * @note Error definitions have _Err suffix to make them distinguishable from other types of definitions.
 * @note Interfacing related errors have MCU_ prefix to make them distinguishable from other types of errors.
 * @note Critical error handling behavior is not specified in this file.
 *
 * Error codes are stored as uint16_t variables. The first 15 bits are used as an error code number representation, meaning
 *  that there are 2^15-1 = 32767 available error codes. The most significant bit (MSb) is used as a critical error flag.
 *
 * All error codes are documented in the ITF_errors.md file. (not ready yet).
 *
 * @todo Add ITF_errors.md file with all needed descriptions and clean the comments on the error codes.
 * @todo Test the ability to really change MUC_NAMESPACE_OFFSET outside this file.
 */

#ifndef ITF_ERROR_CODES_H_
#define ITF_ERROR_CODES_H_

/*!
 * @todo Update this description
 *
 * @brief defines offset for MCU error codes.
 *
 * Can be re-defined in any file that includes interfacing.h or directly in interfacing.h.
 * if redefinition is used outside the interfacing.h file, #define must be placed be placed before inclusion of interfacing.h!
 */
#ifndef MCU_NAMESPACE_OFFSET
    #define MCU_NAMESPACE_OFFSET               (0U)
#endif /* MCU_ERROR_NAMESPACE_OFFSET */

/*!
 * Define that is used to set the sixteenth bit of the error code, representing a critical error.
 *  If this parameter is not present - the mistake is tolerable, meaning that the software will normally handle it and do not need to be stopped.
 */
#define CRITICAL_ERROR_FLAG                     (0x8000)

/****************************************************************************************/
/*                                                                                      */
/*                               Mistakes codes for MCU                                 */
/*                                                                                      */
/****************************************************************************************/

// @todo Add proper descriptions to all mistakes.

/*! Debugging related mistakes */
#define MCU_ERROR_LOG_OVERFLOW_Err              (1U + MCU_NAMESPACE_OFFSET)    //!< Tolerable mistake.

/*! Clock setup related errors */
#define MCU_PLL_FAILED_Err                      (2U + MCU_NAMESPACE_OFFSET + CRITICAL_ERROR_FLAG) //!< System will most likely know the wrong working frequency.
#define MCU_PLL_WRONG_FREQUENCY_SOURCE_Err      (3U + MCU_NAMESPACE_OFFSET)
#define MCU_FAILED_TO_LAUNCH_HSE_Err            (4U + MCU_NAMESPACE_OFFSET)
#define MCU_HSE_FAILED_WHILE_RUNNING_Err        (5U + MCU_NAMESPACE_OFFSET)

/*! Communications related mistakes */
#define MCU_SPI2_TRANSMISSION_FAILED_Err        (10U + MCU_NAMESPACE_OFFSET + CRITICAL_ERROR_FLAG)   //!< This means that SPI wasn't set up or was disabled.
#define MCU_SPI2_BAUD_RATE_SETUP_FAILED_Err     (11U + MCU_NAMESPACE_OFFSET)   //!< This means that the desired SPI frequency is very small, and the system frequency is too high.
#define MCU_SPI3_TRANSMISSION_FAILED_Err        (12U + MCU_NAMESPACE_OFFSET + CRITICAL_ERROR_FLAG)   //!< This means that SPI wasn't set up or was disabled.
#define MCU_SPI3_BAUD_RATE_SETUP_FAILED_Err     (13U + MCU_NAMESPACE_OFFSET)   //!< This means that the desired SPI frequency is very small, and the system frequency is too high.


#endif /* ITF_ERROR_CODES_H_ */
