/*!
 * @file icm20600_error_codes.h
 *
 * @brief This file contains all error codes for the ICM-20600 interfacing independent library.
 *
 * All error codes are divided into tolerable and critical. Tolerable errors can be fully handled by the system and just warn users
 *  about occurred problems. Critical mistakes can lead to unknown behavior of the system, which can cause harm and must be handled properly.
 *
 * @note Error definitions have _Err suffix to make them distinguishable from other types of definitions.
 * @note Critical error handling behavior is not specified in this file.
 *
 * Error codes are stored as uint16_t variables. The first 15 bits are used as an error code number representation, meaning
 *  that there are 2^15-1 = 32767 available error codes. The most significant bit (MSb) is used as a critical error flag.
 *
 * All error codes are documented in the ICM20600_errors.md file. (not ready yet).
 *
 * @todo Add Errors.md file with all needed descriptions and clean the comments on the error codes.
 * @todo Test the ability to really change MUC_NAMESPACE_OFFSET outside this file.
 */

#ifndef ICM20600_ERROR_CODES_H_
#define ICM20600_ERROR_CODES_H_

/**
 * @brief defines indent for ICM20600 mistake codes.
 *
 * @note Can be re-defined at any point in the interfacing.h file.
 */
#ifndef ICM20600_NAMESPACE_OFFSET
    #define ICM20600_NAMESPACE_OFFSET    (500)
#endif

//! Define that is used to set the sixteenth bit of the error code, representing a critical error.
#define CRITICAL_ERROR_FLAG                     (0x8000)

/****************************************************************************************/
/*                                                                                      */
/*                            Mistakes codes for ICM20600                               */
/*                                                                                      */
/****************************************************************************************/

/*! Connection related mistakes codes */
#define ICM20600_WAS_NOT_INITIALIZED_Err                (1 + ICM20600_NAMESPACE_OFFSET + CRITICAL_ERROR_FLAG)
#define ICM20600_IS_NOT_CONNECTED_Err                   (2 + ICM20600_NAMESPACE_OFFSET + CRITICAL_ERROR_FLAG)
#define ICM20600_SPI_WRITE_FUNCTION_ABSENT_Err          (3 + ICM20600_NAMESPACE_OFFSET + CRITICAL_ERROR_FLAG)
#define ICM20600_SPI_CS_TOGGLE_FUNCTIONS_ABSENT_Err     (4 + ICM20600_NAMESPACE_OFFSET + CRITICAL_ERROR_FLAG)

#define ICM20600_WRONG_GYRO_SCALE_Err                   (5 + ICM20600_NAMESPACE_OFFSET)
#define ICM20600_WRONG_ACCEL_SCALE_Err                  (6 + ICM20600_NAMESPACE_OFFSET)
#define ICM20600_WRONG_GYRO_AVERAGING_Err               (7 + ICM20600_NAMESPACE_OFFSET)
#define ICM20600_WRONG_ACCEL_AVERAGING_Err              (8 + ICM20600_NAMESPACE_OFFSET)
#define ICM20600_WRONG_GYRO_FILTER_BW_Err               (9 + ICM20600_NAMESPACE_OFFSET)
#define ICM20600_WRONG_ACCEL_FILTER_BW_Err              (10 + ICM20600_NAMESPACE_OFFSET)
#define ICM20600_WRONG_GYRO_INDEX_Err                   (11 + ICM20600_NAMESPACE_OFFSET)
#define ICM20600_WRONG_ACCEL_INDEX_Err                  (12 + ICM20600_NAMESPACE_OFFSET)
#define ICM20600_WRONG_SAMPLE_RATE_DIVIDER_Err          (13 + ICM20600_NAMESPACE_OFFSET)

#endif /* ICM20600_ERROR_CODES_H_ */
