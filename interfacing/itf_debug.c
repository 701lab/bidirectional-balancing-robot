/*!
 * @file itf_debug.c
 */

#include "itf_debug.h"

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
 * @todo Make the file interfacing independent. Change exact timer name to define. Add checks for undefined defines and write instructions
 *      on how to use debug features.
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
        mistakes_log[MISTAKES_LOG_SIZE].mistake_time_in_milliseconds = CURRENT_MILLISECOND_Val;

#endif /* MISTAKE_LOG_SHOULD_SAVE_TIME */

    }

    // Add mistake to the log.
    mistakes_log[mistakes_log_pointer].mistake_code = (mistake_code & 0x7F); // to hide critical mistake flag. Critical mistakes should must be handled separately.

    // Add logging time to the log if enabled.
#ifdef MISTAKE_LOG_SHOULD_SAVE_TIME

    mistakes_log[mistakes_log_pointer].mistake_time_in_seconds = seconds_from_setup;
    mistakes_log[mistakes_log_pointer].mistake_time_in_milliseconds = CURRENT_MILLISECOND_Val;

#endif /* MISTAKE_LOG_SHOULD_SAVE_TIME */

    ++mistakes_log_pointer;


    // Return mistake code for further use.
    return mistake_code;
}

