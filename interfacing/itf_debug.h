/*!
 * @file itf_debug.h
 */

#ifndef ITF_DEBUG_H_
#define ITF_DEBUG_H_

#include "interfacing.h"


//! Makes misteakes_log entry with given mistake code at the current time if time logging is enabled.
uint32_t add_mistake_to_the_log(uint32_t mistake_code);

#endif /* ITF_DEBUG_H_ */
