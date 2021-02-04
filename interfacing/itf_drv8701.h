/*!
 * @file itf_drv8701.h
 */

#ifndef ITF_DRV8701_H_
#define ITF_DRV8701_H_

#include "interfacing.h"
#include "itf_debug.h"

void setup_motor1( void );
void setup_motor2( void );
void setup_encoder1( void );
void setup_encoder2( void );


inline int16_t get_encoder1_value(void);
inline int16_t get_encoder2_value(void);


#endif /* ITF_DRV8701_H_ */
