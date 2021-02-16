/*!
 * @file basic_tests.c
 */

#include "basic_tests.h"

void run_motors_triangle_movement_test( motor *m1, motor *m2 )
{
    for ( int32_t i = 0; i <= PWM_PRECISION; ++i )
    {
        m1->set_pwm_duty_cycle(i);
        m2->set_pwm_duty_cycle(i);
        dummy_delay(10000);
    }
    for (int32_t i = PWM_PRECISION; i >= -PWM_PRECISION; --i)
    {
        m1->set_pwm_duty_cycle(i);
        m2->set_pwm_duty_cycle(i);
        dummy_delay(10000);
    }
    for ( int32_t i = -PWM_PRECISION; i <= 0; ++i )
    {
        m1->set_pwm_duty_cycle(i);
        m2->set_pwm_duty_cycle(i);
        dummy_delay(10000);
    }
}

