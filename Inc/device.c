/*!
 * @file devive.c
 *
 * Contains implementations of functions from device.h
 */

#include "device.h"

/*!
 * @brief Sets up all project related peripherals.
 */
void setup_device(void)
{
    // Enable FPU.
    SCB->CPACR |= 0x3 << 20 ;

    /*! Every single time related parameter is based on SYSTEM_MAIN_FREQUENCY, thus clock be set up a soon as possible. */
    setup_system_clock();

    setup_timers();

    setup_leds();

    setup_motor1();
    setup_motor2();

    setup_nrf24l01p_peripherals();
    setup_icm20600_peripherals();

    dummy_delay(200000);

//    setup_system_timer();

    // @todo Add some kind of control loop zeroing.
    // @todo write the code to init slave devices.
}


void dummy_delay(uint32_t duration)
{
    for(uint32_t i = 0; i < duration; ++i){}
}
