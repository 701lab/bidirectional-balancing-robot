/*!
 * @file devive.c
 *
 * Contains implementations of functions from device.h
 */

#include "device.h"



/*!
 * @brief Sets up all project related peripherals.
 */
void init_device(void)
{
    // Enable FPU.
    SCB->CPACR |= 0x3 << 20 ;

    /*! Every single time related parameter is based on SYSTEM_MAIN_FREQUENCY, thus clock be set up a soon as possible. */
    setup_system_clock();

    setup_timers();

    setup_leds();

    // init all the other stuff

//    setup_motor1();
//    setup_motor2();


    /*
     * System timer should be set up last, so that control loop won't start working before MCU setup is finished.
     *
     * @todo In future this setup should be called after every other device (TMC5130 and encoder) were set up.
     *        Because otherwise it is possible that control loop will ask for data before setup was finished.
     */
    setup_system_timer();

    // @todo Add some kind of control loop zeroing.
    // @todo write the code to init slave devices.

}


