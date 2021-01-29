/*!
 * @file itf_nrf24l01p.c
 *
 * Robots board v2.0 uses STM32G474RBT6 MCU.
 *  PCB contains NRF24L01+ radio module.
 *
 * NRF24l01+:
 *  PC10-PC12 - SCK, MISO, MOSI respectively (SPI3, alternate function 6);
 *  PD2 - NRF_CS (digital output) NRF24L01+ chip selected;
 *  PB3 - NRF_CE (digital output) NRF24L01+ chip enable;
 *  PA15 - NRF_IRQ (external interrupt input) NRF24L01+ interrupt request.
 */

#include "itf_nrf24l01p.h"

