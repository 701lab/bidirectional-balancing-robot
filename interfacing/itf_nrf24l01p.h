/*
 * @file itf_nrf24l01p.h
 */

#ifndef ITF_NRF24L01P_H_
#define ITF_NRF24L01P_H_

#include "interfacing.h"
#include "itf_debug.h"

void setup_nrf24l01p_peripherals( void );

//! Transmits and receives a single byte via SPI3.
uint8_t spi3_write_single_byte(const uint8_t byte_to_send);

void nrf24_set_cs_high( void );
void nrf24_set_cs_low( void );

void nrf24_set_ce_high( void );
void nrf24_set_ce_low( void );


#endif /* ITF_NRF24L01P_H_ */
