/**
 * Radio module
 *
 * This module provides all the required functions to manage the nRF51822
 * transceiver.
 **/

#pragma once

#include "MicroBit.h"

extern uint8_t rx_buffer[254];                     /* Rx buffer used by RF to store packets. */
extern uint8_t tx_buffer[254];

void radio_disable(void);
uint8_t channel_to_freq(int channel);
void radio_set_sniff(int channel);
void radio_sniff_aa(uint32_t access_address, int channel);
void radio_follow_aa(uint32_t accessAddress, int channel, uint32_t crcInit);
void radio_follow_conn(uint32_t accessAddress, int channel, uint32_t crcInit);
void radio_set_channel_fast(int channel);
void radio_send(uint8_t *pBuffer, int size);
void radio_send_rx(uint8_t *pBuffer, int size, int channel);
void radio_receive(void);
void radio_jam_advertisements(uint8_t *pattern,int size, int offset,int channel);
