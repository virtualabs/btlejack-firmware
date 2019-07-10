#pragma once

#include "MicroBit.h"

uint32_t btle_reverse_crc(uint32_t crc, uint8_t *data, int len);
uint8_t swap_bits(uint8_t b);
void dewhiten(uint8_t *data, int len, int channel);
int is_valid_aa(uint32_t aa);
void chm_to_array(uint8_t *chm, uint8_t *chmArray);
void array_to_chm(uint8_t *chmArray, uint8_t *chm);
uint32_t whiten_pattern(uint8_t *pattern,int size, int offset,int channel);
