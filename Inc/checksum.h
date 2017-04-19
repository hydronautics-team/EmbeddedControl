#ifndef CHECKSUM_H
#define CHECKSUM_H

#include <stdint.h>
#include <stdbool.h>

bool isChecksumm16bCorrect(uint8_t *msg, uint16_t length);
void addChecksumm16b(uint8_t *msg, uint16_t length);

bool isChecksumm8bCorrect(uint8_t *msg, uint16_t length);
void addChecksumm8b(uint8_t *msg, uint16_t length);

uint16_t IMUchecksum(uint8_t *arr, uint16_t length);

#endif
