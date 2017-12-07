#ifndef CHECKSUM_H
#define CHECKSUM_H

#include <stdint.h>
#include <stdbool.h>
#include "main.h"

bool IsChecksumm16bCorrect(uint8_t *msg, uint16_t length);
void AddChecksumm16b(uint8_t *msg, uint16_t length);

bool IsChecksumm8bCorrect(uint8_t *msg, uint16_t length);
void AddChecksumm8b(uint8_t *msg, uint16_t length);

uint16_t IMUchecksum(uint8_t *arr, uint16_t length);

#endif
