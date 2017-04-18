#ifndef CHECKSUM_H
#define CHECKSUM_H

#include <stdint.h>

uint8_t easyCheckSumm(uint8_t *msg, uint16_t length);

uint8_t isChecksumm16bCorrect(uint8_t *msg, uint16_t length);
void addChecksumm16b(uint8_t *msg, uint16_t length);

uint8_t isCheckSumm8bCorrect(uint8_t *msg, uint16_t length);
void addCheckSumm8b(uint8_t *msg, uint16_t length);

uint16_t IMUchecksum(uint8_t* arr, uint8_t size);


#endif
