#include "global.h"

struct Robot Q100;


uint8_t ShoreRequestBuf[SHORE_REQUEST_LENGTH];
uint8_t ShoreRequestConfigBuf[REQUEST_CONFIG_LENGTH];
uint8_t ShoreResponseBuf[SHORE_RESPONSE_LENGTH];

uint8_t IMURequestBuf[IMU_REQUEST_LENGTH];
uint8_t IMUResponseBuf[IMU_RESPONSE_LENGTH];

uint8_t VMARequestBuf[VMA_DEV_REQUEST_LENGTH];
uint8_t VMAResponseBuf[VMA_DEV_RESPONSE_LENGTH];

uint8_t DevRequestBuf[VMA_DEV_REQUEST_LENGTH];
uint8_t DevResponseBuf[VMA_DEV_RESPONSE_LENGTH];
