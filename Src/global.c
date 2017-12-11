#include "global.h"

struct Robot Q100;

uint8_t shorePackageError = 0;

uint8_t ShoreRequestBuf[SHORE_REQUEST_LENGTH];
uint8_t ShoreRequestConfigBuf[REQUEST_CONFIG_LENGTH];
uint8_t ShoreResponseBuf[SHORE_RESPONSE_LENGTH];

uint8_t IMURequestBuf[IMU_REQUEST_LENGTH] = { 's', 'n', 'p', 0x00, 0xAE, 0x01, 0xFF };;
uint8_t IMUResponseBuf[IMU_RESPONSE_LENGTH*IMU_CHECKSUMS*2];

uint8_t BTReceiveBuf[BT_SIZE];
uint8_t BTCalls = 0;

uint8_t VMARequestBuf[VMA_DEV_REQUEST_LENGTH];
uint8_t VMAResponseBuf[VMA_DRIVER_NUMBER][VMA_DEV_RESPONSE_LENGTH];

uint8_t DevRequestBuf[VMA_DEV_REQUEST_LENGTH];
uint8_t DevResponseBuf[DEV_DRIVER_NUMBER][VMA_DEV_RESPONSE_LENGTH];
