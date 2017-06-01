#include "global.h"

struct Robot Q100;

uint8_t shorePackageError = 0;

uint8_t ShoreRequestBuf[SHORE_REQUEST_LENGTH];
uint8_t ShoreRequestConfigBuf[REQUEST_CONFIG_LENGTH];
uint8_t ShoreResponseBuf[SHORE_RESPONSE_LENGTH];

uint8_t IMUReceiveBuf[IMU_RECEIVE_PACKET_SIZE*5*4];

uint8_t BTReceiveBuf[BT_SIZE];
uint8_t BTCalls=0;

uint8_t VMARequestBuf[VMA_DEV_REQUEST_LENGTH];
uint8_t VMAResponseBuf[VMA_DEV_RESPONSE_LENGTH];

uint8_t DevRequestBuf[VMA_DEV_REQUEST_LENGTH];
uint8_t DevResponseBuf[VMA_DEV_RESPONSE_LENGTH];
