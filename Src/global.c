#include "global.h"

struct Robot Q100;

enum ImuErrcodes {
	IMU_FIND_ERROR=1,
	IMU_BAD_CHECKSUM_ERROR
};

uint8_t ShoreRequestBuf[SHORE_REQUEST_LENGTH];
uint8_t ShoreRequestConfigBuf[REQUEST_CONFIG_LENGTH];
uint8_t ShoreResponseBuf[SHORE_RESPONSE_LENGTH];

uint8_t IMU_Receive[IMU_RECEIVE_PACKET_SIZE*5*2];

uint8_t VMARequestBuf[VMA_DEV_REQUEST_LENGTH];
uint8_t VMAResponseBuf[VMA_DEV_RESPONSE_LENGTH];

uint8_t DevRequestBuf[VMA_DEV_REQUEST_LENGTH];
uint8_t DevResponseBuf[VMA_DEV_RESPONSE_LENGTH];
