#ifndef GLOBAL_H
#define GLOBAL_H

#include <stdint.h>
#include <stdbool.h>

#include "robot.h"
#include "messages.h"

extern struct Robot Q100;
/*
extern bool shore_RX_enable;
extern bool shore_TX_enable;
extern bool VMA_RX_enable;
extern bool VMA_TX_enable;
extern bool DEV_RX_enable;
extern bool DEV_TX_enable;

extern bool VMA_RX_enable;
extern bool VMA_TX_enable;
extern bool DEV_RX_enable;
extern bool DEV_TX_enable;
*/
extern uint8_t shorePackageError;

extern uint8_t ShoreRequestBuf[SHORE_REQUEST_LENGTH];
extern uint8_t ShoreRequestConfigBuf[REQUEST_CONFIG_LENGTH];
extern uint8_t ShoreResponseBuf[SHORE_RESPONSE_LENGTH];

extern uint8_t ImuRequestBuf[IMU_REQUEST_LENGTH];
extern uint8_t ImuResetRequestBuf[IMU_REQUEST_LENGTH];
extern uint8_t ImuResponseBuf[IMU_RESPONSE_LENGTH*IMU_CHECKSUMS];

extern uint8_t VmaRequestBuf[VMA_REQUEST_LENGTH];
extern uint8_t VmaResponseBuf[VMA_DRIVER_NUMBER][VMA_RESPONSE_LENGTH];

extern uint8_t DevRequestBuf[DEV_REQUEST_LENGTH];
extern uint8_t DevResponseBuf[DEV_DRIVER_NUMBER][DEV_RESPONSE_LENGTH];

extern uint8_t SensorsResponseBuf[SENSORS_DEVICES_NUM][SENSORS_PACKAGE_SIZE];

#endif
