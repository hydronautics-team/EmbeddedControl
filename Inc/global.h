#ifndef GLOBAL_H
#define GLOBAL_H

#include <stdint.h>
#include <stdbool.h>

#include "robot.h"
#include "messages.h"

extern struct Robot Q100;

extern uint8_t ShoreRequestBuffer[REQUEST_CONFIG_LENGTH];
extern uint8_t ShoreResponseBuffer[SHORE_CONFIG_RESPONSE_LENGTH];

extern uint8_t ImuRequestBuffer[IMU_REQUEST_LENGTH];
extern uint8_t ImuResetRequestBuffer[IMU_REQUEST_LENGTH];
extern uint8_t ImuResponseBuffer[IMU_RESPONSE_LENGTH*IMU_CHECKSUMS];

extern uint8_t ThrustersRequestBuffer[THRUSTERS_REQUEST_LENGTH];
extern uint8_t ThrustersResponseBuffer[THRUSTERS_NUMBER][THRUSTERS_RESPONSE_LENGTH];

extern uint8_t DevicesRequestBuffer[DEVICES_REQUEST_LENGTH];
extern uint8_t DevicesResponseBuffer[DEVICES_NUMBER][DEVICES_RESPONSE_LENGTH];

extern uint8_t PressureResponseBuffer[PRESSURE_SENSOR_SIZE];

extern uint8_t ContourSelected;

#endif
