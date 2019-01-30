#include "global.h"

struct Robot Q100;

uint8_t ShoreRequestBuffer[REQUEST_CONFIG_LENGTH];
uint8_t ShoreResponseBuffer[SHORE_RESPONSE_LENGTH];

uint8_t ImuRequestBuffer[IMU_REQUEST_LENGTH] = { 's', 'n', 'p', 0x00, 0xAE, 0x01, 0xFF };
uint8_t ImuResetRequestBuffer[IMU_REQUEST_LENGTH] = { 's', 'n', 'p', 0x00, 0xAC, 0x01, 0xFD };
uint8_t ImuResponseBuffer[IMU_RESPONSE_LENGTH*IMU_CHECKSUMS];

uint8_t ThrustersRequestBuffer[THRUSTERS_REQUEST_LENGTH];
uint8_t ThrustersResponseBuffer[THRUSTERS_NUMBER][THRUSTERS_RESPONSE_LENGTH];

uint8_t DevicesRequestBuffer[DEVICES_REQUEST_LENGTH];
uint8_t DevicesResponseBuffer[DEVICES_NUMBER][DEVICES_RESPONSE_LENGTH];

uint8_t PressureResponseBuffer[PRESSURE_SENSOR_SIZE];
