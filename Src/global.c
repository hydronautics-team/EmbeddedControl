#include "global.h"

struct Robot Q100;

uint8_t shorePackageError = 0;

uint8_t ShoreRequestBuf[SHORE_REQUEST_LENGTH];
uint8_t ShoreRequestConfigBuf[REQUEST_CONFIG_LENGTH];
uint8_t ShoreResponseBuf[SHORE_RESPONSE_LENGTH];

uint8_t ImuRequestBuf[IMU_REQUEST_LENGTH] = { 's', 'n', 'p', 0x00, 0xAE, 0x01, 0xFF };
uint8_t ImuResetRequestBuf[IMU_REQUEST_LENGTH] = { 's', 'n', 'p', 0x00, 0xAC, 0x01, 0xFD };
uint8_t ImuResponseBuf[IMU_RESPONSE_LENGTH*IMU_CHECKSUMS];

uint8_t VmaRequestBuf[VMA_REQUEST_LENGTH];
uint8_t VmaResponseBuf[VMA_DRIVER_NUMBER][VMA_RESPONSE_LENGTH];

uint8_t DevRequestBuf[DEV_REQUEST_LENGTH];
uint8_t DevResponseBuf[DEV_DRIVER_NUMBER][DEV_RESPONSE_LENGTH];

uint8_t SensorsStartMeasurementBuf[SENSORS_START_REQUEST_LENGTH];
uint8_t SensorsReliseMeasurementByte;
uint8_t SensorsResponseBuf[SENSORS_DEVICES_NUM][SENSORS_PACKAGE_SIZE];
