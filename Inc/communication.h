#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include "robot.h"

#define UART_NUMBER 4

enum {
	SHORE_UART = 0,
	VMA_UART,
	DEV_UART,
	IMU_UART
};

enum {
	DEV_I2C = 0,
	PC_I2C
};

enum ImuErrcodes {
    IMU_FIND_ERROR=1,
    IMU_BAD_CHECKSUM_ERROR
};

enum BTErrCodes {
    BT_ERROR_RECEIVED_NOTHING=1,
    BT_ERROR_RECEIVED_LESS
};

extern bool uartPackageReceived[UART_NUMBER];
extern uint8_t RxBuffer;
extern uint16_t numberRx;
extern uint16_t counterRx;

// Custom UART DMA receive/transmit functions
void variableInit(void);

void receivePackage(uint8_t UART, uint8_t *buf, uint8_t length);
void transmitPackage(uint8_t UART, uint8_t *buf, uint8_t length);
void transmitAndReceive(uint8_t UART, uint8_t *tr_buf, uint8_t tr_length, uint8_t *re_buf, uint8_t re_length);
void receiveI2cPackageDMA (uint8_t I2C, uint16_t addr, uint8_t *buf, uint8_t length);
void transmitI2cPackageDMA(uint8_t I2C, uint16_t addr, uint8_t *buf, uint8_t length);

void DevRequestUpdate(struct Robot *robot, uint8_t *buf, uint8_t dev);
void DevResponseUpdate(struct Robot *robot, uint8_t *buf, uint8_t dev);

void VmaRequestUpdate(struct Robot *robot, uint8_t *buf, uint8_t vma);
void VmaResponseUpdate(struct Robot *robot, uint8_t *buf, uint8_t vma);

void ShoreReceive();
void ShoreRequest(struct Robot *robot, uint8_t *requestBuf);
void ShoreConfigRequest(struct Robot *robot, uint8_t *requestBuf);
void ShoreResponse(struct Robot *robot, uint8_t *responseBuf);

void ImuReceive(struct Robot *robot, uint8_t *IMUReceiveBuf, uint8_t *ErrCode);

void SensorsResponseUpdate(struct Robot *robot, uint8_t *buf, uint8_t Sensor_id);

#endif
