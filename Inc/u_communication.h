#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include "u_robot.h"

enum{
	SHORE_UART = 1,
	VMA_UART,
	DEV_UART,
	IMU_UART
};

// Custom UART DMA receive/transmit functions
void variableInit(void);

void receiveByte(uint8_t UART, uint8_t *byte);
void receivePackageDMA(uint8_t UART, uint8_t *buf, uint8_t length);
void transmitPackageDMA(uint8_t UART, uint8_t *buf, uint8_t length);

float FloatFromUint8(uint8_t *buff, uint8_t high_byte_pos);

void DevRequestUpdate(struct Robot *robot, uint8_t *buf, uint8_t dev);
void DevResponseUpdate(struct Robot *robot, uint8_t *buf, uint8_t dev);

void VMARequestUpdate(struct Robot *robot, uint8_t *buf, uint8_t vma);
void VMAResponseUpdate(struct Robot *robot, uint8_t *buf, uint8_t vma);

void ShoreRequest(struct Robot *robot, uint8_t *requestBuf, int16_t *pitchError, int16_t *rollError);
void ShoreConfigRequest(struct Robot *robot, uint8_t *requestBuf);
void ShoreResponse(struct Robot *robot, uint8_t *responseBuf);

void IMUReceive(struct Robot *robot, uint8_t *IMUReceiveBuf, uint8_t *ErrCode);
void IMUReset(void);
void CompChecksum(uint8_t *upbyte, uint8_t *lowbyte, uint8_t *msg, uint8_t size);

void BTReset(void);
void BTRequest(uint8_t *ReceiveBuf);
void BTReceive(struct Robot *robot, uint8_t *ReceiveBuf, uint8_t *ErrCode);

#endif
