#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <stdint.h>
#include <stdlib.h>
#include "main.h"
#include "robot.h"
#include "checksum.h"
#include "messages.h"


float FloatFromUint8(uint8_t *buff, uint8_t high_byte_pos);

void DevRequestUpdate(struct Robot *robot, uint8_t *buf, uint8_t dev);
void DevResponseUpdate(struct Robot *robot, uint8_t *buf, uint8_t dev);

void VMARequestUpdate(struct Robot *robot, uint8_t *buf, uint8_t vma);
void VMAResponseUpdate(struct Robot *robot, uint8_t *buf, uint8_t vma);

void ShoreRequest(struct Robot *robot, uint8_t *requestBuf);
void ShoreConfigRequest(struct Robot *robot, uint8_t *requestBuf);
void ShoreResponse(struct Robot *robot, uint8_t *responseBuf);

void IMUResponse(struct Robot *robot, uint8_t *IMUResponseBuf);
void IMUReset(void);



#endif
