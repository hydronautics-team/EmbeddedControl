#include <stdint.h>
#include <stdlib.h>
#include "cmsis_os.h"
#include "usart.h"
#include "i2c.h"
#include "main.h"
#include "tim.h"

#include "communication.h"
#include "global.h"
#include "messages.h"
#include "checksum.h"

#define PACKAGE_TOLLERANCE 20

enum ImuErrcodes {
	IMU_FIND_ERROR=1,
	IMU_BAD_CHECKSUM_ERROR
};

enum BTErrCodes {
	BT_ERROR_RECEIVED_NOTHING=1,
	BT_ERROR_RECEIVED_LESS
};

// GOVNO S UARTA NACHALO

extern TimerHandle_t UARTTimer;
extern osMutexId mutDataHandle;

#define TASK_WAITING	5
#define SHORE_WAITING 15
#define DELAY	1

uint8_t RxBuffer[1] = {0};
uint16_t numberRx = 0;
uint16_t counterRx = 0;

bool uart1PackageTransmit = false;
bool uart2PackageTransmit = false;
bool uart3PackageTransmit = false;
bool uart4PackageTransmit = false;

bool uart1PackageReceived = false;
bool uart2PackageReceived = false;
bool uart3PackageReceived = false;
bool uart4PackageReceived = false;

void variableInit() {
	Q100.VMA[HLB].address = 0x07;
	Q100.VMA[HLF].address = 0x05;
	Q100.VMA[HRB].address = 0x02;
	Q100.VMA[HRF].address = 0x04;
	Q100.VMA[VB].address = 0x03;
	Q100.VMA[VF].address = 0x06;
	Q100.VMA[VL].address = 0x013;
	Q100.VMA[VR].address = 0x08;
	
	Q100.device.agar.address = 0x03;
	Q100.device.grab.squeezeAddress = 0x01;
	Q100.device.grab.rotationAddress = 0x02;
	Q100.device.tilt.address = 0x04;
	
	Q100.pitchStabilization.enable = true;
	Q100.pitchStabilization.iPartEnable = false;
	Q100.pitchStabilization.pid_pGain = 1;
	Q100.pitchStabilization.pid_iGain = 1;
	Q100.pitchStabilization.pid_iMin = 0;
	Q100.pitchStabilization.pid_iMax = 0;
	Q100.pitchStabilization.positionFeedbackCoef = 32768;
	Q100.pitchStabilization.speedFeedbackCoef = 32768;
	
	Q100.rollStabilization.enable = true;
	Q100.rollStabilization.iPartEnable = false;
	Q100.rollStabilization.pid_pGain = 1;
	Q100.rollStabilization.pid_iGain = 1;
	Q100.rollStabilization.pid_iMin = 0;
	Q100.rollStabilization.pid_iMax = 0;
	Q100.rollStabilization.positionFeedbackCoef = 32768;
	Q100.rollStabilization.speedFeedbackCoef = 32768;
	
	Q100.yawStabilization.enable = true;
	Q100.yawStabilization.iPartEnable = false;
	Q100.yawStabilization.pid_pGain = 1;
	Q100.yawStabilization.pid_iGain = 1;
	Q100.yawStabilization.pid_iMin = 0;
	Q100.yawStabilization.pid_iMax = 0;
	Q100.yawStabilization.positionFeedbackCoef = 32768;
	Q100.yawStabilization.speedFeedbackCoef = 32768;
}

void transmitPackageDMA(uint8_t UART, uint8_t *buf, uint8_t length) {
	TickType_t timeBegin = xTaskGetTickCount();
	switch(UART) {
		case SHORE_UART:
			HAL_HalfDuplex_EnableTransmitter(&huart1);
			HAL_UART_Transmit_DMA(&huart1, buf, length);
			while (!uart1PackageTransmit && xTaskGetTickCount() - timeBegin < SHORE_WAITING) {
				osDelay(DELAY);
			}
			uart1PackageTransmit = false;
			break;
		case VMA_UART:
			HAL_HalfDuplex_EnableTransmitter(&huart2);
			HAL_UART_Transmit_DMA(&huart2, buf, length);
			while (!uart2PackageTransmit && xTaskGetTickCount() - timeBegin < TASK_WAITING) {
				osDelay(DELAY);
			}
			uart2PackageTransmit = false;
			break;
		case DEV_UART:
			HAL_HalfDuplex_EnableTransmitter(&huart3);
			HAL_UART_Transmit_DMA(&huart3, buf, length);
			while (!uart3PackageTransmit && xTaskGetTickCount() - timeBegin < TASK_WAITING) {
				osDelay(DELAY);
			}
			uart3PackageTransmit = false;
			break;
		case IMU_UART:
			HAL_UART_Transmit_DMA(&huart4, buf, length);
			while (!uart4PackageTransmit && xTaskGetTickCount() - timeBegin < TASK_WAITING) {
				osDelay(DELAY);
			}
			uart4PackageTransmit = false;
			break;
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart1) {
		uart1PackageTransmit = true;
	}
	else if(huart == &huart2) {
		uart2PackageTransmit = true;
	}
	else if(huart == &huart3) {
		uart3PackageTransmit = true;
	}
	else if(huart == &huart4) {
		uart4PackageTransmit = true;
	}
}

void receiveByte(uint8_t UART, uint8_t *byte)
{
	TickType_t timeBegin = xTaskGetTickCount();
	switch(UART){
		case SHORE_UART:
			HAL_HalfDuplex_EnableReceiver(&huart1);
			HAL_UART_Receive_IT(&huart1, byte, 1);
			while (!uart1PackageReceived && xTaskGetTickCount() - timeBegin < 1) {
				//delayUs(DELAY);
				osDelay(DELAY);
			}
			uart1PackageReceived = false;
			break;
		case VMA_UART:
			HAL_HalfDuplex_EnableReceiver(&huart2);
			HAL_UART_Receive_DMA(&huart2, byte, 1);
			while (!uart2PackageReceived && xTaskGetTickCount() - timeBegin < 1) {
				//delayUs(DELAY);
				osDelay(DELAY);
			}
			uart2PackageReceived = false;
			break;
		case DEV_UART:
			HAL_HalfDuplex_EnableReceiver(&huart3);
			HAL_UART_Receive_DMA(&huart3, byte, 1);
			while (!uart3PackageReceived && xTaskGetTickCount() - timeBegin < 1) {
				//delayUs(DELAY);
				osDelay(DELAY);
			}
			uart3PackageReceived = false;
			break;
		case IMU_UART:
			HAL_HalfDuplex_EnableReceiver(&huart4);
			HAL_UART_Receive_DMA(&huart4, byte, 1);
			while (!uart4PackageReceived && xTaskGetTickCount() - timeBegin < 1) {
				//delayUs(DELAY);
				osDelay(DELAY);
			}
			uart4PackageReceived = false;
			break;
	}
}

void receivePackageDMA(uint8_t UART, uint8_t *buf, uint8_t length)
{
	TickType_t timeBegin = xTaskGetTickCount();
	switch(UART){
		case SHORE_UART:
			HAL_HalfDuplex_EnableReceiver(&huart1);
			HAL_UART_Receive_DMA(&huart1, buf, length);
			while (!uart1PackageReceived && xTaskGetTickCount() - timeBegin < SHORE_WAITING){
				osDelay(DELAY);
			}
			uart1PackageReceived = false;
			break;
		case VMA_UART:
			HAL_HalfDuplex_EnableReceiver(&huart2);
			HAL_UART_Receive_DMA(&huart2, buf, length);
			while (!uart2PackageReceived  && xTaskGetTickCount() - timeBegin < TASK_WAITING){
				osDelay(DELAY);
			}
			uart2PackageReceived = false;
			break;
		case DEV_UART:
			HAL_HalfDuplex_EnableReceiver(&huart3);
			HAL_UART_Receive_DMA(&huart3, buf, length);
			while (!uart3PackageReceived  && xTaskGetTickCount() - timeBegin < TASK_WAITING){
				osDelay(DELAY);
			}
			uart3PackageReceived = false;
			break;
		case IMU_UART:
			HAL_HalfDuplex_EnableReceiver(&huart4);
			HAL_UART_Receive_DMA(&huart4, buf, length);
			while (!uart4PackageReceived  && xTaskGetTickCount() - timeBegin < TASK_WAITING){
				osDelay(DELAY);
			}
			uart4PackageReceived = false;
			break;
	}
}

void ShoreReceive()
{
	static portBASE_TYPE xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;
	
	if (counterRx == 0) {
		xTimerStartFromISR(UARTTimer, &xHigherPriorityTaskWoken);
		switch(RxBuffer[0]) {
			case SHORE_REQUEST_CODE:
				numberRx = SHORE_REQUEST_LENGTH;
				break;
			case REQUEST_CONFIG_CODE:
				numberRx = REQUEST_CONFIG_LENGTH;
				break;
		}
	}
	
	switch(numberRx) {
		case SHORE_REQUEST_LENGTH:
			ShoreRequestBuf[counterRx] = RxBuffer[0];
			break;
		case REQUEST_CONFIG_LENGTH:
			ShoreRequestConfigBuf[counterRx] = RxBuffer[0];
			break;
	}
	++counterRx;
	
	if (counterRx == numberRx) {
		uart1PackageReceived = true;
		counterRx = 0;
	}
	else {
		HAL_UART_Receive_IT(&huart1, (uint8_t *)RxBuffer, 1);
	}

	if (xHigherPriorityTaskWoken == pdTRUE){
		xHigherPriorityTaskWoken = pdFALSE;
		taskYIELD();
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart1){
		ShoreReceive();
	}
	else if(huart == &huart2){
		uart2PackageReceived = true;
	}
	else if(huart == &huart3){
		uart3PackageReceived = true;
	}
	else if(huart == &huart4){
		uart4PackageReceived = true;
	}
}

void DevRequestUpdate(struct Robot *robot, uint8_t *buf, uint8_t DEV) 
{
	buf[VMA_DEV_REQUEST_AA1] = 0xAA;
	buf[VMA_DEV_REQUEST_AA2] = 0xAA;
	switch(DEV) {
		case AGAR:
			buf[VMA_DEV_REQUEST_ADDRESS] = robot->device.agar.address;
			buf[VMA_DEV_REQUEST_SETTING] = robot->device.agar.settings;
			buf[VMA_DEV_REQUEST_VELOCITY1] = NULL;
			buf[VMA_DEV_REQUEST_VELOCITY2] = robot->device.agar.opening;
			break;
		case GRAB:
			buf[VMA_DEV_REQUEST_ADDRESS] = robot->device.grab.squeezeAddress;
			buf[VMA_DEV_REQUEST_SETTING] = robot->device.grab.settings;
			buf[VMA_DEV_REQUEST_VELOCITY1] = NULL;
			buf[VMA_DEV_REQUEST_VELOCITY2] = robot->device.grab.squeeze;
			break;
		case GRAB_ROTATION:
			buf[VMA_DEV_REQUEST_ADDRESS] = robot->device.grab.rotationAddress;
			buf[VMA_DEV_REQUEST_SETTING] = robot->device.grab.settings;
			buf[VMA_DEV_REQUEST_VELOCITY1] = NULL;
			buf[VMA_DEV_REQUEST_VELOCITY2] = robot->device.grab.rotation;
			break;
		case TILT:
			buf[VMA_DEV_REQUEST_ADDRESS] = robot->device.tilt.address;
			buf[VMA_DEV_REQUEST_SETTING] = robot->device.tilt.settings;
			buf[VMA_DEV_REQUEST_VELOCITY1] = NULL;
			buf[VMA_DEV_REQUEST_VELOCITY2] = robot->device.tilt.rotation;
			break;
	}		
	AddChecksumm8b(buf, VMA_DEV_REQUEST_LENGTH);
}

void writeBit(uint8_t *byte, uint8_t value, uint8_t biteNumb)
{
	if (value == 0){
		*byte &= ~(1 << biteNumb);
	}
	else{
		*byte |= (1 << biteNumb);
	}
}

void DevResponseUpdate(struct Robot *robot, uint8_t *buf, uint8_t dev)
{
	if(IsChecksumm8bCorrect(buf, VMA_DEV_RESPONSE_LENGTH)) {
		switch(dev) {
			case AGAR:
				robot->device.agar.current = (uint16_t)(buf[VMA_DEV_RESPONSE_CURRENT_1H] << 8 | buf[VMA_DEV_RESPONSE_CURRENT_1L]);
				writeBit(&(robot->device.errors), buf[VMA_DEV_RESPONSE_ERRORS], AGAR);
				break;
			case GRAB:
				robot->device.grab.squeezeCurrent = (uint16_t)(buf[VMA_DEV_RESPONSE_CURRENT_1H] << 8 | buf[VMA_DEV_RESPONSE_CURRENT_1L]);
				writeBit(&(robot->device.errors), buf[VMA_DEV_RESPONSE_ERRORS], GRAB);
				break;
			case GRAB_ROTATION:
				robot->device.agar.current = (uint16_t)(buf[VMA_DEV_RESPONSE_CURRENT_1H] << 8 | buf[VMA_DEV_RESPONSE_CURRENT_1L]);
				writeBit(&(robot->device.errors), buf[VMA_DEV_RESPONSE_ERRORS], GRAB_ROTATION);
				break;
			case TILT:
				robot->device.agar.current = (uint16_t)(buf[VMA_DEV_RESPONSE_CURRENT_1H] << 8 | buf[VMA_DEV_RESPONSE_CURRENT_1L]);
				writeBit(&(robot->device.errors), buf[VMA_DEV_RESPONSE_ERRORS], TILT);
				break;
		}
	}
}

void VMARequestUpdate(struct Robot *robot, uint8_t *buf, uint8_t vma)
{
	buf[VMA_DEV_REQUEST_AA1] = 0xAA;
	buf[VMA_DEV_REQUEST_AA2] = 0xAA;
	
	switch(vma) {
		case HLB:
			buf[VMA_DEV_REQUEST_ADDRESS] = robot->VMA[HLB].address;
			buf[VMA_DEV_REQUEST_SETTING] = robot->VMA[HLB].settings;
			buf[VMA_DEV_REQUEST_VELOCITY1] = robot->VMA[HLB].desiredSpeed;
			buf[VMA_DEV_REQUEST_VELOCITY2] = robot->device.light.brightness;
			break;
		
		case HLF:
			buf[VMA_DEV_REQUEST_ADDRESS] = robot->VMA[HLF].address;
			buf[VMA_DEV_REQUEST_SETTING] = robot->VMA[HLF].settings;
			buf[VMA_DEV_REQUEST_VELOCITY1] = robot->VMA[HLF].desiredSpeed;
			buf[VMA_DEV_REQUEST_VELOCITY2] = 0;
			break;
	
		case HRB:
			buf[VMA_DEV_REQUEST_ADDRESS] = robot->VMA[HRB].address;
			buf[VMA_DEV_REQUEST_SETTING] = robot->VMA[HRB].settings;
			buf[VMA_DEV_REQUEST_VELOCITY1] = robot->VMA[HRB].desiredSpeed;
			buf[VMA_DEV_REQUEST_VELOCITY2] = robot->device.bottomLight.brightness;
			break;
		
		case HRF:
			buf[VMA_DEV_REQUEST_ADDRESS] = robot->VMA[HRF].address;
			buf[VMA_DEV_REQUEST_SETTING] = robot->VMA[HRF].settings;
			buf[VMA_DEV_REQUEST_VELOCITY1] = robot->VMA[HRF].desiredSpeed;
			buf[VMA_DEV_REQUEST_VELOCITY2] = NULL;
			break;
		
		case VB:
			buf[VMA_DEV_REQUEST_ADDRESS] = robot->VMA[VB].address;
			buf[VMA_DEV_REQUEST_SETTING] = robot->VMA[VB].settings;
			buf[VMA_DEV_REQUEST_VELOCITY1] = robot->VMA[VB].desiredSpeed;
			buf[VMA_DEV_REQUEST_VELOCITY2] = NULL;
			break;
		
		case VF:
			buf[VMA_DEV_REQUEST_ADDRESS] = robot->VMA[VF].address;
			buf[VMA_DEV_REQUEST_SETTING] = robot->VMA[VF].settings;
			buf[VMA_DEV_REQUEST_VELOCITY1] = robot->VMA[VF].desiredSpeed;
			buf[VMA_DEV_REQUEST_VELOCITY2] = NULL;
			break;
	
		case VL:
			buf[VMA_DEV_REQUEST_ADDRESS] = robot->VMA[VL].address;
			buf[VMA_DEV_REQUEST_SETTING] = robot->VMA[VL].settings;
			buf[VMA_DEV_REQUEST_VELOCITY1] = robot->VMA[VL].desiredSpeed;
			buf[VMA_DEV_REQUEST_VELOCITY2] = NULL;
			break;
	
		case VR:
			buf[VMA_DEV_REQUEST_ADDRESS] = robot->VMA[VR].address;
			buf[VMA_DEV_REQUEST_SETTING] = robot->VMA[VR].settings;
			buf[VMA_DEV_REQUEST_VELOCITY1] = robot->VMA[VR].desiredSpeed;
			buf[VMA_DEV_REQUEST_VELOCITY2] = NULL;
			break;
	}
	AddChecksumm8b(buf, VMA_DEV_REQUEST_LENGTH);
}

void VMAResponseUpdate(struct Robot *robot, uint8_t *buf, uint8_t vma)
{
//TODO errors parsing!
	if(IsChecksumm8bCorrect(buf, VMA_DEV_RESPONSE_LENGTH)) {
		switch(vma) {
			case HLB:
				robot->VMA[HLB].errors = buf[VMA_DEV_RESPONSE_ERRORS];
				robot->VMA[HLB].current = (uint16_t)(buf[VMA_DEV_RESPONSE_CURRENT_1H] << 8 | buf[VMA_DEV_RESPONSE_CURRENT_1L]);
				robot->device.light.current = (uint16_t)(buf[VMA_DEV_RESPONSE_CURRENT_2H] << 8 | buf[VMA_DEV_RESPONSE_CURRENT_2L]);
				robot->VMA[HLB].realSpeed = buf[VMA_DEV_RESPONSE_VELOCITY1];
				break;
		
			case HLF:
				robot->VMA[HLF].errors = buf[VMA_DEV_RESPONSE_ERRORS];
				robot->VMA[HLF].current = (uint16_t)(buf[VMA_DEV_RESPONSE_CURRENT_1H] << 8 | buf[VMA_DEV_RESPONSE_CURRENT_1L]);
				robot->device.bottomLight.current = (uint16_t)(buf[VMA_DEV_RESPONSE_CURRENT_2H] << 8 | buf[VMA_DEV_RESPONSE_CURRENT_2L]);
				robot->VMA[HLF].realSpeed = buf[VMA_DEV_RESPONSE_VELOCITY1];
				break;
			
			case HRB:
				robot->VMA[HRB].errors = buf[VMA_DEV_RESPONSE_ERRORS];
				robot->VMA[HRB].current = (uint16_t)(buf[VMA_DEV_RESPONSE_CURRENT_1H] << 8 | buf[VMA_DEV_RESPONSE_CURRENT_1L]);
				robot->VMA[HRB].realSpeed = buf[VMA_DEV_RESPONSE_VELOCITY1];
				break;
		
			case HRF:
				robot->VMA[HRF].errors = buf[VMA_DEV_RESPONSE_ERRORS];
				robot->VMA[HRF].current = (uint16_t)(buf[VMA_DEV_RESPONSE_CURRENT_1H] << 8 | buf[VMA_DEV_RESPONSE_CURRENT_1L]);
				robot->VMA[HRF].realSpeed = buf[VMA_DEV_RESPONSE_VELOCITY1];
				break;
		
			case VB:
				robot->VMA[VB].errors = buf[VMA_DEV_RESPONSE_ERRORS];
				robot->VMA[VB].current = (uint16_t)(buf[VMA_DEV_RESPONSE_CURRENT_1H] << 8 | buf[VMA_DEV_RESPONSE_CURRENT_1L]);
				robot->VMA[VB].realSpeed = buf[VMA_DEV_RESPONSE_VELOCITY1];
				break;
		
			case VF:
				robot->VMA[VF].errors = buf[VMA_DEV_RESPONSE_ERRORS];
				robot->VMA[VF].current = (uint16_t)(buf[VMA_DEV_RESPONSE_CURRENT_1H] << 8 | buf[VMA_DEV_RESPONSE_CURRENT_1L]);
				robot->VMA[VF].realSpeed = buf[VMA_DEV_RESPONSE_VELOCITY1];
				break;
		
			case VL:
				robot->VMA[VL].errors = buf[VMA_DEV_RESPONSE_ERRORS];
				robot->VMA[VL].current = (uint16_t)(buf[VMA_DEV_RESPONSE_CURRENT_1H] << 8 | buf[VMA_DEV_RESPONSE_CURRENT_1L]);
				robot->VMA[VL].realSpeed = buf[VMA_DEV_RESPONSE_VELOCITY1];
				break;
		
			case VR:
				robot->VMA[VR].errors = buf[VMA_DEV_RESPONSE_ERRORS];
				robot->VMA[VR].current = (uint16_t)(buf[VMA_DEV_RESPONSE_CURRENT_1H] << 8 | buf[VMA_DEV_RESPONSE_CURRENT_1L]);
				robot->VMA[VR].realSpeed = buf[VMA_DEV_RESPONSE_VELOCITY1];
				break;
		}
	}
}


float FloatFromUint8(uint8_t *buff, uint8_t high_byte_pos)
{
  float result;
  result = (float)((buff[high_byte_pos] << 24) | (buff[high_byte_pos + 1] << 16) | (buff[high_byte_pos + 2] << 8) | buff[high_byte_pos + 3]);
  return result;
}


void ShoreConfigRequest(struct Robot *robot, uint8_t *requestBuf)
{
	if(IsChecksumm16bCorrect(requestBuf, SHORE_REQUEST_LENGTH)) {
		robot->depthStabilization.iPartEnable = requestBuf[REQUEST_CONFIG_CONST_TIME_DEPTH];
		robot->depthStabilization.positionFeedbackCoef = FloatFromUint8(requestBuf, REQUEST_CONFIG_K1_DEPTH);
		robot->depthStabilization.speedFeedbackCoef = FloatFromUint8(requestBuf, REQUEST_CONFIG_K2_DEPTH);
		robot->depthStabilization.pid_iMax = FloatFromUint8(requestBuf, REQUEST_CONFIG_START_DEPTH);
		robot->depthStabilization.pid_iMin = -robot->depthStabilization.pid_iMax;
		robot->depthStabilization.pid_pGain = FloatFromUint8(requestBuf, REQUEST_CONFIG_GAIN_DEPTH);
		
		robot->rollStabilization.iPartEnable = requestBuf[REQUEST_CONFIG_CONST_TIME_ROLL];
		robot->rollStabilization.positionFeedbackCoef = FloatFromUint8(requestBuf, REQUEST_CONFIG_K1_ROLL);
		robot->rollStabilization.speedFeedbackCoef = FloatFromUint8(requestBuf, REQUEST_CONFIG_K2_ROLL);
		robot->rollStabilization.pid_iMax = FloatFromUint8(requestBuf, REQUEST_CONFIG_START_ROLL);
		robot->rollStabilization.pid_iMin = -robot->rollStabilization.pid_iMax;
		robot->rollStabilization.pid_pGain = FloatFromUint8(requestBuf, REQUEST_CONFIG_GAIN_ROLL);
		
		robot->pitchStabilization.iPartEnable = requestBuf[REQUEST_CONFIG_CONST_TIME_PITCH];
		robot->pitchStabilization.positionFeedbackCoef = FloatFromUint8(requestBuf, REQUEST_CONFIG_K1_PITCH);
		robot->pitchStabilization.speedFeedbackCoef = FloatFromUint8(requestBuf, REQUEST_CONFIG_K2_PITCH);
		robot->pitchStabilization.pid_iMax = FloatFromUint8(requestBuf, REQUEST_CONFIG_START_PITCH);
		robot->pitchStabilization.pid_iMin = -robot->pitchStabilization.pid_iMax;
		robot->pitchStabilization.pid_pGain = FloatFromUint8(requestBuf, REQUEST_CONFIG_GAIN_PITCH);
		
		robot->yawStabilization.iPartEnable = requestBuf[REQUEST_CONFIG_CONST_TIME_YAW];
		robot->yawStabilization.positionFeedbackCoef = FloatFromUint8(requestBuf, REQUEST_CONFIG_K1_YAW);
		robot->yawStabilization.speedFeedbackCoef = FloatFromUint8(requestBuf, REQUEST_CONFIG_K2_YAW);
		robot->yawStabilization.pid_iMax = FloatFromUint8(requestBuf, REQUEST_CONFIG_START_YAW);
		robot->yawStabilization.pid_iMin = -robot->yawStabilization.pid_iMax;
		robot->yawStabilization.pid_pGain = FloatFromUint8(requestBuf, REQUEST_CONFIG_GAIN_YAW);
	
		robot->VMA[HLB].address = requestBuf[REQUEST_CONFIG_POSITION_HLB];
		robot->VMA[HLF].address = requestBuf[REQUEST_CONFIG_POSITION_HLF];
		robot->VMA[HRB].address = requestBuf[REQUEST_CONFIG_POSITION_HRB];
		robot->VMA[HRF].address = requestBuf[REQUEST_CONFIG_POSITION_HRF];
		robot->VMA[VB].address = requestBuf[REQUEST_CONFIG_POSITION_VB];
		robot->VMA[VF].address = requestBuf[REQUEST_CONFIG_POSITION_VF];
		robot->VMA[VL].address  = requestBuf[REQUEST_CONFIG_POSITION_VL];
		robot->VMA[VR].address = requestBuf[REQUEST_CONFIG_POSITION_VR];
		
		robot->VMA[HLB].settings = requestBuf[REQUEST_CONFIG_SETTING_HLB];
		robot->VMA[HLF].settings = requestBuf[REQUEST_CONFIG_SETTING_HLF];
		robot->VMA[HRB].settings = requestBuf[REQUEST_CONFIG_SETTING_HRB];
		robot->VMA[HRF].settings = requestBuf[REQUEST_CONFIG_SETTING_HRF];
		robot->VMA[VB].settings = requestBuf[REQUEST_CONFIG_SETTING_VB];
		robot->VMA[VF].settings = requestBuf[REQUEST_CONFIG_SETTING_VF];
		robot->VMA[VL].settings = requestBuf[REQUEST_CONFIG_SETTING_VL];
		robot->VMA[VR].settings = requestBuf[REQUEST_CONFIG_SETTING_VR];
		
		robot->VMA[HLB].kForward = FloatFromUint8(requestBuf, REQUEST_CONFIG_K_FORWARD_HLB);
		robot->VMA[HLF].kForward = FloatFromUint8(requestBuf, REQUEST_CONFIG_K_FORWARD_HLF);
		robot->VMA[HRB].kForward = FloatFromUint8(requestBuf, REQUEST_CONFIG_K_FORWARD_HRB);
		robot->VMA[HRF].kForward = FloatFromUint8(requestBuf, REQUEST_CONFIG_K_FORWARD_HRF);
		robot->VMA[VB].kForward = FloatFromUint8(requestBuf, REQUEST_CONFIG_K_FORWARD_VB);
		robot->VMA[VF].kForward = FloatFromUint8(requestBuf, REQUEST_CONFIG_K_FORWARD_VF);
		robot->VMA[VL].kForward = FloatFromUint8(requestBuf, REQUEST_CONFIG_K_FORWARD_VL);
		robot->VMA[VR].kForward = FloatFromUint8(requestBuf, REQUEST_CONFIG_K_FORWARD_VR);
		
		robot->VMA[HLB].kBackward = FloatFromUint8(requestBuf, REQUEST_CONFIG_K_FORWARD_HLB);
		robot->VMA[HLF].kBackward = FloatFromUint8(requestBuf, REQUEST_CONFIG_K_FORWARD_HLF);
		robot->VMA[HRB].kBackward = FloatFromUint8(requestBuf, REQUEST_CONFIG_K_FORWARD_HRB);
		robot->VMA[HRF].kBackward = FloatFromUint8(requestBuf, REQUEST_CONFIG_K_FORWARD_HRF);
		robot->VMA[VB].kBackward = FloatFromUint8(requestBuf, REQUEST_CONFIG_K_FORWARD_VB);
		robot->VMA[VF].kBackward = FloatFromUint8(requestBuf, REQUEST_CONFIG_K_FORWARD_VF);
		robot->VMA[VL].kBackward = FloatFromUint8(requestBuf, REQUEST_CONFIG_K_FORWARD_VL);
		robot->VMA[VR].kBackward = FloatFromUint8(requestBuf, REQUEST_CONFIG_K_FORWARD_VR);
	}
}


void ShoreRequest(struct Robot *robot, uint8_t *requestBuf, int16_t *pitchError, int16_t *rollError)
{
	if (IsChecksumm16bCorrect(requestBuf, SHORE_REQUEST_LENGTH)) {
		shorePackageError = 0;
		robot->j_movement.march = (((int16_t)requestBuf[SHORE_REQUEST_MARCH]) << 8) | requestBuf[SHORE_REQUEST_MARCH + 1];
		robot->j_movement.lag = (((int16_t)requestBuf[SHORE_REQUEST_LAG]) << 8) | requestBuf[SHORE_REQUEST_LAG + 1];
		robot->j_movement.depth = (((int16_t)requestBuf[SHORE_REQUEST_DEPTH]) << 8) | requestBuf[SHORE_REQUEST_DEPTH + 1];
		robot->j_movement.pitch = (((int16_t)requestBuf[SHORE_REQUEST_ROLL]) << 8) | requestBuf[SHORE_REQUEST_ROLL + 1];
		robot->j_movement.roll = (((int16_t)requestBuf[SHORE_REQUEST_PITCH]) << 8) | requestBuf[SHORE_REQUEST_PITCH + 1];
		robot->j_movement.yaw = (((int16_t)requestBuf[SHORE_REQUEST_YAW]) << 8) | requestBuf[SHORE_REQUEST_YAW + 1];
		
		robot->j_movement.pitch -= *pitchError;
		robot->j_movement.roll -= *rollError;;
		
		robot->device.light.brightness = requestBuf[SHORE_REQUEST_LIGHT];
		robot->device.agar.opening = requestBuf[SHORE_REQUEST_AGAR];
		robot->device.bottomLight.brightness = requestBuf[SHORE_REQUEST_BOTTOM_LIGHT];
				
		robot->device.grab.squeeze = requestBuf[SHORE_REQUEST_GRAB];
		if (robot->device.grab.squeeze < -127){
			robot->device.grab.squeeze = -127;
		}	
		robot->device.grab.rotation  = requestBuf[SHORE_REQUEST_GRAB_ROTATE]; 
		if (robot->device.grab.rotation < -127){
			robot->device.grab.rotation = -127;
		}
		robot->device.tilt.rotation = requestBuf[SHORE_REQUEST_TILT];
		if (robot->device.tilt.rotation < -127){
			robot->device.tilt.rotation = -127;
		}	
		
		robot->depthStabilization.enable = (bool) requestBuf[SHORE_REQUEST_STABILIZE_DEPTH];
		robot->rollStabilization.enable = (bool) requestBuf[SHORE_REQUEST_STABILIZE_ROLL];
		robot->pitchStabilization.enable = (bool) requestBuf[SHORE_REQUEST_STABILIZE_PITCH ];
		robot->yawStabilization.enable = (bool) requestBuf[SHORE_REQUEST_STABILIZE_YAW ];
	
		robot->sensors.resetIMU = (bool) requestBuf[SHORE_REQUEST_RESET_IMU];
		
		int16_t bYaw, bRoll, bPitch;
		if(robot->rollStabilization.enable) {
			bRoll = robot->j_movement.yaw-robot->rollStabilization.speedError;
		}
		else {
			bRoll = robot->j_movement.roll;
		}
		
		if(robot->pitchStabilization.enable) {
			bPitch = robot->j_movement.yaw-robot->pitchStabilization.speedError;
		}
		else {
			bPitch = robot->j_movement.pitch;
		}
		
		if(robot->yawStabilization.enable) {
			bYaw = robot->j_movement.yaw-robot->yawStabilization.speedError;
		}
		else {
			bYaw = robot->j_movement.yaw;
		}
		
		int16_t velocity[VMA_DRIVER_NUMBER];
		velocity[HLB] = (int16_t)( - robot->j_movement.march + robot->j_movement.lag - bYaw); 
		velocity[HLF] = (int16_t)(+ robot->j_movement.march + robot->j_movement.lag + bYaw);
		velocity[HRB] = (int16_t)-( - robot->j_movement.march - robot->j_movement.lag + bYaw);
		velocity[HRF] = (int16_t)(+ robot->j_movement.march - robot->j_movement.lag - bYaw);
		velocity[VB] = (int16_t)(- robot->j_movement.depth - bPitch); 
		velocity[VF] = (int16_t)-(- robot->j_movement.depth + bPitch); 
		velocity[VL] = (int16_t)(- robot->j_movement.depth + bRoll);
		velocity[VR] = (int16_t)(- robot->j_movement.depth - bRoll);
		
		for (uint8_t i = 0; i < VMA_DRIVER_NUMBER; ++i) {
			velocity[i] = (int8_t)(velocity[i] / 0xFF);
			if (velocity[i] > 127) {
				robot->VMA[i].desiredSpeed = 127;
			}
			else if( velocity[i] > -127) {
				robot->VMA[i].desiredSpeed = velocity[i];
			}
			else {
				robot->VMA[i].desiredSpeed = -127;				
			}
		}
	}
	else{
		++shorePackageError;
	}
	
	if (shorePackageError == PACKAGE_TOLLERANCE) {
		robot->j_movement.march = 0;
		robot->j_movement.lag = 0;
		robot->j_movement.depth = 0;
		robot->j_movement.pitch = 0;
		robot->j_movement.roll = 0;
		robot->j_movement.yaw = 0;
	
		robot->device.light.brightness = 0;
		robot->device.agar.opening = 0;
		robot->device.bottomLight.brightness = 0;
		robot->device.grab.squeeze = 0;
		robot->device.grab.rotation  = 0;
		robot->device.tilt.rotation = 0;
		
		for (uint8_t i = 0; i < VMA_DRIVER_NUMBER; ++i){
			robot->VMA[i].desiredSpeed = 0;
		}
	}
}

void ShoreResponse(struct Robot *robot, uint8_t *responseBuf)
{
	responseBuf[SHORE_RESPONSE_ROLL] = (int16_t) robot->sensors.roll >> 8;
	responseBuf[SHORE_RESPONSE_ROLL + 1] = (int16_t) robot->sensors.roll;
	responseBuf[SHORE_RESPONSE_PITCH] = (int16_t) robot->sensors.pitch >> 8;
	responseBuf[SHORE_RESPONSE_PITCH + 1] = (int16_t) robot->sensors.pitch;
	responseBuf[SHORE_RESPONSE_YAW] = (int16_t) robot->sensors.yaw >> 8;
	responseBuf[SHORE_RESPONSE_YAW + 1] = (int16_t) robot->sensors.yaw;
	responseBuf[SHORE_RESPONSE_ROLL_SPEED] = (int16_t) robot->sensors.rollSpeed >> 8;
	responseBuf[SHORE_RESPONSE_ROLL_SPEED + 1] = (int16_t) robot->sensors.rollSpeed;
	responseBuf[SHORE_RESPONSE_PITCH_SPEED] = (int16_t) robot->sensors.pitchSpeed >> 8;
	responseBuf[SHORE_RESPONSE_PITCH_SPEED + 1] = (int16_t) robot->sensors.pitchSpeed;
	responseBuf[SHORE_RESPONSE_YAW_SPEED] = (int16_t) robot->sensors.yawSpeed >> 8;
	responseBuf[SHORE_RESPONSE_YAW_SPEED + 1] = (int16_t) robot->sensors.yawSpeed;
	
  responseBuf[SHORE_RESPONSE_PRESSURE] = ((int16_t) robot->sensors.pressure) >> 8;
	responseBuf[SHORE_RESPONSE_PRESSURE + 1] = robot->sensors.pressure;
	
	for(uint8_t i = 0; i < BLUETOOTH_MESSAGE_SIZE; ++i) {
		responseBuf[SHORE_RESPONSE_BLUETOOTH + i] = robot->bluetooth.message[i];
	}	
	
	responseBuf[SHORE_RESPONSE_VMA_CURRENT_HLB] = robot->VMA[HLB].current >> 8;
	responseBuf[SHORE_RESPONSE_VMA_CURRENT_HLB + 1] = robot->VMA[HLB].current;
	responseBuf[SHORE_RESPONSE_VMA_CURRENT_HLF] = robot->VMA[HLF].current >> 8;
	responseBuf[SHORE_RESPONSE_VMA_CURRENT_HLF + 1] = robot->VMA[HLF].current;
	responseBuf[SHORE_RESPONSE_VMA_CURRENT_HRB] = robot->VMA[HRB].current >> 8;
	responseBuf[SHORE_RESPONSE_VMA_CURRENT_HRB + 1] = robot->VMA[HRB].current;
	responseBuf[SHORE_RESPONSE_VMA_CURRENT_HRF] = robot->VMA[HRF].current >> 8;
	responseBuf[SHORE_RESPONSE_VMA_CURRENT_HRF + 1] = robot->VMA[HRF].current;
	responseBuf[SHORE_RESPONSE_VMA_CURRENT_VB] = robot->VMA[VB].current >> 8;
	responseBuf[SHORE_RESPONSE_VMA_CURRENT_VB + 1] = robot->VMA[VB].current;
	responseBuf[SHORE_RESPONSE_VMA_CURRENT_VF] = robot->VMA[VF].current >> 8;
	responseBuf[SHORE_RESPONSE_VMA_CURRENT_VF + 1] = robot->VMA[VF].current;
	responseBuf[SHORE_RESPONSE_VMA_CURRENT_VL] = robot->VMA[VL].current >> 8;
	responseBuf[SHORE_RESPONSE_VMA_CURRENT_VL + 1] = robot->VMA[VL].current;
	responseBuf[SHORE_RESPONSE_VMA_CURRENT_VR] = robot->VMA[VR].current >> 8;
	responseBuf[SHORE_RESPONSE_VMA_CURRENT_VR + 1] = robot->VMA[VR].current;
		
	responseBuf[SHORE_RESPONSE_VMA_VELOCITY_HLB] = robot->VMA[HLB].desiredSpeed;
	responseBuf[SHORE_RESPONSE_VMA_VELOCITY_HLF] = robot->VMA[HLF].desiredSpeed;
	responseBuf[SHORE_RESPONSE_VMA_VELOCITY_HRB] = robot->VMA[HRB].desiredSpeed;
	responseBuf[SHORE_RESPONSE_VMA_VELOCITY_HRF] = robot->VMA[HRF].desiredSpeed;
	responseBuf[SHORE_RESPONSE_VMA_VELOCITY_VB] = robot->VMA[VB].desiredSpeed;
	responseBuf[SHORE_RESPONSE_VMA_VELOCITY_VF] = robot->VMA[VF].desiredSpeed;
	responseBuf[SHORE_RESPONSE_VMA_VELOCITY_VL] = robot->VMA[VR].desiredSpeed;
	responseBuf[SHORE_RESPONSE_VMA_VELOCITY_VR] = robot->VMA[VR].desiredSpeed;
		
	responseBuf[SHORE_RESPONSE_LIGHT_CURRENT] = robot->device.light.current;
	responseBuf[SHORE_RESPONSE_BOTTOM_LIGHT_CURRENT] = robot->device.bottomLight.current;
	responseBuf[SHORE_RESPONSE_AGAR_CURRENT] = robot->device.agar.current;
	responseBuf[SHORE_RESPONSE_GRAB_CURRENT] = robot->device.grab.squeezeCurrent;
	responseBuf[SHORE_RESPONSE_GRAB_ROTATE_CURRENT] = robot->device.grab.rotationCurrent;
	responseBuf[SHORE_RESPONSE_CURRENT_TILT] = robot->device.tilt.current;
	
	responseBuf[SHORE_RESPONSE_VMA_ERRORS] = 0x55;         //!!!!!TODO!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	responseBuf[SHORE_RESPONSE_VMA_ERRORS + 1] = 0x55;
	responseBuf[SHORE_RESPONSE_DEV_ERRORS] = robot->device.errors;
		
	AddChecksumm16b(responseBuf, SHORE_RESPONSE_LENGTH);
}	


void IMUReceive(struct Robot *robot, uint8_t *ReceiveBuf, uint8_t *ErrCode)
{
	for(uint8_t i = 0; i < IMU_CHECKSUMS; ++i) {
		if(!IsChecksum16bSCorrect(&ReceiveBuf[i*IMU_RESPONSE_LENGTH], IMU_RESPONSE_LENGTH)) {
			*ErrCode = 1;
			return;
		}
	}
	
	int16_t buffer = 0;
	
	buffer = (((int16_t) ReceiveBuf[EULER_PSI]) << 8) | ReceiveBuf[EULER_PSI+1];
	robot->sensors.yaw = ((double) buffer) * 0.0109863;
	buffer = (((int16_t) ReceiveBuf[EULER_PHI]) << 8) | ReceiveBuf[EULER_PHI+1];
	robot->sensors.roll = ((double) buffer) * 0.0109863;
	buffer = (((int16_t) ReceiveBuf[EULER_TETA]) << 8) | ReceiveBuf[EULER_TETA+1];
	robot->sensors.pitch = ((double) buffer) * 0.0109863;
	
	buffer = (((int16_t) ReceiveBuf[GYRO_PROC_X]) << 8) | ReceiveBuf[GYRO_PROC_X+1];
	robot->sensors.rollSpeed = ((double) buffer) * 0.1; //0.000183105;
	buffer = (((int16_t) ReceiveBuf[GYRO_PROC_Y]) << 8) | ReceiveBuf[GYRO_PROC_Y+1];
	robot->sensors.pitchSpeed = ((double) buffer) * 0.1;
	buffer = (((int16_t) ReceiveBuf[GYRO_PROC_Z]) << 8) | ReceiveBuf[GYRO_PROC_Z+1];
	robot->sensors.yawSpeed = ((double) buffer) * 0.1;
	
	buffer = (((int16_t) ReceiveBuf[ACCEL_PROC_X]) << 8) | ReceiveBuf[ACCEL_PROC_X+1];
	robot->sensors.accelX = ((double) buffer) * 0.000183105;
	buffer = (((int16_t) ReceiveBuf[ACCEL_PROC_Y]) << 8) | ReceiveBuf[ACCEL_PROC_Y+1];
	robot->sensors.accelY = ((double) buffer) * 0.000183105;
	buffer = (((int16_t) ReceiveBuf[ACCEL_PROC_Z]) << 8) | ReceiveBuf[ACCEL_PROC_Z+1];
	robot->sensors.accelZ = ((double) buffer) * 0.000183105;
	
	buffer = (((int16_t) ReceiveBuf[MAG_PROC_X]) << 8) | ReceiveBuf[MAG_PROC_X+1];
	robot->sensors.magX = ((double) buffer) * 0.000305176;
	buffer = (((int16_t) ReceiveBuf[MAG_PROC_Y]) << 8) | ReceiveBuf[MAG_PROC_Y+1];
	robot->sensors.magY = ((double) buffer) * 0.000305176;
	buffer = (((int16_t) ReceiveBuf[MAG_PROC_Z]) << 8) | ReceiveBuf[MAG_PROC_Z+1];
	robot->sensors.magZ = ((double) buffer) * 0.000305176;
	
	buffer = (((int16_t) ReceiveBuf[QUAT_A]) << 8) | ReceiveBuf[QUAT_A+1];
	robot->sensors.quatA = ((double) buffer) * 0.0000335693;
	buffer = (((int16_t) ReceiveBuf[QUAT_B]) << 8) | ReceiveBuf[QUAT_B+1];
	robot->sensors.quatB = ((double) buffer) * 0.0000335693;
	buffer = (((int16_t) ReceiveBuf[QUAT_C]) << 8) | ReceiveBuf[QUAT_C+1];
	robot->sensors.quatC = ((double) buffer) * 0.0000335693;
	buffer = (((int16_t) ReceiveBuf[QUAT_D]) << 8) | ReceiveBuf[QUAT_D+1];	
	robot->sensors.quatD = ((double) buffer) * 0.0000335693;
}
