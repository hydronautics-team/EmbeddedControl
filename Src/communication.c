#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "cmsis_os.h"
#include "usart.h"
#include "i2c.h"
#include "main.h"
#include "tim.h"

#include "communication.h"
#include "global.h"
#include "messages.h"
#include "checksum.h"
#include "robot.h"
#include "stabilization.h"

#define PACKAGE_TOLLERANCE 	20

extern TimerHandle_t UARTTimer;

struct uartBus_s uartBus[UART_NUMBER];

uint8_t VMAbrokenRxTolerance = 0;

const uint16_t ShoreLength[SHORE_REQUEST_MODES_NUMBER] = {SHORE_REQUEST_LENGTH, REQUEST_CONFIG_LENGTH};
const uint8_t ShoreCodes[SHORE_REQUEST_MODES_NUMBER] = {SHORE_REQUEST_CODE, REQUEST_CONFIG_CODE};

uint16_t counterRx = 0;

bool i2c1PackageTransmit = false;
bool i2c2PackageTransmit = false;

bool i2c1PackageReceived = false;
bool i2c2PackageReceived = false;

void variableInit() {
    rThrusters[HLB].address = 6;
    rThrusters[HLF].address = 5;
    rThrusters[HRB].address = 3;
    rThrusters[HRF].address = 4;
    rThrusters[VB].address = 2;
    rThrusters[VF].address = 1;
    rThrusters[VL].address = 8;
    rThrusters[VR].address = 7;

    rThrusters[HLB].kForward = 1;
    rThrusters[HLF].kForward = 1;
    rThrusters[HRB].kForward = 1;
    rThrusters[HRF].kForward = 1;
    rThrusters[VB].kForward = 1;
    rThrusters[VF].kForward = 1;
    rThrusters[VL].kForward = 1;
    rThrusters[VR].kForward = 1;

    rThrusters[HLB].kBackward = 1;
    rThrusters[HLF].kBackward = 1;
    rThrusters[HRB].kBackward = 1;
    rThrusters[HRF].kBackward = 1;
    rThrusters[VB].kBackward = 1;
    rThrusters[VF].kBackward = 1;
    rThrusters[VL].kBackward = 1;
    rThrusters[VR].kBackward = 1;

    rThrusters[HLB].Saturation = 50;
    rThrusters[HLF].Saturation = 50;
    rThrusters[HRB].Saturation = 50;
    rThrusters[HRF].Saturation = 50;
    rThrusters[VB].Saturation = 50;
    rThrusters[VF].Saturation = 50;
    rThrusters[VL].Saturation = 50;
    rThrusters[VR].Saturation = 50;

    rThrusters[HLB].inverse = true;
    rThrusters[HLF].inverse = false;
    rThrusters[HRB].inverse = false;
    rThrusters[HRF].inverse = false;
    rThrusters[VB].inverse = false;
    rThrusters[VF].inverse = false;
    rThrusters[VL].inverse = false;
    rThrusters[VR].inverse = true;

    rDevice[DEV1].address = 0x03;
    rDevice[GRAB].address = 0x01;
    rDevice[GRAB_ROTATION].address = 0x02;
    rDevice[TILT].address = 0x04;

    rSensors.yaw = 0;
    rSensors.roll =  0;
    rSensors.pitch =  0;

    rSensors.rollSpeed = 0;
    rSensors.pitchSpeed = 0;
    rSensors.yawSpeed = 0;

    rSensors.accelX = 0;
    rSensors.accelY = 0;
    rSensors.accelZ = 0;

    rSensors.magX = 0;
    rSensors.magY = 0;
    rSensors.magZ = 0;

    rSensors.quatA = 0;
    rSensors.quatB = 0;
    rSensors.quatC = 0;
    rSensors.quatD = 0;

    rSensors.resetIMU = true;
}

void uartBusesInit()
{
	// Shore UART configuration
	uartBus[SHORE_UART].huart = &huart1; // Link to huart will be set before receiving
	uartBus[SHORE_UART].rxBuffer = ShoreRequestBuffer;
	uartBus[SHORE_UART].txBuffer = ShoreResponseBuffer;
	uartBus[SHORE_UART].rxLength = 0; // Length of the received message will be determined when first byte will be received
	uartBus[SHORE_UART].txLength = 0; // Length of the transmitted message will be determined before transmit
	uartBus[SHORE_UART].brokenRxTolerance = 20;
	uartBus[SHORE_UART].timeoutRxTolerance = 500;
	uartBus[SHORE_UART].receiveTimeout = 200;
	uartBus[SHORE_UART].transmitTimeout = 200;
	uartBus[SHORE_UART].txrxType = TXRX_IT;

	// Thrusters UART configuration
	uartBus[THRUSTERS_UART].huart = &huart2;
	uartBus[THRUSTERS_UART].rxBuffer = 0; // Receive bugger will be set before receive
	uartBus[THRUSTERS_UART].txBuffer = 0; // Transmit bugger will be set before transmit
	uartBus[THRUSTERS_UART].rxLength = 0; // Receive length will be set before transmit
	uartBus[THRUSTERS_UART].txLength = 0; // Transmit length will be set before transmit
	uartBus[THRUSTERS_UART].brokenRxTolerance = 0; // There is no special event on this bus
	uartBus[THRUSTERS_UART].timeoutRxTolerance = 0; // There is no special event on this bus
	uartBus[THRUSTERS_UART].receiveTimeout = 100;
	uartBus[THRUSTERS_UART].transmitTimeout = 100;
	uartBus[THRUSTERS_UART].txrxType = TXRX_DMA;

	// Devices UART configuration
	uartBus[DEVICES_UART].huart = &huart3;
	uartBus[DEVICES_UART].rxBuffer = 0; // Receive bugger will be set before receive
	uartBus[DEVICES_UART].txBuffer = 0; // Transmit bugger will be set before transmit
	uartBus[DEVICES_UART].rxLength = DEVICES_REQUEST_LENGTH;
	uartBus[DEVICES_UART].txLength = DEVICES_RESPONSE_LENGTH;
	uartBus[DEVICES_UART].brokenRxTolerance = 0; // There is no special event on this bus
	uartBus[DEVICES_UART].timeoutRxTolerance = 0; // There is no special event on this bus
	uartBus[DEVICES_UART].receiveTimeout = 100;
	uartBus[DEVICES_UART].transmitTimeout = 100;
	uartBus[DEVICES_UART].txrxType = TXRX_DMA;

	// IMU UART configuration
	uartBus[IMU_UART].huart = &huart4;
	uartBus[IMU_UART].rxBuffer = ImuResponseBuffer;
	uartBus[IMU_UART].txBuffer = 0; // Buffer will be set before transmit
	uartBus[IMU_UART].rxLength = 0; // Receive length will be set before transmit
	uartBus[IMU_UART].txLength = 0; // Transmit length will be set before transmit
	uartBus[IMU_UART].brokenRxTolerance = 0; // There is no special event on this bus
	uartBus[IMU_UART].timeoutRxTolerance = 0; // There is no special event on this bus
	uartBus[IMU_UART].receiveTimeout = 100;
	uartBus[IMU_UART].transmitTimeout = 100;
	uartBus[IMU_UART].txrxType = TXRX_IT;

	for(uint8_t i=0; i<UART_NUMBER; i++) {
		uartBus[i].packageReceived = false;
		uartBus[i].packageTransmitted = false;
		uartBus[i].successRxCounter = 0;
		uartBus[i].brokenRxCounter = 0;
		uartBus[i].outdatedRxCounter = 0;
		uartBus[i].timeoutCounter = 0;
		uartBus[i].lastMessage = 0;
	}
}

bool transmitPackage(struct uartBus_s *bus, bool isrMode)
{
    bus->packageTransmitted = false;

    HAL_UART_AbortTransmit_IT(bus->huart);
    switch(bus->txrxType) {
        case TXRX_DMA:
            HAL_UART_Transmit_DMA(bus->huart, bus->txBuffer, bus->txLength);
            break;
        case TXRX_IT:
        	HAL_UART_Transmit_IT(bus->huart, bus->txBuffer, bus->txLength);
            break;
        default:
            return false;
    }

    bus->timeoutCounter = fromTickToMs(xTaskGetTickCount());
    while (!bus->packageTransmitted && !isrMode) {
    	if(fromTickToMs(xTaskGetTickCount()) - bus->timeoutCounter > bus->transmitTimeout) {
    		return false;
    	}
    	osDelay(DELAY_UART_TIMEOUT);
    }
    return true;
}

bool receivePackage(struct uartBus_s *bus, bool isrMode)
{
	bus->packageReceived = false;

	HAL_UART_AbortReceive_IT(bus->huart);
	switch(bus->txrxType) {
		case TXRX_DMA:
			HAL_UART_Receive_DMA(bus->huart, bus->rxBuffer, bus->rxLength);
			break;
		case TXRX_IT:
			HAL_UART_Receive_IT(bus->huart, bus->rxBuffer, bus->rxLength);
			break;
		default:
			return false;
	}

	bus->timeoutCounter = HAL_GetTick();
	while (!bus->packageReceived && !isrMode) {
		if(HAL_GetTick() - bus->timeoutCounter > bus->receiveTimeout) {
			return false;
		}
		osDelay(DELAY_UART_TIMEOUT);
	}
	return true;
}

bool transmitAndReceive(struct uartBus_s *bus, bool isrMode)
{
	bus->packageReceived = false;
	bus->packageTransmitted = false;

	HAL_UART_AbortReceive_IT(bus->huart);
	HAL_UART_AbortTransmit_IT(bus->huart);
	switch(bus->txrxType) {
		case TXRX_DMA:
			HAL_UART_Receive_DMA(bus->huart, bus->rxBuffer, bus->rxLength);
			HAL_UART_Transmit_DMA(bus->huart, bus->txBuffer, bus->txLength);
			break;
		case TXRX_IT:
			HAL_UART_Receive_IT(bus->huart, bus->rxBuffer, bus->rxLength);
			HAL_UART_Transmit_IT(bus->huart, bus->txBuffer, bus->txLength);
			break;
		default:
			return false;
	}

	bus->timeoutCounter = fromTickToMs(xTaskGetTickCount());
	while (!bus->packageTransmitted && !bus->packageReceived && !isrMode) {
		if(fromTickToMs(xTaskGetTickCount()) - bus->timeoutCounter > bus->transmitTimeout) {
			return false;
		}
		osDelay(DELAY_UART_TIMEOUT);
	}
	return true;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == uartBus[SHORE_UART].huart) {
		uartBus[SHORE_UART].packageTransmitted = true;
		return;
	}

	struct uartBus_s *bus = 0;
	for(uint8_t i=0; i<UART_NUMBER; i++) {
		if(uartBus[i].huart == huart) {
			bus = &uartBus[i];
			bus->packageTransmitted = true;
			break;
		}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == uartBus[SHORE_UART].huart) {
		ShoreReceive();
		return;
	}

	struct uartBus_s *bus = 0;
	for(uint8_t i=0; i<UART_NUMBER; i++) {
		if(uartBus[i].huart == huart) {
			bus = &uartBus[i];
			bus->packageReceived = true;
			bus->lastMessage = fromTickToMs(xTaskGetTickCount());
			break;
		}
	}
}

bool receiveI2cPackageDMA (uint8_t I2C, uint16_t addr, uint8_t *buf, uint8_t length)
{
	float timeBegin = fromTickToMs(xTaskGetTickCount());
	i2c2PackageReceived = false;
	switch(I2C) {
	case DEV_I2C:
		HAL_I2C_Master_Receive_IT(&hi2c2, addr>>1, buf, length);
		while (!i2c2PackageReceived) {
			if(fromTickToMs(xTaskGetTickCount()) - timeBegin < WAITING_SENSORS) {
				HAL_I2C_Master_Abort_IT(&hi2c2, addr>>1);
				return false;
			}
			osDelay(DELAY_SENSOR_TASK);
		}
		break;
	}
	return true;
}


void transmitI2cPackageDMA(uint8_t I2C, uint16_t addr, uint8_t *buf, uint8_t length)
{
	TickType_t timeBegin = xTaskGetTickCount();
	i2c2PackageTransmit = false;
	switch(I2C) {
	case DEV_I2C:
		HAL_I2C_Master_Transmit_IT(&hi2c2, addr>>1, buf, length);
		while (!i2c2PackageTransmit && xTaskGetTickCount() - timeBegin < WAITING_SENSORS) {
			osDelay(DELAY_SENSOR_TASK);
		}
		break;
	}
}


void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	if(hi2c == &hi2c1) {
		i2c2PackageReceived = true;
	}
	else if(hi2c == &hi2c2) {
		i2c2PackageReceived = true;
	}
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	if(hi2c == &hi2c1) {
		i2c1PackageTransmit = true;
	}
	else if(hi2c == &hi2c2) {
		i2c1PackageTransmit  = true;
	}
}


void SensorsResponseUpdate(uint8_t *buf, uint8_t Sensor_id)
{
	switch(Sensor_id) {
	case DEV_I2C:
		rSensors.pressure = FloatFromUint8Reverse(buf, 0);
		break;
	}
}

void ShoreReceive()
{
	static portBASE_TYPE xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;
	if(counterRx == 0) {
		for(uint8_t i=0; i<SHORE_REQUEST_MODES_NUMBER; ++i) {
			if(uartBus[SHORE_UART].rxBuffer[0] == ShoreCodes[i]) {
				counterRx = 1;
				uartBus[SHORE_UART].rxLength = ShoreLength[i]-1;
				HAL_UART_Receive_IT(uartBus[SHORE_UART].huart, uartBus[SHORE_UART].rxBuffer+1, uartBus[SHORE_UART].rxLength);
				xTimerStartFromISR(UARTTimer, &xHigherPriorityTaskWoken);
				break;
			}

			if(i == SHORE_REQUEST_MODES_NUMBER-1) {
				HAL_UART_Receive_IT(uartBus[SHORE_UART].huart, uartBus[SHORE_UART].rxBuffer, 1);
			}
		}
	}
	else if(counterRx == 1) {
		uartBus[SHORE_UART].packageReceived = true;
		uartBus[SHORE_UART].lastMessage = fromTickToMs(xTaskGetTickCount());
		counterRx = 2;
	}

	if (xHigherPriorityTaskWoken == pdTRUE) {
		xHigherPriorityTaskWoken = pdFALSE;
		taskYIELD();
	}
}

void DevicesRequestUpdate(uint8_t *buf, uint8_t dev)
{
	struct devicesRequest_s req;

    req.AA1 = 0xAA;
    req.AA2 = 0xAA;
    req.address = rDevice[dev].address;
    req.setting = rDevice[dev].settings;
    req.velocity1 = 0;
    req.velocity2 = rDevice[dev].force;

    memcpy((void*)buf, (void*)&req, DEVICES_REQUEST_LENGTH);
    AddChecksumm8b(buf, DEVICES_REQUEST_LENGTH);
}

void DevicesResponseUpdate(uint8_t *buf, uint8_t dev)
{
    if(IsChecksumm8bCorrect(buf, DEVICES_RESPONSE_LENGTH)) {
    	struct devicesResponse_s res;
    	memcpy((void*)&res, (void*)buf, DEVICES_RESPONSE_LENGTH);

        rDevice[dev].current = res.current1;
        // TODO make errors work pls
        //writeBit(&(robot->device[dev].errors), res.errors, AGAR);

        ++uartBus[DEVICES_UART].successRxCounter;
    }
    else {
    	++uartBus[DEVICES_UART].brokenRxCounter;
    }
}

void ThrustersRequestUpdate(uint8_t *buf, uint8_t thruster)
{
    struct thrustersRequest_s res;

    res.AA = 0xAA;
    res.type = 0x01;
    res.address = rThrusters[thruster].address;
    res.velocity = rThrusters[thruster].desiredSpeed;

    // Inverting
    if(rThrusters[thruster].inverse) {
    	res.velocity *= -1;
    }

    // Multiplier constants
    if(rThrusters[thruster].desiredSpeed > 0) {
    	res.velocity *= rThrusters[thruster].kForward;
    }
    else if(rThrusters[thruster].desiredSpeed < 0) {
    	res.velocity *= rThrusters[thruster].kBackward;
    }

    // Saturation
    if(rThrusters[thruster].desiredSpeed > rThrusters[thruster].Saturation) {
    	rThrusters[thruster].desiredSpeed = rThrusters[thruster].Saturation;
    }
    else if(rThrusters[thruster].desiredSpeed < -rThrusters[thruster].Saturation) {
    	rThrusters[thruster].desiredSpeed = -rThrusters[thruster].Saturation;
    }

    memcpy((void*)buf, (void*)&res, THRUSTERS_REQUEST_LENGTH);
    AddChecksumm8bVma(buf, THRUSTERS_REQUEST_LENGTH);
}

void ThrustersResponseUpdate(uint8_t *buf, uint8_t thruster)
{
	//TODO errors parsing! and what is all this new stuff means
    if(IsChecksumm8bCorrectVma(buf, THRUSTERS_RESPONSE_LENGTH) && buf[0] != 0) {
    	struct thrustersResponse_s res;
    	memcpy((void*)&res, (void*)buf, THRUSTERS_RESPONSE_LENGTH);

        rThrusters[thruster].current = res.current;

        ++uartBus[THRUSTERS_UART].successRxCounter;
    }
    else {
    	++uartBus[THRUSTERS_UART].brokenRxCounter;
    }
}

void ShoreConfigRequest(uint8_t *requestBuf)
{
	if(IsCrc16ChecksummCorrect(requestBuf, REQUEST_CONFIG_LENGTH)) {
		struct shoreConfigRequest_s req;
		memcpy((void*)&req, (void*)requestBuf, REQUEST_CONFIG_LENGTH);

		rJoySpeed.march = req.march;
		rJoySpeed.lag = req.lag;
		rJoySpeed.depth = req.depth;
		rJoySpeed.roll = req.roll;
		rJoySpeed.pitch = req.pitch;
		rJoySpeed.yaw = req.yaw;

		rStabConstants[req.contour].pJoyUnitCast = req.pJoyUnitCast;
		rStabConstants[req.contour].pSpeedDyn = req.pSpeedDyn;
		rStabConstants[req.contour].pErrGain = req.pErrGain;

		rStabConstants[req.contour].aFilter[POS_FILTER].T = req.posFilterT;
		rStabConstants[req.contour].aFilter[POS_FILTER].K = req.posFilterK;
		rStabConstants[req.contour].aFilter[SPEED_FILTER].T = req.speedFilterT;
		rStabConstants[req.contour].aFilter[SPEED_FILTER].K = req.speedFilterK;

		rStabConstants[req.contour].pid.pGain = req.pid_pGain;
		rStabConstants[req.contour].pid.iGain = req.pid_iGain;
		rStabConstants[req.contour].pid.iMax = req.pid_iMax;
		rStabConstants[req.contour].pid.iMin = req.pid_iMin;

		rStabConstants[req.contour].pThrustersCast = req.pThrustersCast;
		rStabConstants[req.contour].pThrustersMin = req.pThrustersMin;
		rStabConstants[req.contour].pThrustersMax = req.pThrustersMax;

		uint8_t old_contour = rState.contourSelected;
		if(old_contour != req.contour) {
			stabilizationStart(req.contour);
		}

		for(uint8_t i=0; i<STABILIZATION_AMOUNT; i++) {
			rStabConstants[i].enable = false;
		}
		rStabConstants[req.contour].enable = true;
		rState.contourSelected = req.contour;

		// TODO tuuuupooo
		formThrustVectors();

		++uartBus[SHORE_UART].successRxCounter;;
	}
	else {
		++uartBus[SHORE_UART].brokenRxCounter;
	}
}


void ShoreRequest(uint8_t *requestBuf)
{
    if (IsCrc16ChecksummCorrect(requestBuf, SHORE_REQUEST_LENGTH)) {
    	struct shoreRequest_s req;
    	memcpy((void*)&req, (void*)requestBuf, SHORE_REQUEST_LENGTH);

    	uint8_t tempCameraNum = 0;

        rJoySpeed.march = req.march;
        rJoySpeed.lag = req.lag;
        rJoySpeed.depth = req.depth;
        rJoySpeed.roll = req.roll;
        rJoySpeed.pitch = req.pitch;
        rJoySpeed.yaw = req.yaw;

        rDevice[LIGHT].force = req.light;
        rDevice[GRAB].force = req.grab;
        if (rDevice[GRAB].force < -127) {
            rDevice[GRAB].force = -127;
        }
        rDevice[TILT].force = req.tilt;
        if (rDevice[TILT].force < -127) {
        	rDevice[TILT].force = -127;
        }
        rDevice[GRAB_ROTATION].force  = req.grab_rotate;
        if (rDevice[GRAB_ROTATION].force < -127) {
            rDevice[GRAB_ROTATION].force = -127;
        }

        rDevice[DEV1].force = req.dev1;
        rDevice[DEV2].force = req.dev2;

        rSensors.resetIMU = PickBit(req.stabilize_flags, SHORE_STABILIZE_IMU_BIT);

        tempCameraNum = req.cameras;

        uint8_t old_reset = rComputer.reset;
        if(old_reset != req.pc_reset) {
            if(req.pc_reset == PC_ON_CODE) {
            	HAL_GPIO_WritePin(GPIOE, RES_PC_2_Pin, GPIO_PIN_RESET); // ONOFF
            }
            else if(req.pc_reset == PC_OFF_CODE) {
            	HAL_GPIO_WritePin(GPIOE, RES_PC_2_Pin, GPIO_PIN_SET); // ONOFF
            }
        }
        rComputer.reset = req.pc_reset;

        bool wasEnabled;

        wasEnabled = rStabConstants[STAB_YAW].enable;
        rStabConstants[STAB_YAW].enable = PickBit(req.stabilize_flags, SHORE_STABILIZE_YAW_BIT);
        if(wasEnabled == false && rStabConstants[STAB_YAW].enable == true) {
        	stabilizationStart(STAB_YAW);
        }

        wasEnabled = rStabConstants[STAB_ROLL].enable;
        rStabConstants[STAB_ROLL].enable = PickBit(req.stabilize_flags, SHORE_STABILIZE_ROLL_BIT);
        if(wasEnabled == false && rStabConstants[STAB_ROLL].enable == true) {
        	stabilizationStart(STAB_ROLL);
        }

        wasEnabled = rStabConstants[STAB_PITCH].enable;
        rStabConstants[STAB_PITCH].enable = PickBit(req.stabilize_flags, SHORE_STABILIZE_PITCH_BIT);
        if(wasEnabled == false && rStabConstants[STAB_PITCH].enable == true) {
        	stabilizationStart(STAB_PITCH);
        }

        wasEnabled = rStabConstants[STAB_DEPTH].enable;
        rStabConstants[STAB_DEPTH].enable = PickBit(req.stabilize_flags, SHORE_STABILIZE_DEPTH_BIT);
        if(wasEnabled == false && rStabConstants[STAB_DEPTH].enable == true) {
        	stabilizationStart(STAB_DEPTH);
        }

        if(tempCameraNum != rState.cameraNum) {
        	rState.cameraNum = tempCameraNum;
        	switch(rState.cameraNum) {
        	case 0:
        		HAL_GPIO_WritePin(GPIOA, CAM1_Pin, GPIO_PIN_RESET);
        		HAL_GPIO_WritePin(GPIOA, CAM2_Pin, GPIO_PIN_RESET);
        		break;

        	case 1:
        		HAL_GPIO_WritePin(GPIOA, CAM1_Pin, GPIO_PIN_SET);
        		HAL_GPIO_WritePin(GPIOA, CAM2_Pin, GPIO_PIN_RESET);
        		break;

        	case 2:
        		HAL_GPIO_WritePin(GPIOA, CAM1_Pin, GPIO_PIN_RESET);
        		HAL_GPIO_WritePin(GPIOA, CAM2_Pin, GPIO_PIN_SET);
        		break;

        	case 3:
        		HAL_GPIO_WritePin(GPIOA, CAM1_Pin, GPIO_PIN_SET);
        		HAL_GPIO_WritePin(GPIOA, CAM2_Pin, GPIO_PIN_SET);
        		break;
        	}
        }

        // TODO tuuuupoooo
        formThrustVectors(&req);

        ++uartBus[SHORE_UART].successRxCounter;
    }
    else {
    	++uartBus[SHORE_UART].brokenRxCounter;

    	//TODO tolerance!
    	/*
        if (brokenRxTolerance == PACKAGE_TOLLERANCE) {
        	robot->i_joySpeed.march = 0;
        	robot->i_joySpeed.lag = 0;
        	robot->i_joySpeed.depth = 0;
        	robot->i_joySpeed.pitch = 0;
        	robot->i_joySpeed.roll = 0;
        	robot->i_joySpeed.yaw = 0;

        	robot->device[LIGHT].force = 0;
        	robot->device[DEV1].force = 0;
        	robot->device[DEV2].force = 0;
        	robot->device[GRAB].force = 0;
        	robot->device[GRAB_ROTATION].force  = 0;
        	robot->device[TILT].force = 0;

        	for (uint8_t i = 0; i < THRUSTERS_NUMBER; ++i){
        		robot->thrusters[i].desiredSpeed = 0;
        	}

        	brokenRxTolerance = 0;
        }
        */
    }
}

void formThrustVectors()
{
	float bYaw, bRoll, bPitch, bDepth;
	if(rStabConstants[STAB_ROLL].enable) {
		bRoll = rStabState[STAB_ROLL].speedError;
	}
	else {
		bRoll = rJoySpeed.roll;
	}

	if(rStabConstants[STAB_PITCH].enable) {
		bPitch = rStabState[STAB_PITCH].speedError;
	}
	else {
		bPitch = rJoySpeed.pitch;
	}

	if(rStabConstants[STAB_YAW].enable) {
		bYaw = rStabState[STAB_YAW].speedError;
	}
	else {
		bYaw = rJoySpeed.yaw;
	}

	if(rStabConstants[STAB_DEPTH].enable) {
		bDepth = rStabState[STAB_DEPTH].speedError;
	}
	else {
		bDepth = rJoySpeed.depth;
	}

	// what the fuck is happening here? this code is actually correct, why?
	int16_t velocity[THRUSTERS_NUMBER];
	velocity[HLB] = + rJoySpeed.march - rJoySpeed.lag + bYaw;
	velocity[HRB] = - rJoySpeed.march - rJoySpeed.lag + bYaw;
	velocity[HLF] = + rJoySpeed.march + rJoySpeed.lag + bYaw;
	velocity[HRF] = - rJoySpeed.march + rJoySpeed.lag + bYaw;
	velocity[VB] = - bDepth + bPitch;
	velocity[VF] = - bDepth - bPitch;
	velocity[VL] = - bDepth + bRoll;
	velocity[VR] = - bDepth - bRoll;

	for (uint8_t i = 0; i < THRUSTERS_NUMBER; ++i) {
		velocity[i] = (int8_t)(velocity[i] / 0xFF);
		if (velocity[i] > 127) {
			rThrusters[i].desiredSpeed = 127;
		}
		else if( velocity[i] > -127) {
			rThrusters[i].desiredSpeed = velocity[i];
		}
		else {
			rThrusters[i].desiredSpeed = -127;
		}
	}
}

void ShoreResponse(uint8_t *responseBuf)
{
	struct shoreResponse_s res;

    res.roll = rSensors.roll;
    res.pitch = rSensors.pitch;
    res.yaw = rSensors.yaw;
    res.rollSpeed = rSensors.rollSpeed;
    res.pitchSpeed = rSensors.pitchSpeed;
    res.yawSpeed = rSensors.yawSpeed;

    res.pressure = rSensors.pressure;

    // TODO eyes bleeding again
    res.vma_current_hlb = rThrusters[HLB].current;
    res.vma_current_hlf = rThrusters[HLF].current;
    res.vma_current_hrb = rThrusters[HRB].current;
    res.vma_current_hrf = rThrusters[HRF].current;
    res.vma_current_vb = rThrusters[VB].current;
    res.vma_current_vf = rThrusters[VF].current;
    res.vma_current_vl = rThrusters[VL].current;
    res.vma_current_vr = rThrusters[VR].current;

    res.dev_current_light = rDevice[LIGHT].current;
    res.dev_current_tilt = rDevice[TILT].current;
    res.dev_current_grab = rDevice[GRAB].current;
    res.dev_current_grab_rotate = rDevice[GRAB_ROTATION].current;
    res.dev_current_dev1 = rDevice[DEV1].current;
    res.dev_current_dev2 = rDevice[DEV2].current;

    res.vma_errors = 0x55;         //!!!!!TODO!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    // TODO do this properly pls
    res.dev_errors = 0;//robot->device.errors;
    res.pc_errors = rComputer.errors;

    memcpy((void*)responseBuf, (void*)&res, SHORE_RESPONSE_LENGTH);

    AddCrc16Checksumm(responseBuf, SHORE_RESPONSE_LENGTH);
}

void ShoreConfigResponse(uint8_t *responseBuf)
{
	struct shoreConfigResponse_s res;

	res.code = REQUEST_CONFIG_CODE;

	res.roll = rSensors.roll;
	res.pitch = rSensors.pitch;
	res.yaw = rSensors.yaw;
	res.rollSpeed = rSensors.rollSpeed;
	res.pitchSpeed = rSensors.pitchSpeed;
	res.yawSpeed = rSensors.yawSpeed;

	res.pressure = rSensors.pressure;
	res.in_pressure = 0;

	res.inputSignal = *rStabState[rState.contourSelected].inputSignal;
	res.speedSignal = *rStabState[rState.contourSelected].speedSignal;
	res.posSignal = *rStabState[rState.contourSelected].posSignal;

	res.oldSpeed = rStabState[rState.contourSelected].oldSpeed;
	res.oldPos = rStabState[rState.contourSelected].oldPos;

	res.joyUnitCasted = rStabState[rState.contourSelected].joyUnitCasted;
	res.joy_iValue = rStabState[rState.contourSelected].joy_iValue;
	res.posError = rStabState[rState.contourSelected].posError;
	res.speedError = rStabState[rState.contourSelected].speedError;
	res.dynSummator = rStabState[rState.contourSelected].dynSummator;
	res.pidValue = rStabState[rState.contourSelected].pidValue;
	res.posErrorAmp = rStabState[rState.contourSelected].posErrorAmp;
	res.speedFiltered = rStabState[rState.contourSelected].speedFiltered;
	res.posFiltered = rStabState[rState.contourSelected].posFiltered;

	res.pid_iValue = rStabState[rState.contourSelected].pid_iValue;

	memcpy((void*)responseBuf, (void*)&res, SHORE_CONFIG_RESPONSE_LENGTH);

	AddCrc16Checksumm(responseBuf, SHORE_CONFIG_RESPONSE_LENGTH);
}

void ImuReceive(uint8_t *ReceiveBuf)
{
    for(uint8_t i = 0; i < IMU_CHECKSUMS; ++i) {
        if(!IsChecksum16bSCorrect(&ReceiveBuf[i*IMU_RESPONSE_LENGTH], IMU_RESPONSE_LENGTH)) {
            ++uartBus[IMU_UART].brokenRxCounter;
            return;
        }
    }


    float yaw = (float) (MergeBytes(&ReceiveBuf[EULER_PSI])) * 0.0109863;
    static float old_yaw = 0;
    static int16_t spins = 0;
    float diff = 0;
    if(abs(old_yaw - yaw) > 180) {
    	if(yaw > 0) {
    		spins--;
    		rSensors.yaw = 360*spins+180;
    		rSensors.yaw -= (180 - abs(yaw));
    	}
    	else {
    		spins++;
    		rSensors.yaw = 360*spins-180;
    		rSensors.yaw += (180 - abs(yaw));
    	}
        old_yaw = yaw;
    }
    else {
    	diff = yaw-old_yaw;
    	rSensors.yaw += diff;
        old_yaw = yaw;
    }

    //rSensors.yaw = (float) (MergeBytes(&ReceiveBuf[EULER_PSI])) * 0.0109863;
    rSensors.roll =  (float) (MergeBytes(&ReceiveBuf[EULER_PHI])) * 0.0109863;
    rSensors.pitch =  (float) (MergeBytes(&ReceiveBuf[EULER_TETA])) * 0.0109863;

    rSensors.rollSpeed = (float) (MergeBytes(&ReceiveBuf[GYRO_PROC_X])) * 0.000183105;
    rSensors.pitchSpeed = (float) (MergeBytes(&ReceiveBuf[GYRO_PROC_Y])) * 0.000183105;
    rSensors.yawSpeed = (float) (MergeBytes(&ReceiveBuf[GYRO_PROC_Z])) * 0.000183105;

    rSensors.accelX = (float) (MergeBytes(&ReceiveBuf[ACCEL_PROC_X])) * 0.0109863;
    rSensors.accelY = (float) (MergeBytes(&ReceiveBuf[ACCEL_PROC_Y])) * 0.0109863;
    rSensors.accelZ = (float) (MergeBytes(&ReceiveBuf[ACCEL_PROC_Z])) * 0.0109863;

    rSensors.magX = (float) (MergeBytes(&ReceiveBuf[MAG_PROC_X])) * 0.000183105;
    rSensors.magY = (float) (MergeBytes(&ReceiveBuf[MAG_PROC_Y])) * 0.000183105;
    rSensors.magZ = (float) (MergeBytes(&ReceiveBuf[MAG_PROC_Z])) * 0.000183105;

    rSensors.quatA = (float) (MergeBytes(&ReceiveBuf[QUAT_A])) * 0.0000335693;
    rSensors.quatB = (float) (MergeBytes(&ReceiveBuf[QUAT_B])) * 0.0000335693;
    rSensors.quatC = (float) (MergeBytes(&ReceiveBuf[QUAT_C])) * 0.0000335693;
    rSensors.quatD = (float) (MergeBytes(&ReceiveBuf[QUAT_D])) * 0.0000335693;

    ++uartBus[IMU_UART].successRxCounter;
}
