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
    Q100.thrusters[HLB].address = 6;
    Q100.thrusters[HLF].address = 5;
    Q100.thrusters[HRB].address = 3;
    Q100.thrusters[HRF].address = 4;
    Q100.thrusters[VB].address = 2;
    Q100.thrusters[VF].address = 1;
    Q100.thrusters[VL].address = 8;
    Q100.thrusters[VR].address = 7;

    Q100.thrusters[HLB].kForward = 1;
    Q100.thrusters[HLF].kForward = 1;
    Q100.thrusters[HRB].kForward = -1;
    Q100.thrusters[HRF].kForward = -1;
    Q100.thrusters[VB].kForward = 1;
    Q100.thrusters[VF].kForward = 1;
    Q100.thrusters[VL].kForward = 1;
    Q100.thrusters[VR].kForward = 1;

    Q100.thrusters[HLB].kForward = 1;
    Q100.thrusters[HLF].kForward = 1;
    Q100.thrusters[HRB].kForward = -1;
    Q100.thrusters[HRF].kForward = -1;
    Q100.thrusters[VB].kForward = 1;
    Q100.thrusters[VF].kForward = 1;
    Q100.thrusters[VL].kForward = 1;
    Q100.thrusters[VR].kForward = 1;

    Q100.thrusters[HLB].inverse = false;
    Q100.thrusters[HLF].inverse = false;
    Q100.thrusters[HRB].inverse = true;
    Q100.thrusters[HRF].inverse = true;
    Q100.thrusters[VB].inverse = false;
    Q100.thrusters[VF].inverse = false;
    Q100.thrusters[VL].inverse = false;
    Q100.thrusters[VR].inverse = false;

    Q100.device[DEV1].address = 0x03;
    Q100.device[GRAB].address = 0x01;
    Q100.device[GRAB_ROTATION].address = 0x02;
    Q100.device[TILT].address = 0x04;
}

void uartBusesInit()
{
	// Shore UART configuration
	uartBus[SHORE_UART].huart = &huart5; // Link to huart will be set before receiving
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
	}
}

bool transmitPackage(struct uartBus_s *bus, bool isrMode)
{
	if(bus == &uartBus[SHORE_UART]) {
		return false;
	}

    bus->timeoutCounter = xTaskGetTickCount();
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

    while (!bus->packageTransmitted && !isrMode) {
    	if(xTaskGetTickCount() - bus->timeoutCounter > bus->transmitTimeout) {
    		return false;
    	}
    	osDelay(DELAY_UART_TIMEOUT);
    }
    return true;
}

bool receivePackage(struct uartBus_s *bus, bool isrMode)
{
	if(bus == &uartBus[SHORE_UART]) {
		return false;
	}

	bus->timeoutCounter = xTaskGetTickCount();
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

	while (!bus->packageReceived && !isrMode) {
		if(xTaskGetTickCount() - bus->timeoutCounter > bus->receiveTimeout) {
			return false;
		}
		osDelay(DELAY_UART_TIMEOUT);
	}
	return true;
}

bool transmitAndReceive(struct uartBus_s *bus, bool isrMode)
{
	if(bus == &uartBus[SHORE_UART]) {
		return false;
	}

	bus->timeoutCounter = xTaskGetTickCount();
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

	while (!bus->packageTransmitted && !isrMode) {
		if(xTaskGetTickCount() - bus->timeoutCounter > bus->transmitTimeout) {
			return false;
		}
		osDelay(DELAY_UART_TIMEOUT);
	}
	return true;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart1) {
		uartBus[SHORE_UART].packageTransmitted = true;
		return;
	}

	struct uartBus_s *bus = 0;
	for(uint8_t i=0; i<UART_NUMBER; i++) {
		if(uartBus[i].huart == huart) {
			bus = &uartBus[i];
			break;
		}
	}

	bus->packageTransmitted = true;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart1) {
		ShoreReceive();
		return;
	}

	struct uartBus_s *bus = 0;
	for(uint8_t i=0; i<UART_NUMBER; i++) {
		if(uartBus[i].huart == huart) {
			bus = &uartBus[i];
			break;
		}
	}

	bus->packageReceived = true;
}

void receiveI2cPackageDMA (uint8_t I2C, uint16_t addr, uint8_t *buf, uint8_t length)
{
	TickType_t timeBegin = xTaskGetTickCount();
    switch(I2C) {
        case DEV_I2C:
            HAL_I2C_Master_Receive_IT(&hi2c2, addr>>1, buf, length);
            while (!i2c1PackageReceived && xTaskGetTickCount() - timeBegin < WAITING_SENSORS) {
                osDelay(DELAY_SENSOR_TASK);
            }
            i2c1PackageReceived = false;
            break;
    }
}


void transmitI2cPackageDMA(uint8_t I2C, uint16_t addr, uint8_t *buf, uint8_t length)
{
    TickType_t timeBegin = xTaskGetTickCount();
    switch(I2C) {
        case DEV_I2C:
        	HAL_I2C_Master_Transmit_IT(&hi2c1, addr>>1, buf, length);
            while (!i2c1PackageTransmit && xTaskGetTickCount() - timeBegin < WAITING_SENSORS) {
                osDelay(DELAY_SENSOR_TASK);
            }
            i2c1PackageTransmit = false;
            break;
    }
}


void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	if(hi2c == &hi2c1) {
		i2c1PackageTransmit = true;
	}
	else if(hi2c == &hi2c2) {
		i2c2PackageTransmit = true;
	}
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	if(hi2c == &hi2c1) {
		i2c1PackageReceived = true;
	}
	else if(hi2c == &hi2c2) {
		i2c2PackageReceived  = true;
	}
}


void SensorsResponseUpdate(struct Robot *robot, uint8_t *buf, uint8_t Sensor_id)
{
	switch(Sensor_id) {
		case DEV_I2C:
			robot->sensors.pressure = FloatFromUint8(buf, 0);
			break;
	}
}

void ShoreReceive()
{
    static portBASE_TYPE xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;

    if(counterRx == 0) {
    	switch(ShoreRequestBuffer[0]) {
    	case SHORE_REQUEST_CODE:
    		xTimerStartFromISR(UARTTimer, &xHigherPriorityTaskWoken);
    		counterRx = 1;
    		HAL_UART_Receive_IT(&huart1, &ShoreRequestBuffer[1], SHORE_REQUEST_LENGTH-1);
    		break;
    	case REQUEST_CONFIG_CODE:
    		xTimerStartFromISR(UARTTimer, &xHigherPriorityTaskWoken);
    		counterRx = 1;
    		HAL_UART_Receive_IT(&huart1, &ShoreRequestBuffer[1], REQUEST_CONFIG_LENGTH-1);
    		break;
    	default:
    		HAL_UART_Receive_IT(&huart1, &ShoreRequestBuffer[0], 1);
    	}

    }
    else if(counterRx == 1) {
    	uartBus[SHORE_UART].packageReceived = true;
    	counterRx = 2;
    }

    if (xHigherPriorityTaskWoken == pdTRUE) {
    	xHigherPriorityTaskWoken = pdFALSE;
    	taskYIELD();
    }
}

void DevicesRequestUpdate(struct Robot *robot, uint8_t *buf, uint8_t dev)
{
	struct devicesRequest_s req;

    req.AA1 = 0xAA;
    req.AA2 = 0xAA;
    req.address = robot->device[dev].address;
    req.setting = robot->device[dev].settings;
    req.velocity1 = 0;
    req.velocity2 = robot->device[dev].force;

    memcpy((void*)buf, (void*)&req, DEVICES_REQUEST_LENGTH);
    AddChecksumm8b(buf, DEVICES_REQUEST_LENGTH);
}

void DevicesResponseUpdate(struct Robot *robot, uint8_t *buf, uint8_t dev)
{
    if(IsChecksumm8bCorrect(buf, DEVICES_RESPONSE_LENGTH)) {
    	struct devicesResponse_s res;
    	memcpy((void*)&res, (void*)buf, DEVICES_RESPONSE_LENGTH);

        robot->device[dev].current = res.current1;
        // TODO make errors work pls
        //writeBit(&(robot->device[dev].errors), res.errors, AGAR);

        ++uartBus[DEVICES_UART].successRxCounter;
    }
    else {
    	++uartBus[DEVICES_UART].brokenRxCounter;
    }
}

void ThrustersRequestUpdate(struct Robot *robot, uint8_t *buf, uint8_t thruster)
{
    struct thrustersRequest_s res;

    res.AA = 0xAA;
    res.type = 0x01;
    res.address = robot->thrusters[thruster].address;
    res.velocity = robot->thrusters[thruster].desiredSpeed;

    // Inverting
    if(robot->thrusters[thruster].inverse) {
    	res.velocity *= -1;
    }

    // Multiplier constants
    if(robot->thrusters[thruster].desiredSpeed > 0) {
    	res.velocity *= robot->thrusters[thruster].kForward;
    }
    else if(robot->thrusters[thruster].desiredSpeed < 0) {
    	res.velocity *= robot->thrusters[thruster].kBackward;
    }

    memcpy((void*)buf, (void*)&res, THRUSTERS_REQUEST_LENGTH);
    AddChecksumm8bVma(buf, THRUSTERS_REQUEST_LENGTH);
}

void ThrustersResponseUpdate(struct Robot *robot, uint8_t *buf, uint8_t thruster)
{
	//TODO errors parsing! and what is all this new stuff means
    if(IsChecksumm8bCorrectVma(buf, THRUSTERS_RESPONSE_LENGTH) && buf[0] != 0) {
    	struct thrustersResponse_s res;
    	memcpy((void*)&res, (void*)buf, THRUSTERS_RESPONSE_LENGTH);

        robot->thrusters[thruster].current = res.current;

        ++uartBus[THRUSTERS_UART].successRxCounter;
    }
    else {
    	++uartBus[THRUSTERS_UART].brokenRxCounter;
    }
}

void ShoreConfigRequest(struct Robot *robot, uint8_t *requestBuf)
{
    if(IsChecksumm16bCorrect(requestBuf, REQUEST_CONFIG_LENGTH)) {
    	struct shoreConfigRequest_s req;
    	memcpy((void*)&req, (void*)requestBuf, REQUEST_CONFIG_LENGTH);

        robot->joySpeed.march = req.march;
        robot->joySpeed.lag = req.lag;
        robot->joySpeed.depth = req.depth;
        robot->joySpeed.roll = req.roll;
        robot->joySpeed.pitch = req.pitch;
        robot->joySpeed.yaw = req.yaw;

        robot->stabConstants[req.contour].pJoyUnitCast = req.pJoyUnitCast;
        robot->stabConstants[req.contour].pSpeedDyn = req.pSpeedDyn;
        robot->stabConstants[req.contour].pErrGain = req.pErrGain;

        robot->stabConstants[req.contour].aFilter[POS_FILTER].T = req.posFilterT;
        robot->stabConstants[req.contour].aFilter[POS_FILTER].K = req.posFilterK;
        robot->stabConstants[req.contour].aFilter[SPEED_FILTER].T = req.speedFilterT;
        robot->stabConstants[req.contour].aFilter[SPEED_FILTER].K = req.speedFilterK;

        robot->stabConstants[req.contour].pid.pGain = req.pid_pGain;
        robot->stabConstants[req.contour].pid.iGain = req.pid_iGain;
        robot->stabConstants[req.contour].pid.iMax = req.pid_iMax;
        robot->stabConstants[req.contour].pid.iMin = req.pid_iMin;

        robot->stabConstants[req.contour].pThrustersCast = req.pThrustersCast;
        robot->stabConstants[req.contour].pThrustersMin = req.pThrustersMin;
        robot->stabConstants[req.contour].pThrustersMax = req.pThrustersMax;

        ContourSelected = req.contour;

        for(uint8_t i=0; i<STABILIZATION_AMOUNT; i++) {
        	robot->stabConstants[i].enable = false;
        }
        robot->stabConstants[req.contour].enable = true;

        ++uartBus[SHORE_UART].successRxCounter;;
    }
    else {
    	++uartBus[SHORE_UART].brokenRxCounter;
    }
}


void ShoreRequest(struct Robot *robot, uint8_t *requestBuf)
{
    if (IsChecksumm16bCorrect(requestBuf, SHORE_REQUEST_LENGTH)) {
    	struct shoreRequest_s req;
    	memcpy((void*)&req, (void*)requestBuf, SHORE_REQUEST_LENGTH);

    	uint8_t tempCameraNum = 0;

        robot->joySpeed.march = req.march;
        robot->joySpeed.lag = req.lag;
        robot->joySpeed.depth = req.depth;
        robot->joySpeed.roll = req.roll;
        robot->joySpeed.pitch = req.pitch;
        robot->joySpeed.yaw = req.yaw;

        robot->device[LIGHT].force = req.light;
        robot->device[GRAB].force = req.grab;
        if (robot->device[GRAB].force < -127) {
            robot->device[GRAB].force = -127;
        }
        robot->device[TILT].force = req.tilt;
        if (robot->device[TILT].force < -127) {
        	robot->device[TILT].force = -127;
        }
        robot->device[GRAB_ROTATION].force  = req.grab_rotate;
        if (robot->device[GRAB_ROTATION].force < -127) {
            robot->device[GRAB_ROTATION].force = -127;
        }

        robot->device[DEV1].force = req.dev1;
        robot->device[DEV2].force = req.dev2;

        bool wasEnabled;

        //TODO govnokod
        wasEnabled = robot->stabConstants[STAB_YAW].enable;
        robot->stabConstants[STAB_YAW].enable = PickBit(req.stabilize_flags, SHORE_STABILIZE_YAW_BIT);
        if(wasEnabled == false && robot->stabConstants[STAB_YAW].enable == true) {
        	stabilizationStart(STAB_YAW);
        }

        wasEnabled = robot->stabConstants[STAB_ROLL].enable;
        robot->stabConstants[STAB_ROLL].enable = PickBit(req.stabilize_flags, SHORE_STABILIZE_ROLL_BIT);
        if(wasEnabled == false && robot->stabConstants[STAB_ROLL].enable == true) {
        	stabilizationStart(STAB_ROLL);
        }

        wasEnabled = robot->stabConstants[STAB_PITCH].enable;
        robot->stabConstants[STAB_PITCH].enable = PickBit(req.stabilize_flags, SHORE_STABILIZE_PITCH_BIT);
        if(wasEnabled == false && robot->stabConstants[STAB_PITCH].enable == true) {
        	stabilizationStart(STAB_PITCH);
        }

        wasEnabled = robot->stabConstants[STAB_DEPTH].enable;
        robot->stabConstants[STAB_DEPTH].enable = PickBit(req.stabilize_flags, SHORE_STABILIZE_DEPTH_BIT);
        if(wasEnabled == false && robot->stabConstants[STAB_DEPTH].enable == true) {
        	stabilizationStart(STAB_DEPTH);
        }

        robot->sensors.resetIMU = PickBit(req.stabilize_flags, SHORE_STABILIZE_IMU_BIT);

        tempCameraNum = req.cameras;
        robot->pc.reset = req.pc_reset;

        if(robot->pc.reset == PC_ON_CODE) {
        	//HAL_GPIO_WritePin(GPIOE, RES_PC_2_Pin, GPIO_PIN_SET); // ONOFF
        }
        else if(robot->pc.reset == PC_OFF_CODE) {
        	//HAL_GPIO_WritePin(GPIOE, RES_PC_2_Pin, GPIO_PIN_RESET); // ONOFF
        }

        if(tempCameraNum != robot->cameraNum) {
        	robot->cameraNum = tempCameraNum;
        	switch(robot->cameraNum) {
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

        float bYaw, bRoll, bPitch, bDepth;
        if(robot->stabConstants[STAB_ROLL].enable) {
            bRoll = robot->stabState[STAB_ROLL].speedError;
        }
        else {
            bRoll = robot->joySpeed.roll;
        }

        if(robot->stabConstants[STAB_PITCH].enable) {
            bPitch = robot->stabState[STAB_PITCH].speedError;
        }
        else {
            bPitch = robot->joySpeed.pitch;
        }

        if(robot->stabConstants[STAB_YAW].enable) {
            bYaw = robot->stabState[STAB_YAW].speedError;
        }
        else {
            bYaw = robot->joySpeed.yaw;
        }

        if(robot->stabConstants[STAB_DEPTH].enable) {
        	bDepth = robot->stabState[STAB_DEPTH].speedError;
        }
        else {
        	bDepth = robot->joySpeed.depth;
        }

        int16_t velocity[THRUSTERS_NUMBER];
        velocity[HLB] = - robot->joySpeed.march + robot->joySpeed.lag - bYaw;
        velocity[HLF] = + robot->joySpeed.march + robot->joySpeed.lag + bYaw;
        velocity[HRB] = + robot->joySpeed.march + robot->joySpeed.lag - bYaw;
        velocity[HRF] = + robot->joySpeed.march - robot->joySpeed.lag - bYaw;
        velocity[VB] = - bDepth - bPitch;
        velocity[VF] = + bDepth - bPitch;
        velocity[VL] = - bDepth + bRoll;
        velocity[VR] = - bDepth - bRoll;

        for (uint8_t i = 0; i < THRUSTERS_NUMBER; ++i) {
            velocity[i] = (int8_t)(velocity[i] / 0xFF);
            if (velocity[i] > 127) {
                robot->thrusters[i].desiredSpeed = 127;
            }
            else if( velocity[i] > -127) {
                robot->thrusters[i].desiredSpeed = velocity[i];
            }
            else {
                robot->thrusters[i].desiredSpeed = -127;
            }
        }

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

void ShoreResponse(struct Robot *robot, uint8_t *responseBuf)
{
	struct shoreResponse_s res;

    res.roll = robot->sensors.roll;
    res.pitch = robot->sensors.pitch;
    res.yaw = robot->sensors.yaw;
    res.rollSpeed = robot->sensors.rollSpeed;
    res.pitchSpeed = robot->sensors.pitchSpeed;
    res.yawSpeed = robot->sensors.yawSpeed;

    res.pressure = robot->sensors.pressure;

    // TODO eyes bleeding again
    res.vma_current_hlb = robot->thrusters[HLB].current;
    res.vma_current_hlf = robot->thrusters[HLF].current;
    res.vma_current_hrb = robot->thrusters[HRB].current;
    res.vma_current_hrf = robot->thrusters[HRF].current;
    res.vma_current_vb = robot->thrusters[VB].current;
    res.vma_current_vf = robot->thrusters[VF].current;
    res.vma_current_vl = robot->thrusters[VL].current;
    res.vma_current_vr = robot->thrusters[VR].current;

    res.dev_current_light = robot->device[LIGHT].current;
    res.dev_current_tilt = robot->device[TILT].current;
    res.dev_current_grab = robot->device[GRAB].current;
    res.dev_current_grab_rotate = robot->device[GRAB_ROTATION].current;
    res.dev_current_dev1 = robot->device[DEV1].current;
    res.dev_current_dev2 = robot->device[DEV2].current;

    res.vma_errors = 0x55;         //!!!!!TODO!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    // TODO do this properly pls
    res.dev_errors = 0;//robot->device.errors;
    res.pc_errors = robot->pc.errors;

    memcpy((void*)responseBuf, (void*)&res, SHORE_RESPONSE_LENGTH);

    AddChecksumm16b(responseBuf, SHORE_RESPONSE_LENGTH);
}

void ShoreConfigResponse(struct Robot *robot, uint8_t *responseBuf)
{
	struct shoreConfigResponse_s res;

	res.code = REQUEST_CONFIG_CODE;

	res.roll = robot->sensors.roll;
	res.pitch = robot->sensors.pitch;
	res.yaw = robot->sensors.yaw;
	res.rollSpeed = robot->sensors.rollSpeed;
	res.pitchSpeed = robot->sensors.pitchSpeed;
	res.yawSpeed = robot->sensors.yawSpeed;

	res.pressure = robot->sensors.pressure;
	res.in_pressure = 0;

	res.inputSignal = *robot->stabState[ContourSelected].inputSignal;
	res.speedSignal = *robot->stabState[ContourSelected].speedSignal;
	res.posSignal = *robot->stabState[ContourSelected].posSignal;

	res.oldSpeed = robot->stabState[ContourSelected].oldSpeed;
	res.oldPos = robot->stabState[ContourSelected].oldPos;

	res.joyUnitCasted = robot->stabState[ContourSelected].joyUnitCasted;
	res.joy_iValue = robot->stabState[ContourSelected].joy_iValue;
	res.posError = robot->stabState[ContourSelected].posError;
	res.speedError = robot->stabState[ContourSelected].speedError;
	res.dynSummator = robot->stabState[ContourSelected].dynSummator;
	res.pidValue = robot->stabState[ContourSelected].pidValue;
	res.posErrorAmp = robot->stabState[ContourSelected].posErrorAmp;
	res.speedFiltered = robot->stabState[ContourSelected].speedFiltered;
	res.posFiltered = robot->stabState[ContourSelected].posFiltered;

	res.LastTick = robot->stabState[ContourSelected].LastTick;

	memcpy((void*)responseBuf, (void*)&res, SHORE_CONFIG_RESPONSE_LENGTH);

	AddChecksumm16b(responseBuf, SHORE_CONFIG_RESPONSE_LENGTH);
}

void ImuReceive(struct Robot *robot, uint8_t *ReceiveBuf)
{
    for(uint8_t i = 0; i < IMU_CHECKSUMS; ++i) {
        if(!IsChecksum16bSCorrect(&ReceiveBuf[i*IMU_RESPONSE_LENGTH], IMU_RESPONSE_LENGTH)) {
            ++uartBus[IMU_UART].brokenRxCounter;
            return;
        }
    }

    robot->sensors.yaw = MergeBytes(ReceiveBuf[EULER_PSI], ReceiveBuf[EULER_PSI]+1);
    robot->sensors.roll =  MergeBytes(ReceiveBuf[EULER_PHI], ReceiveBuf[EULER_PHI+1]);
    robot->sensors.pitch =  MergeBytes(ReceiveBuf[EULER_TETA], ReceiveBuf[EULER_TETA+1]);

    robot->sensors.rollSpeed = MergeBytes(ReceiveBuf[GYRO_PROC_X], ReceiveBuf[GYRO_PROC_X+1]);
    robot->sensors.pitchSpeed = MergeBytes(ReceiveBuf[GYRO_PROC_Y], ReceiveBuf[GYRO_PROC_Y+1]);
    robot->sensors.yawSpeed = MergeBytes(ReceiveBuf[GYRO_PROC_Z], ReceiveBuf[GYRO_PROC_Z+1]);

    robot->sensors.accelX = MergeBytes(ReceiveBuf[ACCEL_PROC_X], ReceiveBuf[ACCEL_PROC_X+1]) * 0.0109863;
    robot->sensors.accelY = MergeBytes(ReceiveBuf[ACCEL_PROC_Y], ReceiveBuf[ACCEL_PROC_Y+1]) * 0.0109863;
    robot->sensors.accelZ = MergeBytes(ReceiveBuf[ACCEL_PROC_Z], ReceiveBuf[ACCEL_PROC_Z+1]) * 0.0109863;

    robot->sensors.magX = MergeBytes(ReceiveBuf[MAG_PROC_X], ReceiveBuf[MAG_PROC_X+1]) * 0.000183105;
    robot->sensors.magY = MergeBytes(ReceiveBuf[MAG_PROC_Y], ReceiveBuf[MAG_PROC_Y+1]) * 0.000183105;
    robot->sensors.magZ = MergeBytes(ReceiveBuf[MAG_PROC_Z], ReceiveBuf[MAG_PROC_Z+1]) * 0.000183105;

    robot->sensors.quatA = MergeBytes(ReceiveBuf[QUAT_A], ReceiveBuf[QUAT_A+1]) * 0.0000335693;
    robot->sensors.quatB = MergeBytes(ReceiveBuf[QUAT_B], ReceiveBuf[QUAT_B+1]) * 0.0000335693;
    robot->sensors.quatC = MergeBytes(ReceiveBuf[QUAT_C], ReceiveBuf[QUAT_C+1]) * 0.0000335693;
    robot->sensors.quatD = MergeBytes(ReceiveBuf[QUAT_D], ReceiveBuf[QUAT_D+1]) * 0.0000335693;

    robot->sensors.yaw = ((double) MergeBytes(ReceiveBuf[EULER_PSI], ReceiveBuf[EULER_PSI]+1)) * 0.0109863;
    robot->sensors.roll =  ((double) MergeBytes(ReceiveBuf[EULER_PHI], ReceiveBuf[EULER_PHI]+1)) * 0.0109863;
    robot->sensors.pitch =  ((double) MergeBytes(ReceiveBuf[EULER_TETA], ReceiveBuf[EULER_TETA]+1)) * 0.0109863;

    robot->sensors.rollSpeed = ((double) MergeBytes(ReceiveBuf[GYRO_PROC_X], ReceiveBuf[GYRO_PROC_X+1])) * 0.000183105;
    robot->sensors.pitchSpeed = ((double) MergeBytes(ReceiveBuf[GYRO_PROC_Y], ReceiveBuf[GYRO_PROC_Y+1])) * 0.000183105;
    robot->sensors.yawSpeed = ((double) MergeBytes(ReceiveBuf[GYRO_PROC_Z], ReceiveBuf[GYRO_PROC_Z+1])) * 0.000183105;

    robot->sensors.accelX = ((double) MergeBytes(ReceiveBuf[ACCEL_PROC_X], ReceiveBuf[ACCEL_PROC_X+1])) * 0.0109863;
    robot->sensors.accelY = ((double) MergeBytes(ReceiveBuf[ACCEL_PROC_Y], ReceiveBuf[ACCEL_PROC_Y+1])) * 0.0109863;
    robot->sensors.accelZ = ((double) MergeBytes(ReceiveBuf[ACCEL_PROC_Z], ReceiveBuf[ACCEL_PROC_Z+1])) * 0.0109863;

    robot->sensors.magX = ((double) MergeBytes(ReceiveBuf[MAG_PROC_X], ReceiveBuf[MAG_PROC_X+1])) * 0.000183105;
    robot->sensors.magY = ((double) MergeBytes(ReceiveBuf[MAG_PROC_Y], ReceiveBuf[MAG_PROC_Y+1])) * 0.000183105;
    robot->sensors.magZ = ((double) MergeBytes(ReceiveBuf[MAG_PROC_Z], ReceiveBuf[MAG_PROC_Z+1])) * 0.000183105;

    robot->sensors.quatA = ((double) MergeBytes(ReceiveBuf[QUAT_A], ReceiveBuf[QUAT_A+1])) * 0.0000335693;
    robot->sensors.quatB = ((double) MergeBytes(ReceiveBuf[QUAT_B], ReceiveBuf[QUAT_B+1])) * 0.0000335693;
    robot->sensors.quatC = ((double) MergeBytes(ReceiveBuf[QUAT_C], ReceiveBuf[QUAT_C+1])) * 0.0000335693;
    robot->sensors.quatD = ((double) MergeBytes(ReceiveBuf[QUAT_D], ReceiveBuf[QUAT_D+1])) * 0.0000335693;

    ++uartBus[IMU_UART].successRxCounter;
}
