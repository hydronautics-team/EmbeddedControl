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
	uartBus[SHORE_UART].huart = 0; // Link to huart will be set before receiving
	uartBus[SHORE_UART].rxBuffer = ShoreRequestBuffer;
	uartBus[SHORE_UART].txBuffer = ShoreResponseBuffer;
	uartBus[SHORE_UART].rxLength = 0; // Length of the received message will be determined when first byte will be received
	uartBus[SHORE_UART].txLength = 0; // Length of the transmitted message will be determined before transmit
	uartBus[SHORE_UART].brokenRxTolerance = 20;
	uartBus[SHORE_UART].timeoutRxTolerance = 500;
	uartBus[SHORE_UART].receiveTimeout = 100;
	uartBus[SHORE_UART].transmitTimeout = 100;
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
	if(huart == &huart5) {
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
        case PC_I2C:
        	// TODO what is this?
            HAL_I2C_Master_Receive_DMA(&hi2c2, SENSORS_PRESSURE_ADDR, buf, length);
            while (!i2c2PackageReceived  && xTaskGetTickCount() - timeBegin < WAITING_PC) {
                osDelay(DELAY_PC_TASK);
            }
            i2c2PackageReceived = false;
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
			robot->f_sensors.pressure = FloatFromUint8(buf, 0);
			break;
	}
}

void ShoreReceive()
{
    static portBASE_TYPE xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;

    if(counterRx == 0) {
    	for(uint8_t i=0; i<SHORE_REQUEST_MODES_NUMBER; ++i) {
    		if(ShoreRequestBuffer[0] == ShoreCodes[i]) {
    			xTimerStartFromISR(UARTTimer, &xHigherPriorityTaskWoken);
    			counterRx = 1;
    			HAL_UART_Receive_IT(&huart5, &ShoreRequestBuffer[1], ShoreLength[i]-1);
    			break;
    		}

    		if(i == SHORE_REQUEST_MODES_NUMBER-1) {
    			HAL_UART_Receive_IT(&huart5, &ShoreRequestBuffer[0], 1);
    		}
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
    	memcpy((void*)&req, (void*)requestBuf, SHORE_REQUEST_LENGTH);
    	// TODO my eyes are bleeding, really
    	/*
        robot->depthStabCons.iJoySpeed = req.depth_k1;
        robot->depthStabCons.pSpeedDyn = req.depth_k2;
        robot->depthStabCons.pErrGain = req.depth_k3;
        robot->depthStabCons.pSpeedFback = req.depth_k4;
        robot->depthStabCons.pid_iMax = req.depth_iborders;
        robot->depthStabCons.pid_iMin = -req.depth_iborders;
        robot->depthStabCons.pid_pGain = req.depth_pgain;
        robot->depthStabCons.pid_iGain = req.depth_igain;

        robot->rollStabCons.iJoySpeed = req.roll_k1;
        robot->rollStabCons.pSpeedDyn = req.roll_k2;
        robot->rollStabCons.pErrGain = req.roll_k3;
        robot->rollStabCons.pSpeedFback = req.roll_k4;
        robot->rollStabCons.pid_iMax = req.roll_iborders;
        robot->rollStabCons.pid_iMin = -req.roll_iborders;
        robot->rollStabCons.pid_pGain = req.roll_pgain;
        robot->rollStabCons.pid_iGain = req.roll_igain;

        robot->pitchStabCons.iJoySpeed = req.pitch_k1;
        robot->pitchStabCons.pSpeedDyn = req.pitch_k2;
        robot->pitchStabCons.pErrGain = req.pitch_k3;
        robot->pitchStabCons.pSpeedFback = req.pitch_k4;
        robot->pitchStabCons.pid_iMax = req.pitch_iborders;
        robot->pitchStabCons.pid_iMin = -req.pitch_iborders;
        robot->pitchStabCons.pid_pGain = req.pitch_pgain;
        robot->pitchStabCons.pid_iGain = req.pitch_igain;

        robot->yawStabCons.iJoySpeed = req.yaw_k1;
        robot->yawStabCons.pSpeedDyn = req.yaw_k2;
        robot->yawStabCons.pErrGain = req.yaw_k3;
        robot->yawStabCons.pSpeedFback = req.yaw_k4;
        robot->yawStabCons.pid_iMax = req.yaw_iborders;
        robot->yawStabCons.pid_iMin = -req.yaw_iborders;
        robot->yawStabCons.pid_pGain = req.yaw_pgain;
        robot->yawStabCons.pid_iGain = req.yaw_igain;

        robot->thrusters[HLB].address = req.position_hlb;
        robot->thrusters[HLF].address = req.position_hlf;
        robot->thrusters[HRB].address = req.position_hrb;
        robot->thrusters[HRF].address = req.position_hrf;
        robot->thrusters[VB].address = req.position_vb;
        robot->thrusters[VF].address = req.position_vf;
        robot->thrusters[VL].address = req.position_vl;
        robot->thrusters[VR].address = req.position_vr;

        robot->thrusters[HLB].settings = req.setting_hlb;
        robot->thrusters[HLF].settings = req.setting_hlf;
        robot->thrusters[HRB].settings = req.setting_hrb;
        robot->thrusters[HRF].settings = req.setting_hrf;
        robot->thrusters[VB].settings = req.setting_vb;
        robot->thrusters[VF].settings = req.setting_vf;
        robot->thrusters[VL].settings = req.setting_vl;
        robot->thrusters[VR].settings = req.setting_vr;

        robot->thrusters[HLB].kForward = req.kforward_hlb;
        robot->thrusters[HLF].kForward = req.kforward_hlf;
        robot->thrusters[HRB].kForward = req.kforward_hrb;
        robot->thrusters[HRF].kForward = req.kforward_hrf;
        robot->thrusters[VB].kForward = req.kforward_vb;
        robot->thrusters[VF].kForward = req.kforward_vf;
        robot->thrusters[VL].kForward = req.kforward_vl;
        robot->thrusters[VR].kForward = req.kforward_vr;

        robot->thrusters[HLB].kBackward = req.kbackward_hlb;
        robot->thrusters[HLF].kBackward = req.kbackward_hlf;
        robot->thrusters[HRB].kBackward = req.kbackward_hrb;
        robot->thrusters[HRF].kBackward = req.kbackward_hrf;
        robot->thrusters[VB].kBackward = req.kbackward_vb;
        robot->thrusters[VF].kBackward = req.kbackward_vf;
        robot->thrusters[VL].kBackward = req.kbackward_vl;
        robot->thrusters[VR].kBackward = req.kbackward_vr;
		*/
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

        robot->i_joySpeed.march = req.march;
        robot->i_joySpeed.lag = req.lag;
        robot->i_joySpeed.depth = req.depth;
        robot->i_joySpeed.roll = req.roll;
        robot->i_joySpeed.pitch = req.pitch;
        robot->i_joySpeed.yaw = req.yaw;

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

        // TODO POMENYAL MESTAMI YAW I DEPTH
        robot->stabConstants[STAB_DEPTH].enable = PickBit(req.stabilize_flags, SHORE_STABILIZE_YAW_BIT);
        robot->stabConstants[STAB_ROLL].enable = PickBit(req.stabilize_flags, SHORE_STABILIZE_ROLL_BIT);
        robot->stabConstants[STAB_PITCH].enable = PickBit(req.stabilize_flags, SHORE_STABILIZE_PITCH_BIT);
        robot->stabConstants[STAB_YAW].enable = PickBit(req.stabilize_flags, SHORE_STABILIZE_DEPTH_BIT);
        robot->i_sensors.resetIMU = PickBit(req.stabilize_flags, SHORE_STABILIZE_IMU_BIT);

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

        // TODO KOEF OSHIBOK VRUCHNUU POMENYAL
        int16_t bYaw, bRoll, bPitch;
        if(robot->stabConstants[STAB_ROLL].enable) {
            bRoll = (int16_t) robot->stabState[STAB_ROLL].speedError*50;
        }
        else {
            bRoll = robot->i_joySpeed.roll;
        }

        if(robot->stabConstants[STAB_PITCH].enable) {
            bPitch = (int16_t) robot->stabState[STAB_PITCH].speedError*50;
        }
        else {
            bPitch = robot->i_joySpeed.pitch;
        }

        if(robot->stabConstants[STAB_YAW].enable) {
            bYaw = (int16_t) robot->stabState[STAB_YAW].speedError*50;
        }
        else {
            bYaw = robot->i_joySpeed.yaw;
        }

        int16_t velocity[THRUSTERS_NUMBER];
        velocity[HLB] = - robot->i_joySpeed.march + robot->i_joySpeed.lag - bYaw;
        velocity[HLF] = + robot->i_joySpeed.march + robot->i_joySpeed.lag + bYaw;
        velocity[HRB] = + robot->i_joySpeed.march + robot->i_joySpeed.lag - bYaw;
        velocity[HRF] = + robot->i_joySpeed.march - robot->i_joySpeed.lag - bYaw;
        velocity[VB] = - robot->i_joySpeed.depth - bPitch;
        velocity[VF] = + robot->i_joySpeed.depth - bPitch;
        velocity[VL] = - robot->i_joySpeed.depth + bRoll;
        velocity[VR] = - robot->i_joySpeed.depth - bRoll;

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

    res.roll = robot->i_sensors.roll;
    res.pitch = robot->i_sensors.pitch;
    res.yaw = robot->i_sensors.yaw;
    res.rollSpeed = robot->i_sensors.rollSpeed;
    res.pitchSpeed = robot->i_sensors.pitchSpeed;
    res.yawSpeed = robot->i_sensors.yawSpeed;

    res.pressure = robot->i_sensors.pressure;

    res.wf_type = robot->wifi.type;
    res.wf_tickrate = robot->wifi.tickrate;
    res.wf_voltage = robot->wifi.voltage;
    res.wf_x = robot->wifi.x;
    res.wf_y = robot->wifi.y;

    SetBit(&res.dev_state, SHORE_DEVICE_AC_BIT, robot->logdevice[ACOUSTIC].state);
    // TODO other devices

    res.wf_y = robot->i_sensors.leak;
    res.wf_y = robot->i_sensors.in_pressure;
    // TODO eyes bleeding again
    res.vma_current_hlb = robot->thrusters[HLB].current;
    res.vma_current_hlf = robot->thrusters[HLF].current;
    res.vma_current_hrb = robot->thrusters[HRB].current;
    res.vma_current_hrf = robot->thrusters[HRF].current;
    res.vma_current_vb = robot->thrusters[VB].current;
    res.vma_current_vf = robot->thrusters[VF].current;
    res.vma_current_vl = robot->thrusters[VL].current;
    res.vma_current_vr = robot->thrusters[VR].current;

    res.vma_velocity_hlb = robot->thrusters[HLB].desiredSpeed;
    res.vma_velocity_hlf = robot->thrusters[HLF].desiredSpeed;
    res.vma_velocity_hrb = robot->thrusters[HRB].desiredSpeed;
    res.vma_velocity_hrf = robot->thrusters[HRF].desiredSpeed;
    res.vma_velocity_vb = robot->thrusters[VB].desiredSpeed;
    res.vma_velocity_vf = robot->thrusters[VF].desiredSpeed;
    res.vma_velocity_vl = robot->thrusters[VL].desiredSpeed;
    res.vma_velocity_vr = robot->thrusters[VR].desiredSpeed;

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

void ImuReceive(struct Robot *robot, uint8_t *ReceiveBuf)
{
    for(uint8_t i = 0; i < IMU_CHECKSUMS; ++i) {
        if(!IsChecksum16bSCorrect(&ReceiveBuf[i*IMU_RESPONSE_LENGTH], IMU_RESPONSE_LENGTH)) {
            ++uartBus[IMU_UART].brokenRxCounter;
            return;
        }
    }

    robot->i_sensors.yaw = MergeBytes(ReceiveBuf[EULER_PSI], ReceiveBuf[EULER_PSI]+1);
    robot->i_sensors.roll =  MergeBytes(ReceiveBuf[EULER_PHI], ReceiveBuf[EULER_PHI+1]);
    robot->i_sensors.pitch =  MergeBytes(ReceiveBuf[EULER_TETA], ReceiveBuf[EULER_TETA+1]);

    robot->i_sensors.rollSpeed = MergeBytes(ReceiveBuf[GYRO_PROC_X], ReceiveBuf[GYRO_PROC_X+1]);
    robot->i_sensors.pitchSpeed = MergeBytes(ReceiveBuf[GYRO_PROC_Y], ReceiveBuf[GYRO_PROC_Y+1]);
    robot->i_sensors.yawSpeed = MergeBytes(ReceiveBuf[GYRO_PROC_Z], ReceiveBuf[GYRO_PROC_Z+1]);

    robot->i_sensors.accelX = MergeBytes(ReceiveBuf[ACCEL_PROC_X], ReceiveBuf[ACCEL_PROC_X+1]) * 0.0109863;
    robot->i_sensors.accelY = MergeBytes(ReceiveBuf[ACCEL_PROC_Y], ReceiveBuf[ACCEL_PROC_Y+1]) * 0.0109863;
    robot->i_sensors.accelZ = MergeBytes(ReceiveBuf[ACCEL_PROC_Z], ReceiveBuf[ACCEL_PROC_Z+1]) * 0.0109863;

    robot->i_sensors.magX = MergeBytes(ReceiveBuf[MAG_PROC_X], ReceiveBuf[MAG_PROC_X+1]) * 0.000183105;
    robot->i_sensors.magY = MergeBytes(ReceiveBuf[MAG_PROC_Y], ReceiveBuf[MAG_PROC_Y+1]) * 0.000183105;
    robot->i_sensors.magZ = MergeBytes(ReceiveBuf[MAG_PROC_Z], ReceiveBuf[MAG_PROC_Z+1]) * 0.000183105;

    robot->i_sensors.quatA = MergeBytes(ReceiveBuf[QUAT_A], ReceiveBuf[QUAT_A+1]) * 0.0000335693;
    robot->i_sensors.quatB = MergeBytes(ReceiveBuf[QUAT_B], ReceiveBuf[QUAT_B+1]) * 0.0000335693;
    robot->i_sensors.quatC = MergeBytes(ReceiveBuf[QUAT_C], ReceiveBuf[QUAT_C+1]) * 0.0000335693;
    robot->i_sensors.quatD = MergeBytes(ReceiveBuf[QUAT_D], ReceiveBuf[QUAT_D+1]) * 0.0000335693;

    robot->f_sensors.yaw = ((double) MergeBytes(ReceiveBuf[EULER_PSI], ReceiveBuf[EULER_PSI]+1)) * 0.0109863;
    robot->f_sensors.roll =  ((double) MergeBytes(ReceiveBuf[EULER_PHI], ReceiveBuf[EULER_PHI]+1)) * 0.0109863;
    robot->f_sensors.pitch =  ((double) MergeBytes(ReceiveBuf[EULER_TETA], ReceiveBuf[EULER_TETA]+1)) * 0.0109863;

    robot->f_sensors.rollSpeed = ((double) MergeBytes(ReceiveBuf[GYRO_PROC_X], ReceiveBuf[GYRO_PROC_X+1])) * 0.000183105;
    robot->f_sensors.pitchSpeed = ((double) MergeBytes(ReceiveBuf[GYRO_PROC_Y], ReceiveBuf[GYRO_PROC_Y+1])) * 0.000183105;
    robot->f_sensors.yawSpeed = ((double) MergeBytes(ReceiveBuf[GYRO_PROC_Z], ReceiveBuf[GYRO_PROC_Z+1])) * 0.000183105;

    robot->f_sensors.accelX = ((double) MergeBytes(ReceiveBuf[ACCEL_PROC_X], ReceiveBuf[ACCEL_PROC_X+1])) * 0.0109863;
    robot->f_sensors.accelY = ((double) MergeBytes(ReceiveBuf[ACCEL_PROC_Y], ReceiveBuf[ACCEL_PROC_Y+1])) * 0.0109863;
    robot->f_sensors.accelZ = ((double) MergeBytes(ReceiveBuf[ACCEL_PROC_Z], ReceiveBuf[ACCEL_PROC_Z+1])) * 0.0109863;

    robot->f_sensors.magX = ((double) MergeBytes(ReceiveBuf[MAG_PROC_X], ReceiveBuf[MAG_PROC_X+1])) * 0.000183105;
    robot->f_sensors.magY = ((double) MergeBytes(ReceiveBuf[MAG_PROC_Y], ReceiveBuf[MAG_PROC_Y+1])) * 0.000183105;
    robot->f_sensors.magZ = ((double) MergeBytes(ReceiveBuf[MAG_PROC_Z], ReceiveBuf[MAG_PROC_Z+1])) * 0.000183105;

    robot->f_sensors.quatA = ((double) MergeBytes(ReceiveBuf[QUAT_A], ReceiveBuf[QUAT_A+1])) * 0.0000335693;
    robot->f_sensors.quatB = ((double) MergeBytes(ReceiveBuf[QUAT_B], ReceiveBuf[QUAT_B+1])) * 0.0000335693;
    robot->f_sensors.quatC = ((double) MergeBytes(ReceiveBuf[QUAT_C], ReceiveBuf[QUAT_C+1])) * 0.0000335693;
    robot->f_sensors.quatD = ((double) MergeBytes(ReceiveBuf[QUAT_D], ReceiveBuf[QUAT_D+1])) * 0.0000335693;

    ++uartBus[IMU_UART].successRxCounter;
}
