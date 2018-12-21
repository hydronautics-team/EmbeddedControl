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

uint16_t counterRx = 0;
uint32_t brokenRxCounter = 0;
uint32_t outdatedRxCounter = 0;
uint32_t successRxCounter = 0;
uint8_t brokenRxTolerance = 0;

const uint16_t ShoreLength[SHORE_REQUEST_MODES_NUMBER] = {SHORE_REQUEST_LENGTH, REQUEST_CONFIG_LENGTH};
const uint8_t ShoreCodes[SHORE_REQUEST_MODES_NUMBER] = {SHORE_REQUEST_CODE, REQUEST_CONFIG_CODE};

uint8_t* uartBuf[UART_NUMBER];
uint8_t uartLength[UART_NUMBER];
bool uartPackageTransmit[UART_NUMBER];
bool uartPackageReceived[UART_NUMBER];

bool i2c1PackageTransmit = false;
bool i2c2PackageTransmit = false;

bool i2c1PackageReceived = false;
bool i2c2PackageReceived = false;

void variableInit() {
    Q100.VMA[HLB].address = 6;
    Q100.VMA[HLF].address = 5;
    Q100.VMA[HRB].address = 3;
    Q100.VMA[HRF].address = 4;
    Q100.VMA[VB].address = 2;
    Q100.VMA[VF].address = 1;
    Q100.VMA[VL].address = 8;
    Q100.VMA[VR].address = 7;

    Q100.device[DEV1].address = 0x03;
    Q100.device[GRAB].address = 0x01;
    Q100.device[GRAB_ROTATION].address = 0x02;
    Q100.device[TILT].address = 0x04;

    // Pitch stabilization constants
    Q100.pitchStabCons.enable = false;
    // Before PID
    Q100.pitchStabCons.iJoySpeed = 1;
    Q100.pitchStabCons.pSpeedDyn = 1;
    Q100.pitchStabCons.pErrGain = 1;
    // PID
    Q100.pitchStabCons.pid_pGain = 1;
    Q100.pitchStabCons.pid_iGain = 1;
    Q100.pitchStabCons.pid_iMin = 0;
    Q100.pitchStabCons.pid_iMax = 32768;
    // Feedback
    Q100.pitchStabCons.pSpeedFback = 1;

    // Roll stabilization constants
    Q100.rollStabCons.enable = false;
    // Before PID
    Q100.rollStabCons.iJoySpeed = 1;
    Q100.rollStabCons.pSpeedDyn = 1;
    Q100.rollStabCons.pErrGain = 1;
    // PID
    Q100.rollStabCons.pid_pGain = 1;
    Q100.rollStabCons.pid_iGain = 1;
    Q100.rollStabCons.pid_iMin = 0;
    Q100.rollStabCons.pid_iMax = 32768;
    // Feedback
    Q100.rollStabCons.pSpeedFback = 1;

    // Yaw stabilization constants
    Q100.yawStabCons.enable = false;
    // Before PID
    Q100.yawStabCons.iJoySpeed = 1;
    Q100.yawStabCons.pSpeedDyn = 1;
    Q100.yawStabCons.pErrGain = 1;
    // PID
    Q100.yawStabCons.pid_pGain = 1;
    Q100.yawStabCons.pid_iGain = 1;
    Q100.yawStabCons.pid_iMin = 0;
    Q100.yawStabCons.pid_iMax = 32768;
    // Feedback
    Q100.yawStabCons.pSpeedFback = 1;

    // Depth stabilization constants
    Q100.depthStabCons.enable = false;
    // Before PID
    Q100.depthStabCons.iJoySpeed = 1;
    Q100.depthStabCons.pSpeedDyn = 1;
    Q100.depthStabCons.pErrGain = 1;
    // PID
    Q100.depthStabCons.pid_pGain = 1;
    Q100.depthStabCons.pid_iGain = 1;
    Q100.depthStabCons.pid_iMin = 0;
    Q100.depthStabCons.pid_iMax = 32768;
    // Feedback
    Q100.depthStabCons.pSpeedFback = 1;

    for(uint8_t i=0; i<UART_NUMBER; i++) {
    	uartBuf[i] = 0;
    	uartLength[i] = 0;
    }
}

void transmitPackage(uint8_t UART, uint8_t *buf, uint8_t length)
{
    TickType_t timeBegin = xTaskGetTickCount();

	uartPackageTransmit[UART] = false;
    switch(UART) {
        case SHORE_UART:
            HAL_UART_Transmit_IT(&huart5, buf, length);
            break;
        case VMA_UART:
            HAL_UART_Transmit_IT(&huart2, buf, length);
            break;
        case DEV_UART:
            HAL_UART_Transmit_DMA(&huart3, buf, length);
            break;
        case IMU_UART:
        	HAL_UART_Transmit_IT(&huart4, buf, length);
            break;
        default:
            	return;
    }
	// TODO different waiting time
    while (!uartPackageTransmit[UART] && xTaskGetTickCount() - timeBegin < WAITING_SHORE) {
    	osDelay(DELAY_TIMER_TASK);
    }
}

void receivePackage(uint8_t UART, uint8_t *buf, uint8_t length)
{
    TickType_t timeBegin = xTaskGetTickCount();

    uartPackageReceived[UART] = false;
    switch(UART){
    case SHORE_UART:
    	HAL_UART_Receive_IT(&huart5, buf, length);
    	break;
    case VMA_UART:
    	HAL_UART_Receive_IT(&huart2, buf, length);
    	break;
    case DEV_UART:
    	HAL_UART_Receive_DMA(&huart3, buf, length);
    	break;
    case IMU_UART:
    	HAL_UART_Receive_IT(&huart4, buf, length);
		break;
    default:
    	return;
    }
    // TODO again
    while (!uartPackageReceived[UART] && xTaskGetTickCount() - timeBegin < WAITING_SHORE) {
        osDelay(DELAY_TIMER_TASK);
    }
}

void transmitAndReceive(uint8_t UART, uint8_t *tr_buf, uint8_t tr_length, uint8_t *re_buf, uint8_t re_length)
{
	TickType_t timeBegin = xTaskGetTickCount();

	uartPackageReceived[UART] = false;
	uartPackageTransmit[UART] = false;
	switch(UART) {
	case SHORE_UART:
		HAL_UART_Transmit_IT(&huart5, tr_buf, tr_length);
		break;
	case VMA_UART:
		HAL_UART_Transmit_IT(&huart2, tr_buf, tr_length);
		break;
	case DEV_UART:
		HAL_UART_Transmit_DMA(&huart3, tr_buf, tr_length);
		break;
	case IMU_UART:
		HAL_UART_Receive_IT(&huart4, re_buf, re_length);
		HAL_UART_Transmit_IT(&huart4, tr_buf, tr_length);
		break;
	default:
	    	return;
	}

	uartBuf[UART] = re_buf;
	uartLength[UART] = re_length;
	// TODO again
	while (!uartPackageReceived[UART] && xTaskGetTickCount() - timeBegin < WAITING_SHORE) {
		osDelay(DELAY_TIMER_TASK);
	}

	if(UART == VMA_UART) {
		HAL_UART_AbortReceive_IT(&huart2);
	}
	else if(UART == IMU_UART) {
		HAL_UART_AbortReceive_IT(&huart4);
	}

	uartBuf[UART] = 0;
	uartLength[UART] = 0;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	// TODO do this smarter pls
	uint8_t UART = 0;
	if(huart == &huart5) {
		UART = SHORE_UART;
	}
	else if(huart == &huart2) {
		UART = VMA_UART;
	}
	else if(huart == &huart3) {
		UART = DEV_UART;
	}
	else if(huart == &huart4) {
		UART = IMU_UART;
	}

	uartPackageTransmit[UART] = true;
	if(uartLength[UART] > 0) {
		switch(UART) {
		case SHORE_UART:
			HAL_UART_Receive_IT(&huart5, uartBuf[UART], uartLength[UART]);
			break;
		case VMA_UART:
			HAL_UART_Receive_IT(&huart2, uartBuf[UART], uartLength[UART]);
			break;
		case DEV_UART:
			HAL_UART_Receive_IT(&huart3, uartBuf[UART], uartLength[UART]);
			break;
		case IMU_UART:

			break;
		}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart5) {
		ShoreReceive();
	}
	else if(huart == &huart2) {
		uartPackageReceived[SHORE_UART] = true;
	}
	else if(huart == &huart3) {
		uartPackageReceived[DEV_UART] = true;
	}
	else if(huart == &huart4) {
		uartPackageReceived[IMU_UART] = true;
	}
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
    		if(ShoreRequestBuf[0] == ShoreCodes[i]) {
    			xTimerStartFromISR(UARTTimer, &xHigherPriorityTaskWoken);
    			counterRx = 1;
    			HAL_UART_Receive_IT(&huart5, &ShoreRequestBuf[1], ShoreLength[i]-1);
    			break;
    		}

    		if(i == SHORE_REQUEST_MODES_NUMBER-1) {
    			HAL_UART_Receive_IT(&huart5, &ShoreRequestBuf[0], 1);
    		}
    	}
    }
    else if(counterRx == 1) {
    	uartPackageReceived[SHORE_UART] = true;
    	counterRx = 2;
    }

    if (xHigherPriorityTaskWoken == pdTRUE) {
    	xHigherPriorityTaskWoken = pdFALSE;
    	taskYIELD();
    }
}

void DevRequestUpdate(struct Robot *robot, uint8_t *buf, uint8_t dev)
{
	struct devRequest_s req;

    req.AA1 = 0xAA;
    req.AA2 = 0xAA;
    req.address = robot->device[dev].address;
    req.setting = robot->device[dev].settings;
    req.velocity1 = 0;
    req.velocity2 = robot->device[dev].force;

    memcpy((void*)buf, (void*)&req, DEV_REQUEST_LENGTH);
    AddChecksumm8b(buf, DEV_REQUEST_LENGTH);
}

void DevResponseUpdate(struct Robot *robot, uint8_t *buf, uint8_t dev)
{
    if(IsChecksumm8bCorrect(buf, DEV_RESPONSE_LENGTH)) {
    	struct devResponse_s res;
    	memcpy((void*)&res, (void*)buf, DEV_RESPONSE_LENGTH);

        robot->device[dev].current = res.current1;
        // TODO make errors work pls
        //writeBit(&(robot->device[dev].errors), res.errors, AGAR);
    }
}

void VmaRequestUpdate(struct Robot *robot, uint8_t *buf, uint8_t vma)
{
    struct vmaRequest_s res;

    res.AA = 0xAA;
    res.type = 0x01;
    res.address = robot->VMA[vma].address;
    res.velocity = robot->VMA[vma].desiredSpeed*0.4;

    memcpy((void*)buf, (void*)&res, VMA_REQUEST_LENGTH);
    AddChecksumm8bVma(buf, VMA_REQUEST_LENGTH);
}

void VmaResponseUpdate(struct Robot *robot, uint8_t *buf, uint8_t vma)
{
	//TODO errors parsing! and what is all this new stuff means
    if(IsChecksumm8bCorrectVma(buf, VMA_RESPONSE_LENGTH) && buf[0] != 0) {
    	struct vmaResponse_s res;
    	memcpy((void*)&res, (void*)buf, VMA_RESPONSE_LENGTH);

        robot->VMA[vma].current = res.current;
    }
}

void ShoreConfigRequest(struct Robot *robot, uint8_t *requestBuf)
{
    if(IsChecksumm16bCorrect(requestBuf, REQUEST_CONFIG_LENGTH)) {
    	struct shoreConfigRequest_s req;
    	memcpy((void*)&req, (void*)requestBuf, SHORE_REQUEST_LENGTH);

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

        robot->VMA[HLB].address = req.position_hlb;
        robot->VMA[HLF].address = req.position_hlf;
        robot->VMA[HRB].address = req.position_hrb;
        robot->VMA[HRF].address = req.position_hrf;
        robot->VMA[VB].address = req.position_vb;
        robot->VMA[VF].address = req.position_vf;
        robot->VMA[VL].address = req.position_vl;
        robot->VMA[VR].address = req.position_vr;

        robot->VMA[HLB].settings = req.setting_hlb;
        robot->VMA[HLF].settings = req.setting_hlf;
        robot->VMA[HRB].settings = req.setting_hrb;
        robot->VMA[HRF].settings = req.setting_hrf;
        robot->VMA[VB].settings = req.setting_vb;
        robot->VMA[VF].settings = req.setting_vf;
        robot->VMA[VL].settings = req.setting_vl;
        robot->VMA[VR].settings = req.setting_vr;

        robot->VMA[HLB].kForward = req.kforward_hlb;
        robot->VMA[HLF].kForward = req.kforward_hlf;
        robot->VMA[HRB].kForward = req.kforward_hrb;
        robot->VMA[HRF].kForward = req.kforward_hrf;
        robot->VMA[VB].kForward = req.kforward_vb;
        robot->VMA[VF].kForward = req.kforward_vf;
        robot->VMA[VL].kForward = req.kforward_vl;
        robot->VMA[VR].kForward = req.kforward_vr;

        robot->VMA[HLB].kBackward = req.kbackward_hlb;
        robot->VMA[HLF].kBackward = req.kbackward_hlf;
        robot->VMA[HRB].kBackward = req.kbackward_hrb;
        robot->VMA[HRF].kBackward = req.kbackward_hrf;
        robot->VMA[VB].kBackward = req.kbackward_vb;
        robot->VMA[VF].kBackward = req.kbackward_vf;
        robot->VMA[VL].kBackward = req.kbackward_vl;
        robot->VMA[VR].kBackward = req.kbackward_vr;

        ++successRxCounter;
    }
    else {
    	++brokenRxCounter;
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
        robot->depthStabCons.enable = PickBit(req.stabilize_flags, SHORE_STABILIZE_YAW_BIT);
        robot->rollStabCons.enable = PickBit(req.stabilize_flags, SHORE_STABILIZE_ROLL_BIT);
        robot->pitchStabCons.enable = PickBit(req.stabilize_flags, SHORE_STABILIZE_PITCH_BIT);
        robot->yawStabCons.enable = PickBit(req.stabilize_flags, SHORE_STABILIZE_DEPTH_BIT);
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
        if(robot->rollStabCons.enable) {
            bRoll = (int16_t) robot->rollStabSt.speedError*50;
        }
        else {
            bRoll = robot->i_joySpeed.roll;
        }

        if(robot->pitchStabCons.enable) {
            bPitch = (int16_t) robot->pitchStabSt.speedError*50;
        }
        else {
            bPitch = robot->i_joySpeed.pitch;
        }

        if(robot->yawStabCons.enable) {
            bYaw = (int16_t) robot->yawStabSt.speedError*50;
        }
        else {
            bYaw = robot->i_joySpeed.yaw;
        }

        int16_t velocity[VMA_DRIVER_NUMBER];
        velocity[HLB] = - robot->i_joySpeed.march + robot->i_joySpeed.lag - bYaw;
        velocity[HLF] = + robot->i_joySpeed.march + robot->i_joySpeed.lag + bYaw;
        velocity[HRB] = + robot->i_joySpeed.march + robot->i_joySpeed.lag - bYaw;
        velocity[HRF] = + robot->i_joySpeed.march - robot->i_joySpeed.lag - bYaw;
        velocity[VB] = - robot->i_joySpeed.depth - bPitch;
        velocity[VF] = + robot->i_joySpeed.depth - bPitch;
        velocity[VL] = - robot->i_joySpeed.depth + bRoll;
        velocity[VR] = - robot->i_joySpeed.depth - bRoll;

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

        ++successRxCounter;
    }
    else {
        ++brokenRxCounter;
        ++brokenRxTolerance;

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

        	for (uint8_t i = 0; i < VMA_DRIVER_NUMBER; ++i){
        		robot->VMA[i].desiredSpeed = 0;
        	}

        	brokenRxTolerance = 0;
        }
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

    res.vma_current_hlb = robot->VMA[HLB].current;
    res.vma_current_hlf = robot->VMA[HLF].current;
    res.vma_current_hrb = robot->VMA[HRB].current;
    res.vma_current_hrf = robot->VMA[HRF].current;
    res.vma_current_vb = robot->VMA[VB].current;
    res.vma_current_vf = robot->VMA[VF].current;
    res.vma_current_vl = robot->VMA[VL].current;
    res.vma_current_vr = robot->VMA[VR].current;

    res.vma_velocity_hlb = robot->VMA[HLB].desiredSpeed;
    res.vma_velocity_hlf = robot->VMA[HLF].desiredSpeed;
    res.vma_velocity_hrb = robot->VMA[HRB].desiredSpeed;
    res.vma_velocity_hrf = robot->VMA[HRF].desiredSpeed;
    res.vma_velocity_vb = robot->VMA[VB].desiredSpeed;
    res.vma_velocity_vf = robot->VMA[VF].desiredSpeed;
    res.vma_velocity_vl = robot->VMA[VL].desiredSpeed;
    res.vma_velocity_vr = robot->VMA[VR].desiredSpeed;

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


void ImuReceive(struct Robot *robot, uint8_t *ReceiveBuf, uint8_t *ErrCode)
{
    for(uint8_t i = 0; i < IMU_CHECKSUMS; ++i) {
        if(!IsChecksum16bSCorrect(&ReceiveBuf[i*IMU_RESPONSE_LENGTH], IMU_RESPONSE_LENGTH)) {
            *ErrCode = 1;
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
}
