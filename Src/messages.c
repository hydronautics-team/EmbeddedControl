#include "messages.h"

void DevRequestUpdate(struct Robot *robot, uint8_t *buf, uint8_t DEV)
{
	buf[VMA_DEV_REQUEST_AA1] = 0xAA;
	buf[VMA_DEV_REQUEST_AA2] = 0xAA;
	
	switch(DEV){
		case AGAR:
			buf[VMA_DEV_REQUEST_ADDRESS] = 0x01;
			buf[VMA_DEV_REQUEST_SETTING] = 0x55;
			buf[VMA_DEV_REQUEST_VELOCITY1] = 0x55;
			buf[VMA_DEV_REQUEST_VELOCITY2] = 0xAA;
			break;
		case GRUB:
			buf[VMA_DEV_REQUEST_ADDRESS] = 0x02;
			buf[VMA_DEV_REQUEST_SETTING] = 0xAA;
			buf[VMA_DEV_REQUEST_VELOCITY1] = 0x55;
			buf[VMA_DEV_REQUEST_VELOCITY2] = 0xAA;
			break;
		case GRUBROTATION:
			buf[VMA_DEV_REQUEST_ADDRESS] = 0x03;
			buf[VMA_DEV_REQUEST_SETTING] = 0xAA;
			buf[VMA_DEV_REQUEST_VELOCITY1] = 0x55;
			buf[VMA_DEV_REQUEST_VELOCITY2] = 0xAA;
			break;
		case TILT:
			buf[VMA_DEV_REQUEST_ADDRESS] = 0x04;
			buf[VMA_DEV_REQUEST_SETTING] = 0xAA;
			buf[VMA_DEV_REQUEST_VELOCITY1] = 0x55;
			buf[VMA_DEV_REQUEST_VELOCITY2] = 0xAA;
			break;
	}
	addCheckSumm8b(buf, VMA_DEV_REQUEST_LENGTH);
}



void VMARequestUpdate(struct Robot *robot, uint8_t *buf, uint8_t DEV)
{
	buf[VMA_DEV_REQUEST_AA1] = 0xAA;
	buf[VMA_DEV_REQUEST_AA2] = 0xAA;
	
	switch(DEV){
		case HLF:
			buf[VMA_DEV_REQUEST_ADDRESS] = robot->VMA[HLF].address;
			buf[VMA_DEV_REQUEST_SETTING] = robot->VMA[HLF].settings;
			buf[VMA_DEV_REQUEST_VELOCITY1] = robot->VMA[HLF].speed;
			buf[VMA_DEV_REQUEST_VELOCITY2] = robot->device.bluetoothLight;
			break;
		
		case HLB:
			buf[VMA_DEV_REQUEST_ADDRESS] = robot->VMA[HLB].address;
			buf[VMA_DEV_REQUEST_SETTING] = robot->VMA[HLB].settings;
			buf[VMA_DEV_REQUEST_VELOCITY1] = robot->VMA[HLB].speed;
			buf[VMA_DEV_REQUEST_VELOCITY2] = robot->device.bottomLight;
			break;
		
		case HRB:
			buf[VMA_DEV_REQUEST_ADDRESS] = robot->VMA[HRB].address;
			buf[VMA_DEV_REQUEST_SETTING] = robot->VMA[HRB].settings;
			buf[VMA_DEV_REQUEST_VELOCITY1] = robot->VMA[HRB].speed;
			buf[VMA_DEV_REQUEST_VELOCITY2] = NULL;
			break;
		
		case HRF:
			buf[VMA_DEV_REQUEST_ADDRESS] = robot->VMA[HRF].address;
			buf[VMA_DEV_REQUEST_SETTING] = robot->VMA[HRF].settings;
			buf[VMA_DEV_REQUEST_VELOCITY1] = robot->VMA[HRF].speed;
			buf[VMA_DEV_REQUEST_VELOCITY2] = NULL;
			break;
		
		case VL:
			buf[VMA_DEV_REQUEST_ADDRESS] = robot->VMA[VL].address;
			buf[VMA_DEV_REQUEST_SETTING] = robot->VMA[VL].settings;
			buf[VMA_DEV_REQUEST_VELOCITY1] = robot->VMA[VL].speed;
			buf[VMA_DEV_REQUEST_VELOCITY2] = NULL;
			break;
		
		case VB:
			buf[VMA_DEV_REQUEST_ADDRESS] = robot->VMA[VB].address;
			buf[VMA_DEV_REQUEST_SETTING] = robot->VMA[VB].settings;
			buf[VMA_DEV_REQUEST_VELOCITY1] = robot->VMA[VB].speed;
			buf[VMA_DEV_REQUEST_VELOCITY2] = NULL;
			break;
		
		case VR:
			buf[VMA_DEV_REQUEST_ADDRESS] = robot->VMA[VR].address;
			buf[VMA_DEV_REQUEST_SETTING] = robot->VMA[VR].settings;
			buf[VMA_DEV_REQUEST_VELOCITY1] = robot->VMA[VR].speed;
			buf[VMA_DEV_REQUEST_VELOCITY2] = NULL;
			break;
		
		case VF:
			buf[VMA_DEV_REQUEST_ADDRESS] = robot->VMA[VF].address;
			buf[VMA_DEV_REQUEST_SETTING] = robot->VMA[VF].settings;
			buf[VMA_DEV_REQUEST_VELOCITY1] = robot->VMA[VF].speed;
			buf[VMA_DEV_REQUEST_VELOCITY2] = NULL;
			break;
	}
	addCheckSumm8b(buf, VMA_DEV_REQUEST_LENGTH);
}

float floatFromUint8(uint8_t *buff, uint8_t high_byte_pos)
{
  float result;
  result = (float)((buff[high_byte_pos] << 24) + (buff[high_byte_pos + 1] << 16) + (buff[high_byte_pos + 2] << 8) + buff[high_byte_pos + 3]);
  return result;
}



void ShoreConfigRequest(struct Robot *robot, uint8_t *requestBuf)
{
  robot->depthStabilization.constTime = requestBuf[REQUEST_CONFIG_CONST_TIME_DEPTH];
  robot->depthStabilization.K1 = floatFromUint8(requestBuf, REQUEST_CONFIG_K1_DEPTH);
  robot->depthStabilization.K2 = floatFromUint8(requestBuf, REQUEST_CONFIG_K2_DEPTH);
  robot->depthStabilization.startValue = floatFromUint8(requestBuf, REQUEST_CONFIG_START_DEPTH);
  robot->depthStabilization.gain = floatFromUint8(requestBuf, REQUEST_CONFIG_GAIN_DEPTH);
  
  robot->rollStabilization.constTime = requestBuf[REQUEST_CONFIG_CONST_TIME_ROLL];
  robot->rollStabilization.K1 = floatFromUint8(requestBuf, REQUEST_CONFIG_K1_ROLL);
  robot->rollStabilization.K2 = floatFromUint8(requestBuf, REQUEST_CONFIG_K2_ROLL);
  robot->rollStabilization.startValue = floatFromUint8(requestBuf, REQUEST_CONFIG_START_ROLL);
  robot->rollStabilization.gain = floatFromUint8(requestBuf, REQUEST_CONFIG_GAIN_ROLL);
  
  robot->pitchStabilization.constTime = requestBuf[REQUEST_CONFIG_CONST_TIME_PITCH];
  robot->pitchStabilization.K1 = floatFromUint8(requestBuf, REQUEST_CONFIG_K1_PITCH);
  robot->pitchStabilization.K2 = floatFromUint8(requestBuf, REQUEST_CONFIG_K2_PITCH);
  robot->pitchStabilization.startValue = floatFromUint8(requestBuf, REQUEST_CONFIG_START_PITCH);
  robot->pitchStabilization.gain = floatFromUint8(requestBuf, REQUEST_CONFIG_GAIN_PITCH);
  
  robot->yawStabilization.constTime = requestBuf[REQUEST_CONFIG_CONST_TIME_YAW];
  robot->yawStabilization.K1 = floatFromUint8(requestBuf, REQUEST_CONFIG_K1_YAW);
  robot->yawStabilization.K2 = floatFromUint8(requestBuf, REQUEST_CONFIG_K2_YAW);
  robot->yawStabilization.startValue = floatFromUint8(requestBuf, REQUEST_CONFIG_START_YAW);
  robot->yawStabilization.gain = floatFromUint8(requestBuf, REQUEST_CONFIG_GAIN_YAW);
  
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
  
  robot->VMA[HLB].kForward = floatFromUint8(requestBuf, REQUEST_CONFIG_K_FORWARD_HLB);
  robot->VMA[HLF].kForward = floatFromUint8(requestBuf, REQUEST_CONFIG_K_FORWARD_HLF);
  robot->VMA[HRB].kForward = floatFromUint8(requestBuf, REQUEST_CONFIG_K_FORWARD_HRB);
  robot->VMA[HRF].kForward = floatFromUint8(requestBuf, REQUEST_CONFIG_K_FORWARD_HRF);
  robot->VMA[VB].kForward = floatFromUint8(requestBuf, REQUEST_CONFIG_K_FORWARD_VB);
  robot->VMA[VF].kForward = floatFromUint8(requestBuf, REQUEST_CONFIG_K_FORWARD_VF);
  robot->VMA[VL].kForward = floatFromUint8(requestBuf, REQUEST_CONFIG_K_FORWARD_VL);
  robot->VMA[VR].kForward = floatFromUint8(requestBuf, REQUEST_CONFIG_K_FORWARD_VR);
  
  robot->VMA[HLB].kBackward = floatFromUint8(requestBuf, REQUEST_CONFIG_K_FORWARD_HLB);
  robot->VMA[HLF].kBackward = floatFromUint8(requestBuf, REQUEST_CONFIG_K_FORWARD_HLF);
  robot->VMA[HRB].kBackward = floatFromUint8(requestBuf, REQUEST_CONFIG_K_FORWARD_HRB);
  robot->VMA[HRF].kBackward = floatFromUint8(requestBuf, REQUEST_CONFIG_K_FORWARD_HRF);
  robot->VMA[VB].kBackward = floatFromUint8(requestBuf, REQUEST_CONFIG_K_FORWARD_VB);
  robot->VMA[VF].kBackward = floatFromUint8(requestBuf, REQUEST_CONFIG_K_FORWARD_VF);
  robot->VMA[VL].kBackward = floatFromUint8(requestBuf, REQUEST_CONFIG_K_FORWARD_VL);
  robot->VMA[VR].kBackward = floatFromUint8(requestBuf, REQUEST_CONFIG_K_FORWARD_VR);
}



void ShoreRequest(struct Robot *robot, uint8_t *requestBuf)
{
  robot->movement.march = (int16_t)((requestBuf[SHORE_REQUEST_MARCH] << 8) + requestBuf[SHORE_REQUEST_MARCH + 1]);
  robot->movement.lag = (int16_t)((requestBuf[SHORE_REQUEST_LAG] << 8) + requestBuf[SHORE_REQUEST_LAG + 1]);
  robot->movement.depth = (int16_t)((requestBuf[SHORE_REQUEST_DEPTH] << 8) + requestBuf[SHORE_REQUEST_DEPTH + 1]);
  robot->movement.pitch = (int16_t)((requestBuf[SHORE_REQUEST_ROLL] << 8) + requestBuf[SHORE_REQUEST_ROLL + 1]);
  robot->movement.roll = (int16_t)((requestBuf[SHORE_REQUEST_PITCH] << 8) + requestBuf[SHORE_REQUEST_PITCH + 1]);
  robot->movement.yaw = (int16_t)((requestBuf[SHORE_REQUEST_YAW] << 8) + requestBuf[SHORE_REQUEST_YAW + 1]);

  robot->device.light = requestBuf[SHORE_REQUEST_LIGHT];
  robot->device.bluetoothLight = requestBuf[SHORE_REQUEST_BLUETOOTH];
  robot->device.bottomLight = requestBuf[SHORE_REQUEST_BOTTOM_LIGHT];
  robot->device.grab = requestBuf[SHORE_REQUEST_GRAB];
  robot->device.grabRotate  = requestBuf[SHORE_REQUEST_GRAB_ROTATE]; 
  robot->device.tilt = requestBuf[SHORE_REQUEST_TILT];

  robot->depthStabilization.enable = (bool) requestBuf[SHORE_REQUEST_STABILIZE_DEPTH];
  robot->rollStabilization.enable = (bool) requestBuf[SHORE_REQUEST_STABILIZE_ROLL];
  robot->pitchStabilization.enable = (bool) requestBuf[SHORE_REQUEST_STABILIZE_PITCH ];
  robot->yawStabilization.enable = (bool) requestBuf[SHORE_REQUEST_STABILIZE_YAW ];

  robot->device.resetIMU = (bool) requestBuf[SHORE_REQUEST_RESET_IMU];
}



void ShoreResponse(struct Robot *robot, uint8_t *responseBuf)
{ 
  responseBuf[SHORE_RESPONSE_ROLL] = (uint8_t)(robot->sensors.roll >> 8);
  responseBuf[SHORE_RESPONSE_ROLL + 1] = (uint8_t)(robot->sensors.roll);  // does it work?
  responseBuf[SHORE_RESPONSE_PITCH] = (uint8_t)(robot->sensors.pitch >> 8);
  responseBuf[SHORE_RESPONSE_PITCH + 1] = (uint8_t)(robot->sensors.pitch);
  responseBuf[SHORE_RESPONSE_YAW] = (uint8_t)(robot->sensors.yaw >> 8);
  responseBuf[SHORE_RESPONSE_YAW + 1] = (uint8_t)(robot->sensors.yaw);
  responseBuf[SHORE_RESPONSE_ROLL_SPEED] = (uint8_t)(robot->sensors.rollSpeed >> 8);
  responseBuf[SHORE_RESPONSE_ROLL_SPEED + 1] = (uint8_t)(robot->sensors.rollSpeed);
  responseBuf[SHORE_RESPONSE_PITCH_SPEED] = (uint8_t)(robot->sensors.pitchSpeed >> 8);
  responseBuf[SHORE_RESPONSE_PITCH_SPEED + 1] = (uint8_t)(robot->sensors.pitchSpeed);
  responseBuf[SHORE_RESPONSE_YAW_SPEED] = (uint8_t)(robot->sensors.yawSpeed >> 8);
  responseBuf[SHORE_RESPONSE_YAW_SPEED + 1] = (uint8_t)(robot->sensors.yawSpeed);
  responseBuf[SHORE_RESPONSE_PRESSURE] = (uint8_t)(robot->sensors.pressure >> 8);
  responseBuf[SHORE_RESPONSE_PRESSURE + 1] = (uint8_t)(robot->sensors.pressure);
  responseBuf[SHORE_RESPONSE_MOTOR_ERRORS] = 0x55;
  responseBuf[SHORE_RESPONSE_MOTOR_ERRORS + 1] = 0xAA;
  
  addChecksumm16b(responseBuf, SHORE_RESPONSE_LENGTH);
}


void IMUResponse(struct Robot *robot, uint8_t *IMUResponseBuf)
{
  uint8_t i = 0;
	while(i < IMU_RESPONSE_LENGTH){
		if ((IMUResponseBuf[i] == 's') && (IMUResponseBuf[(i + 1) % IMU_RESPONSE_LENGTH] == 'n') && (IMUResponseBuf[(i + 2) % IMU_RESPONSE_LENGTH] == 'p') && (IMUResponseBuf[(i + 3) % IMU_RESPONSE_LENGTH] == 0xC8)){
			switch(IMUResponseBuf[(i + 4) % IMU_RESPONSE_LENGTH]){
				case 0x62:
					robot->sensors.roll = (int16_t) ((IMUResponseBuf[(i + 5) % IMU_RESPONSE_LENGTH] << 8 ) + IMUResponseBuf[(i + 6) % IMU_RESPONSE_LENGTH]);
					robot->sensors.pitch = (int16_t) ((IMUResponseBuf[(i + 7) % IMU_RESPONSE_LENGTH] << 8 ) + IMUResponseBuf[(i + 8) % IMU_RESPONSE_LENGTH]);
					robot->sensors.yaw = (int16_t) ((IMUResponseBuf[(i + 9) % IMU_RESPONSE_LENGTH] << 8 ) + IMUResponseBuf[(i + 10) % IMU_RESPONSE_LENGTH]);
					i = i + 14;
				break;
				case 0x5C:
					robot->sensors.rollSpeed = (int16_t) ((IMUResponseBuf[(i + 5) % IMU_RESPONSE_LENGTH] << 8 ) + IMUResponseBuf[(i + 6) % IMU_RESPONSE_LENGTH]);
					robot->sensors.pitchSpeed = (int16_t) ((IMUResponseBuf[(i + 7) % IMU_RESPONSE_LENGTH] << 8 ) + IMUResponseBuf[(i + 8) % IMU_RESPONSE_LENGTH]);
					robot->sensors.yawSpeed = (int16_t) ((IMUResponseBuf[(i + 9) % IMU_RESPONSE_LENGTH] << 8 ) + IMUResponseBuf[(i + 10) % IMU_RESPONSE_LENGTH]);
					i = i + 14;
				break;
			}
		}
		++i;
	}
}
