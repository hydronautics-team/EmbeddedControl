#include "communication.h"


void DevRequestUpdate(struct Robot *robot, uint8_t *buf, uint8_t DEV)
{
	buf[VMA_DEV_REQUEST_AA1] = 0xAA;
	buf[VMA_DEV_REQUEST_AA2] = 0xAA;
	
	switch(DEV){
		case AGAR:
			buf[VMA_DEV_REQUEST_ADDRESS] = robot->device.agar.address;
			buf[VMA_DEV_REQUEST_SETTING] = robot->device.agar.settings;
			buf[VMA_DEV_REQUEST_VELOCITY1] = robot->device.agar.opening;
			buf[VMA_DEV_REQUEST_VELOCITY2] = NULL;
			break;
		case GRAB:
			buf[VMA_DEV_REQUEST_ADDRESS] = robot->device.grab.squeezeAddress;
			buf[VMA_DEV_REQUEST_SETTING] = robot->device.grab.settings;
			buf[VMA_DEV_REQUEST_VELOCITY1] = robot->device.grab.squeeze;
			buf[VMA_DEV_REQUEST_VELOCITY2] = NULL;
			break;
		case GRABROTATION:
			buf[VMA_DEV_REQUEST_ADDRESS] = robot->device.grab.rotationAddress;
			buf[VMA_DEV_REQUEST_SETTING] = robot->device.grab.settings;
			buf[VMA_DEV_REQUEST_VELOCITY1] = robot->device.grab.rotation;
			buf[VMA_DEV_REQUEST_VELOCITY2] = NULL;
			break;
		case TILT:
			buf[VMA_DEV_REQUEST_ADDRESS] = robot->device.tilt.address;
			buf[VMA_DEV_REQUEST_SETTING] = robot->device.tilt.settings;
			buf[VMA_DEV_REQUEST_VELOCITY1] = robot->device.tilt.rotation;
			buf[VMA_DEV_REQUEST_VELOCITY2] = NULL;
			break;
	}
	AddChecksumm8b(buf, VMA_DEV_REQUEST_LENGTH);
}



void DevResponseUpdate(struct Robot *robot, uint8_t *buf, uint8_t DEV)
{
	
}



void VMARequestUpdate(struct Robot *robot, uint8_t *buf, uint8_t DEV)
{
	buf[VMA_DEV_REQUEST_AA1] = 0xAA;
	buf[VMA_DEV_REQUEST_AA2] = 0xAA;
	
	switch(DEV){
		case HLB:
			buf[VMA_DEV_REQUEST_ADDRESS] = robot->VMA[HLB].address;
			buf[VMA_DEV_REQUEST_SETTING] = robot->VMA[HLB].settings;
			buf[VMA_DEV_REQUEST_VELOCITY1] = robot->VMA[HLB].speed;
			buf[VMA_DEV_REQUEST_VELOCITY2] = robot->device.light.brightness;
			break;
		
		case HLF:
			buf[VMA_DEV_REQUEST_ADDRESS] = robot->VMA[HLF].address;
			buf[VMA_DEV_REQUEST_SETTING] = robot->VMA[HLF].settings;
			buf[VMA_DEV_REQUEST_VELOCITY1] = robot->VMA[HLF].speed;
			buf[VMA_DEV_REQUEST_VELOCITY2] = robot->device.bottomLight.brightness;
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
		
		case VB:
			buf[VMA_DEV_REQUEST_ADDRESS] = robot->VMA[VB].address;
			buf[VMA_DEV_REQUEST_SETTING] = robot->VMA[VB].settings;
			buf[VMA_DEV_REQUEST_VELOCITY1] = robot->VMA[VB].speed;
			buf[VMA_DEV_REQUEST_VELOCITY2] = NULL;
			break;
		
		case VF:
			buf[VMA_DEV_REQUEST_ADDRESS] = robot->VMA[VF].address;
			buf[VMA_DEV_REQUEST_SETTING] = robot->VMA[VF].settings;
			buf[VMA_DEV_REQUEST_VELOCITY1] = robot->VMA[VF].speed;
			buf[VMA_DEV_REQUEST_VELOCITY2] = NULL;
			break;
		
		case VL:
			buf[VMA_DEV_REQUEST_ADDRESS] = robot->VMA[VL].address;
			buf[VMA_DEV_REQUEST_SETTING] = robot->VMA[VL].settings;
			buf[VMA_DEV_REQUEST_VELOCITY1] = robot->VMA[VL].speed;
			buf[VMA_DEV_REQUEST_VELOCITY2] = NULL;
			break;
		
		case VR:
			buf[VMA_DEV_REQUEST_ADDRESS] = robot->VMA[VR].address;
			buf[VMA_DEV_REQUEST_SETTING] = robot->VMA[VR].settings;
			buf[VMA_DEV_REQUEST_VELOCITY1] = robot->VMA[VR].speed;
			buf[VMA_DEV_REQUEST_VELOCITY2] = NULL;
			break;
	}
	AddChecksumm8b(buf, VMA_DEV_REQUEST_LENGTH);
}



void VMAResponseUpdate(struct Robot *robot, uint8_t *buf, uint8_t VMA)
{
	
}


float FloatFromUint8(uint8_t *buff, uint8_t high_byte_pos)
{
  float result;
  result = (float)((buff[high_byte_pos] << 24) + (buff[high_byte_pos + 1] << 16) + (buff[high_byte_pos + 2] << 8) + buff[high_byte_pos + 3]);
  return result;
}


void ShoreConfigRequest(struct Robot *robot, uint8_t *requestBuf)
{
  robot->depthStabilization.constTime = requestBuf[REQUEST_CONFIG_CONST_TIME_DEPTH];
  robot->depthStabilization.K1 = FloatFromUint8(requestBuf, REQUEST_CONFIG_K1_DEPTH);
  robot->depthStabilization.K2 = FloatFromUint8(requestBuf, REQUEST_CONFIG_K2_DEPTH);
  robot->depthStabilization.startValue = FloatFromUint8(requestBuf, REQUEST_CONFIG_START_DEPTH);
  robot->depthStabilization.gain = FloatFromUint8(requestBuf, REQUEST_CONFIG_GAIN_DEPTH);
  
  robot->rollStabilization.constTime = requestBuf[REQUEST_CONFIG_CONST_TIME_ROLL];
  robot->rollStabilization.K1 = FloatFromUint8(requestBuf, REQUEST_CONFIG_K1_ROLL);
  robot->rollStabilization.K2 = FloatFromUint8(requestBuf, REQUEST_CONFIG_K2_ROLL);
  robot->rollStabilization.startValue = FloatFromUint8(requestBuf, REQUEST_CONFIG_START_ROLL);
  robot->rollStabilization.gain = FloatFromUint8(requestBuf, REQUEST_CONFIG_GAIN_ROLL);
  
  robot->pitchStabilization.constTime = requestBuf[REQUEST_CONFIG_CONST_TIME_PITCH];
  robot->pitchStabilization.K1 = FloatFromUint8(requestBuf, REQUEST_CONFIG_K1_PITCH);
  robot->pitchStabilization.K2 = FloatFromUint8(requestBuf, REQUEST_CONFIG_K2_PITCH);
  robot->pitchStabilization.startValue = FloatFromUint8(requestBuf, REQUEST_CONFIG_START_PITCH);
  robot->pitchStabilization.gain = FloatFromUint8(requestBuf, REQUEST_CONFIG_GAIN_PITCH);
  
  robot->yawStabilization.constTime = requestBuf[REQUEST_CONFIG_CONST_TIME_YAW];
  robot->yawStabilization.K1 = FloatFromUint8(requestBuf, REQUEST_CONFIG_K1_YAW);
  robot->yawStabilization.K2 = FloatFromUint8(requestBuf, REQUEST_CONFIG_K2_YAW);
  robot->yawStabilization.startValue = FloatFromUint8(requestBuf, REQUEST_CONFIG_START_YAW);
  robot->yawStabilization.gain = FloatFromUint8(requestBuf, REQUEST_CONFIG_GAIN_YAW);
  
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



void ShoreRequest(struct Robot *robot, uint8_t *requestBuf)
{
  robot->movement.march = (int16_t)((requestBuf[SHORE_REQUEST_MARCH] << 8) + requestBuf[SHORE_REQUEST_MARCH + 1]);
  robot->movement.lag = (int16_t)((requestBuf[SHORE_REQUEST_LAG] << 8) + requestBuf[SHORE_REQUEST_LAG + 1]);
  robot->movement.depth = (int16_t)((requestBuf[SHORE_REQUEST_DEPTH] << 8) + requestBuf[SHORE_REQUEST_DEPTH + 1]);
  robot->movement.pitch = (int16_t)((requestBuf[SHORE_REQUEST_ROLL] << 8) + requestBuf[SHORE_REQUEST_ROLL + 1]);
  robot->movement.roll = (int16_t)((requestBuf[SHORE_REQUEST_PITCH] << 8) + requestBuf[SHORE_REQUEST_PITCH + 1]);
  robot->movement.yaw = (int16_t)((requestBuf[SHORE_REQUEST_YAW] << 8) + requestBuf[SHORE_REQUEST_YAW + 1]);

  robot->device.light.brightness = requestBuf[SHORE_REQUEST_LIGHT];
  robot->device.agar.opening = requestBuf[SHORE_REQUEST_AGAR];
  robot->device.bottomLight.brightness = requestBuf[SHORE_REQUEST_BOTTOM_LIGHT];
  robot->device.grab.squeeze = requestBuf[SHORE_REQUEST_GRAB];
  robot->device.grab.rotation  = requestBuf[SHORE_REQUEST_GRAB_ROTATE]; 
  robot->device.tilt.rotation = requestBuf[SHORE_REQUEST_TILT];

  robot->depthStabilization.enable = (bool) requestBuf[SHORE_REQUEST_STABILIZE_DEPTH];
  robot->rollStabilization.enable = (bool) requestBuf[SHORE_REQUEST_STABILIZE_ROLL];
  robot->pitchStabilization.enable = (bool) requestBuf[SHORE_REQUEST_STABILIZE_PITCH ];
  robot->yawStabilization.enable = (bool) requestBuf[SHORE_REQUEST_STABILIZE_YAW ];

  robot->sensors.resetIMU = (bool) requestBuf[SHORE_REQUEST_RESET_IMU];
}



void ShoreResponse(struct Robot *robot, uint8_t *responseBuf)
{
  responseBuf[SHORE_RESPONSE_ROLL] = robot->sensors.roll >> 8;
  responseBuf[SHORE_RESPONSE_ROLL + 1] = robot->sensors.roll;  // does it work?
  responseBuf[SHORE_RESPONSE_PITCH] = robot->sensors.pitch >> 8;
  responseBuf[SHORE_RESPONSE_PITCH + 1] = robot->sensors.pitch;
  responseBuf[SHORE_RESPONSE_YAW] = robot->sensors.yaw >> 8;
  responseBuf[SHORE_RESPONSE_YAW + 1] = robot->sensors.yaw;
  responseBuf[SHORE_RESPONSE_ROLL_SPEED] = robot->sensors.rollSpeed >> 8;
  responseBuf[SHORE_RESPONSE_ROLL_SPEED + 1] = robot->sensors.rollSpeed;
  responseBuf[SHORE_RESPONSE_PITCH_SPEED] = robot->sensors.pitchSpeed >> 8;
  responseBuf[SHORE_RESPONSE_PITCH_SPEED + 1] = robot->sensors.pitchSpeed;
  responseBuf[SHORE_RESPONSE_YAW_SPEED] = robot->sensors.yawSpeed >> 8;
  responseBuf[SHORE_RESPONSE_YAW_SPEED + 1] = robot->sensors.yawSpeed;
  
	responseBuf[SHORE_RESPONSE_PRESSURE] = robot->sensors.pressure >> 8;
  responseBuf[SHORE_RESPONSE_PRESSURE + 1] = robot->sensors.pressure;
	
	for(uint8_t i; i < BLUETOOTH_MESSAGE_SIZE; ++i){
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
	
	responseBuf[SHORE_RESPONSE_VMA_VELOCITY_HLB] = robot->VMA[HLB].speed;
	responseBuf[SHORE_RESPONSE_VMA_VELOCITY_HLF] = robot->VMA[HLF].speed;
	responseBuf[SHORE_RESPONSE_VMA_VELOCITY_HRB] = robot->VMA[HRB].speed;
	responseBuf[SHORE_RESPONSE_VMA_VELOCITY_HRF] = robot->VMA[HRF].speed;
	responseBuf[SHORE_RESPONSE_VMA_VELOCITY_VB] = robot->VMA[VB].speed;
	responseBuf[SHORE_RESPONSE_VMA_VELOCITY_VF] = robot->VMA[VF].speed;
	responseBuf[SHORE_RESPONSE_VMA_VELOCITY_VL] = robot->VMA[VR].speed;
	responseBuf[SHORE_RESPONSE_VMA_VELOCITY_VR] = robot->VMA[VR].speed;
	
	
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



void IMUReset()
{
  IMURequestBuf[0] = 's';
  IMURequestBuf[1] = 'n';
  IMURequestBuf[2] = 'p';
  IMURequestBuf[3] = 0x00;
  IMURequestBuf[4] = 0xAC;  // Zero_gyros
  uint16_t checksum = IMUchecksum(IMURequestBuf, 5);
  IMURequestBuf[5] = checksum >> 8;
  IMURequestBuf[6] = (uint8_t) checksum;
  IMURequestBuf[7] = 's';
  IMURequestBuf[8] = 'n';
  IMURequestBuf[9] = 'p';
  IMURequestBuf[10] = 0x00;
  IMURequestBuf[11] = 0xAD;  // Reset EKF
  checksum = IMUchecksum(IMURequestBuf + 7, 5);
  IMURequestBuf[12] = checksum >> 8;
  IMURequestBuf[13] = (uint8_t) checksum;
  IMURequestBuf[14] = 's';
  IMURequestBuf[15] = 'n';
  IMURequestBuf[16] = 'p';
  IMURequestBuf[17] = 0x00;
  IMURequestBuf[18] = 0xAF;  // Set Accelerometer Reference vector
  checksum = IMUchecksum(IMURequestBuf + 14, 5);
  IMURequestBuf[19] = checksum >> 8;
  IMURequestBuf[20] = (uint8_t) checksum;
}
