#include "messages.h"

void DevRequestUpdate(uint8_t *buf, uint8_t DEV)
{
	buf[VMA_DEV_REQUEST_AA1] = 0xAA;
	buf[VMA_DEV_REQUEST_AA2] = 0xAA;
	
	switch(DEV){
		case SUCKER:
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



void VMARequestUpdate(uint8_t *buf, uint8_t DEV)
{
	buf[VMA_DEV_REQUEST_AA1] = 0xAA;
	buf[VMA_DEV_REQUEST_AA2] = 0xAA;
	
	switch(DEV){
		case HLF:
			buf[VMA_DEV_REQUEST_ADDRESS] = 0x01;
			buf[VMA_DEV_REQUEST_SETTING] = 0x55;
			buf[VMA_DEV_REQUEST_VELOCITY1] = 0x55;
			buf[VMA_DEV_REQUEST_VELOCITY2] = 0xAA;
			break;
		case HLB:
			buf[VMA_DEV_REQUEST_ADDRESS] = 0x02;
			buf[VMA_DEV_REQUEST_SETTING] = 0x55;
			buf[VMA_DEV_REQUEST_VELOCITY1] = 0x55;
			buf[VMA_DEV_REQUEST_VELOCITY2] = 0xAA;
			break;
		case HRB:
			buf[VMA_DEV_REQUEST_ADDRESS] = 0x03;
			buf[VMA_DEV_REQUEST_SETTING] = 0xAA;
			buf[VMA_DEV_REQUEST_VELOCITY1] = 0x55;
			buf[VMA_DEV_REQUEST_VELOCITY2] = 0xAA;
			break;
		case HRF:
			buf[VMA_DEV_REQUEST_ADDRESS] = 0x04;
			buf[VMA_DEV_REQUEST_SETTING] = 0xAA;
			buf[VMA_DEV_REQUEST_VELOCITY1] = 0x55;
			buf[VMA_DEV_REQUEST_VELOCITY2] = 0xAA;
			break;
		case VL:
			buf[VMA_DEV_REQUEST_ADDRESS] = 0x05;
			buf[VMA_DEV_REQUEST_SETTING] = 0xAA;
			buf[VMA_DEV_REQUEST_VELOCITY1] = 0x55;
			buf[VMA_DEV_REQUEST_VELOCITY2] = 0xAA;
			break;
		case VB:
			buf[VMA_DEV_REQUEST_ADDRESS] = 0x06;
			buf[VMA_DEV_REQUEST_SETTING] = 0xAA;
			buf[VMA_DEV_REQUEST_VELOCITY1] = 0x55;
			buf[VMA_DEV_REQUEST_VELOCITY2] = 0xAA;
			break;
		case VR:
			buf[VMA_DEV_REQUEST_ADDRESS] = 0x07;
			buf[VMA_DEV_REQUEST_SETTING] = 0xAA;
			buf[VMA_DEV_REQUEST_VELOCITY1] = 0x55;
			buf[VMA_DEV_REQUEST_VELOCITY2] = 0xAA;
			break;
		case VF:
			buf[VMA_DEV_REQUEST_ADDRESS] = 0x08;
			buf[VMA_DEV_REQUEST_SETTING] = 0xAA;
			buf[VMA_DEV_REQUEST_VELOCITY1] = 0x55;
			buf[VMA_DEV_REQUEST_VELOCITY2] = 0xAA;
			break;
	}
	addCheckSumm8b(buf, VMA_DEV_REQUEST_LENGTH);
}

float FloatFromUint8(uint8_t *buff, uint8_t high_byte_pos)
{
  float result;
  result = (float)((buff[high_byte_pos] << 24) + (buff[high_byte_pos + 1] << 16) + (buff[high_byte_pos + 2] << 8) + buff[high_byte_pos + 3]);
  return result;
}



void ShoreConfigRequest(struct Robot *robot, uint8_t *requestBuf)
{
  robot->depth_stabilization.const_time = requestBuf[REQUEST_CONFIG_CONST_TIME_DEPTH];
  robot->depth_stabilization.K1 = FloatFromUint8(requestBuf, REQUEST_CONFIG_K1_DEPTH);
  robot->depth_stabilization.K2 = FloatFromUint8(requestBuf, REQUEST_CONFIG_K2_DEPTH);
  robot->depth_stabilization.start_value = FloatFromUint8(requestBuf, REQUEST_CONFIG_START_DEPTH);
  robot->depth_stabilization.gain = FloatFromUint8(requestBuf, REQUEST_CONFIG_GAIN_DEPTH);
  
  robot->roll_stabilization.const_time = requestBuf[REQUEST_CONFIG_CONST_TIME_ROLL];
  robot->roll_stabilization.K1 = FloatFromUint8(requestBuf, REQUEST_CONFIG_K1_ROLL);
  robot->roll_stabilization.K2 = FloatFromUint8(requestBuf, REQUEST_CONFIG_K2_ROLL);
  robot->roll_stabilization.start_value = FloatFromUint8(requestBuf, REQUEST_CONFIG_START_ROLL);
  robot->roll_stabilization.gain = FloatFromUint8(requestBuf, REQUEST_CONFIG_GAIN_ROLL);
  
  robot->pitch_stabilization.const_time = requestBuf[REQUEST_CONFIG_CONST_TIME_PITCH];
  robot->pitch_stabilization.K1 = FloatFromUint8(requestBuf, REQUEST_CONFIG_K1_PITCH);
  robot->pitch_stabilization.K2 = FloatFromUint8(requestBuf, REQUEST_CONFIG_K2_PITCH);
  robot->pitch_stabilization.start_value = FloatFromUint8(requestBuf, REQUEST_CONFIG_START_PITCH);
  robot->pitch_stabilization.gain = FloatFromUint8(requestBuf, REQUEST_CONFIG_GAIN_PITCH);
  
  robot->yaw_stabilization.const_time = requestBuf[REQUEST_CONFIG_CONST_TIME_YAW];
  robot->yaw_stabilization.K1 = FloatFromUint8(requestBuf, REQUEST_CONFIG_K1_YAW);
  robot->yaw_stabilization.K2 = FloatFromUint8(requestBuf, REQUEST_CONFIG_K2_YAW);
  robot->yaw_stabilization.start_value = FloatFromUint8(requestBuf, REQUEST_CONFIG_START_YAW);
  robot->yaw_stabilization.gain = FloatFromUint8(requestBuf, REQUEST_CONFIG_GAIN_YAW);
  
  robot->HLB.address = requestBuf[REQUEST_CONFIG_POSITION_HLB];
  robot->HLF.address = requestBuf[REQUEST_CONFIG_POSITION_HLF];
  robot->HRB.address = requestBuf[REQUEST_CONFIG_POSITION_HRB];
  robot->HRF.address = requestBuf[REQUEST_CONFIG_POSITION_HRF];
  robot->VB.address = requestBuf[REQUEST_CONFIG_POSITION_VB];
  robot->VF.address = requestBuf[REQUEST_CONFIG_POSITION_VF];
  robot->VL.address  = requestBuf[REQUEST_CONFIG_POSITION_VL];
  robot->VR.address = requestBuf[REQUEST_CONFIG_POSITION_VR];
  
  robot->HLB.inverse = requestBuf[REQUEST_CONFIG_INVERSE_HLB];
  robot->HLF.inverse = requestBuf[REQUEST_CONFIG_INVERSE_HLF];
  robot->HRB.inverse = requestBuf[REQUEST_CONFIG_INVERSE_HRB];
  robot->HRF.inverse = requestBuf[REQUEST_CONFIG_INVERSE_HRF];
  robot->VB.inverse = requestBuf[REQUEST_CONFIG_INVERSE_VB];
  robot->VF.inverse = requestBuf[REQUEST_CONFIG_INVERSE_VF];
  robot->VL.inverse = requestBuf[REQUEST_CONFIG_INVERSE_VL];
  robot->VR.inverse = requestBuf[REQUEST_CONFIG_INVERSE_VR];
  
  robot->HLB.k_forward = FloatFromUint8(requestBuf, REQUEST_CONFIG_K_FORWARD_HLB);
  robot->HLF.k_forward = FloatFromUint8(requestBuf, REQUEST_CONFIG_K_FORWARD_HLF);
  robot->HRB.k_forward = FloatFromUint8(requestBuf, REQUEST_CONFIG_K_FORWARD_HRB);
  robot->HRF.k_forward = FloatFromUint8(requestBuf, REQUEST_CONFIG_K_FORWARD_HRF);
  robot->VB.k_forward = FloatFromUint8(requestBuf, REQUEST_CONFIG_K_FORWARD_VB);
  robot->VF.k_forward = FloatFromUint8(requestBuf, REQUEST_CONFIG_K_FORWARD_VF);
  robot->VL.k_forward = FloatFromUint8(requestBuf, REQUEST_CONFIG_K_FORWARD_VL);
  robot->VR.k_forward = FloatFromUint8(requestBuf, REQUEST_CONFIG_K_FORWARD_VR);
  
  robot->HLB.k_backward = FloatFromUint8(requestBuf, REQUEST_CONFIG_K_BACKWARD_HLB);
  robot->HLF.k_backward = FloatFromUint8(requestBuf, REQUEST_CONFIG_K_BACKWARD_HLF);
  robot->HRB.k_backward = FloatFromUint8(requestBuf, REQUEST_CONFIG_K_BACKWARD_HRB);
  robot->HRF.k_backward = FloatFromUint8(requestBuf, REQUEST_CONFIG_K_BACKWARD_HRF);
  robot->VB.k_backward = FloatFromUint8(requestBuf, REQUEST_CONFIG_K_BACKWARD_VB);
  robot->VF.k_backward = FloatFromUint8(requestBuf, REQUEST_CONFIG_K_BACKWARD_VF);
  robot->VL.k_backward = FloatFromUint8(requestBuf, REQUEST_CONFIG_K_BACKWARD_VL);
  robot->VR.k_backward = FloatFromUint8(requestBuf, REQUEST_CONFIG_K_BACKWARD_VR);
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
  robot->device.bluetooth_light = requestBuf[SHORE_REQUEST_BLUETOOTH];
  robot->device.bottom_light = requestBuf[SHORE_REQUEST_BOTTOM_LIGHT];
  robot->device.grab = requestBuf[SHORE_REQUEST_GRAB];
  robot->device.grab_rotate  = requestBuf[SHORE_REQUEST_GRAB_ROTATE]; 
  robot->device.tilt = requestBuf[SHORE_REQUEST_TILT];

  robot->depth_stabilization.enable = (bool) requestBuf[SHORE_REQUEST_STABILIZE_DEPTH];
  robot->roll_stabilization.enable = (bool) requestBuf[SHORE_REQUEST_STABILIZE_ROLL];
  robot->pitch_stabilization.enable = (bool) requestBuf[SHORE_REQUEST_STABILIZE_PITCH ];
  robot->yaw_stabilization.enable = (bool) requestBuf[SHORE_REQUEST_STABILIZE_YAW ];

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
  responseBuf[SHORE_RESPONSE_ROLL_SPEED] = (uint8_t)(robot->sensors.roll_speed >> 8);
  responseBuf[SHORE_RESPONSE_ROLL_SPEED + 1] = (uint8_t)(robot->sensors.roll_speed);
  responseBuf[SHORE_RESPONSE_PITCH_SPEED] = (uint8_t)(robot->sensors.pitch_speed >> 8);
  responseBuf[SHORE_RESPONSE_PITCH_SPEED + 1] = (uint8_t)(robot->sensors.pitch_speed);
  responseBuf[SHORE_RESPONSE_YAW_SPEED] = (uint8_t)(robot->sensors.yaw_speed >> 8);
  responseBuf[SHORE_RESPONSE_YAW_SPEED + 1] = (uint8_t)(robot->sensors.yaw_speed);
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
					robot->sensors.roll_speed = (int16_t) ((IMUResponseBuf[(i + 5) % IMU_RESPONSE_LENGTH] << 8 ) + IMUResponseBuf[(i + 6) % IMU_RESPONSE_LENGTH]);
					robot->sensors.pitch_speed = (int16_t) ((IMUResponseBuf[(i + 7) % IMU_RESPONSE_LENGTH] << 8 ) + IMUResponseBuf[(i + 8) % IMU_RESPONSE_LENGTH]);
					robot->sensors.yaw_speed = (int16_t) ((IMUResponseBuf[(i + 9) % IMU_RESPONSE_LENGTH] << 8 ) + IMUResponseBuf[(i + 10) % IMU_RESPONSE_LENGTH]);
					i = i + 14;
				break;
			}
		}
		++i;
	}
}
