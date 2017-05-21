#include "communication.h"
#define PACKAGE_TOLLERANCE 20

void DevRequestUpdate(struct Robot *robot, uint8_t *buf, uint8_t DEV)
{
	buf[VMA_DEV_REQUEST_AA1] = 0xAA;
	buf[VMA_DEV_REQUEST_AA2] = 0xAA;
	
	switch(DEV){
		case AGAR:
			buf[VMA_DEV_REQUEST_ADDRESS] = robot->device.agar.address;
			buf[VMA_DEV_REQUEST_SETTING] = robot->device.agar.settings;
			buf[VMA_DEV_REQUEST_VELOCITY1] = NULL;//robot->device.agar.opening;
			buf[VMA_DEV_REQUEST_VELOCITY2] = robot->device.agar.opening;//NULL;
			break;
		case GRAB:
			buf[VMA_DEV_REQUEST_ADDRESS] = robot->device.grab.squeezeAddress;
			buf[VMA_DEV_REQUEST_SETTING] = robot->device.grab.settings;
			buf[VMA_DEV_REQUEST_VELOCITY1] = NULL;//robot->device.grab.squeeze;
			buf[VMA_DEV_REQUEST_VELOCITY2] = robot->device.grab.squeeze;//NULL;
			break;
		case GRAB_ROTATION:
			buf[VMA_DEV_REQUEST_ADDRESS] = robot->device.grab.rotationAddress;
			buf[VMA_DEV_REQUEST_SETTING] = robot->device.grab.settings;
			buf[VMA_DEV_REQUEST_VELOCITY1] = NULL;//robot->device.grab.rotation;
			buf[VMA_DEV_REQUEST_VELOCITY2] = robot->device.grab.rotation;//NULL;
			break;
		case TILT:
			buf[VMA_DEV_REQUEST_ADDRESS] = robot->device.tilt.address;
			buf[VMA_DEV_REQUEST_SETTING] = robot->device.tilt.settings;
			buf[VMA_DEV_REQUEST_VELOCITY1] = NULL;//robot->device.tilt.rotation;
			buf[VMA_DEV_REQUEST_VELOCITY2] = robot->device.tilt.rotation;//NULL;
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
	if(IsChecksumm8bCorrect(buf, VMA_DEV_RESPONSE_LENGTH)){
		switch(dev){
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
	
	switch(vma){
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
			buf[VMA_DEV_REQUEST_VELOCITY2] = robot->device.bottomLight.brightness;
			break;
		
		case HRB:
			buf[VMA_DEV_REQUEST_ADDRESS] = robot->VMA[HRB].address;
			buf[VMA_DEV_REQUEST_SETTING] = robot->VMA[HRB].settings;
			buf[VMA_DEV_REQUEST_VELOCITY1] = robot->VMA[HRB].desiredSpeed;
			buf[VMA_DEV_REQUEST_VELOCITY2] = NULL;
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
	if(IsChecksumm8bCorrect(buf, VMA_DEV_RESPONSE_LENGTH)){
		switch(vma){
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
  result = (float)((buff[high_byte_pos] << 24) + (buff[high_byte_pos + 1] << 16) + (buff[high_byte_pos + 2] << 8) + buff[high_byte_pos + 3]);
  return result;
}


void ShoreConfigRequest(struct Robot *robot, uint8_t *requestBuf)
{
	if (IsChecksumm16bCorrect(requestBuf + 1, SHORE_REQUEST_LENGTH - 1)){
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
}



void ShoreRequest(struct Robot *robot, uint8_t *requestBuf)
{
	if (IsChecksumm16bCorrect(requestBuf + 1, SHORE_REQUEST_LENGTH - 1)){
		shorePackageError = 0;
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
		
		int16_t velocity[VMA_DRIVER_NUMBER];
		velocity[HLB] = (int16_t)-(( - robot->movement.march + robot->movement.lag  - robot->movement.yaw) >> 1); 
		velocity[HLF] = (int16_t)((+ robot->movement.march + robot->movement.lag  + robot->movement.yaw) >> 1);
		velocity[HRB] = (int16_t)-(( - robot->movement.march - robot->movement.lag  + robot->movement.yaw) >> 1);
		velocity[HRF] = (int16_t)((+ robot->movement.march - robot->movement.lag  - robot->movement.yaw) >> 1);
		velocity[VB]  = (int16_t)((- robot->movement.depth + robot->movement.pitch) >> 1); 
		velocity[VF]  = (int16_t)((+ robot->movement.depth + robot->movement.pitch) >> 1); 
		velocity[VL]  = (int16_t)((- robot->movement.depth + robot->movement.roll) >> 1);
		velocity[VR]  = (int16_t)((- robot->movement.depth - robot->movement.roll) >> 1);
		
		for (uint8_t i = 0; i < VMA_DRIVER_NUMBER; ++i){
			velocity[i] = (int8_t)(velocity[i] / 0xFF);
			if (velocity[i] > 127){
				robot->VMA[i].desiredSpeed = 127;
			}
			else if( velocity[i] > -127){
				robot->VMA[i].desiredSpeed = velocity[i];
			}
			else{
				robot->VMA[i].desiredSpeed = -127;				
			}
		}
	}
	else{
		++shorePackageError;
	}
	
	if (shorePackageError == PACKAGE_TOLLERANCE){
		robot->movement.march = 0;
		robot->movement.lag = 0;
		robot->movement.depth = 0;
		robot->movement.pitch = 0;
		robot->movement.roll = 0;
		robot->movement.yaw = 0;

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
  responseBuf[SHORE_RESPONSE_ROLL] = robot->sensors.roll >> 8;
  responseBuf[SHORE_RESPONSE_ROLL + 1] = robot->sensors.roll;  // does it work? xz
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
