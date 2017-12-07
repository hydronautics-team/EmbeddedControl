#ifndef ROBOT_H
#define ROBOT_H

#include <stdbool.h>
#include <math.h>
#include <stdint.h>

#include "u_messages.h"

enum VMA{
	HLB = 0,
	HLF, 
	HRB, 
	HRF, 
	VB, 
	VF, 
	VL, 
	VR
};

enum DEV{
	AGAR = 0,
	GRAB,
	GRAB_ROTATION,
	TILT,
	LIGHT,
	BOTTOM_LIGHT
};

struct Robot {
	
	struct robotVMA {
		uint8_t address;
		int8_t desiredSpeed;
		int8_t realSpeed;
		int8_t speedError;
		
		uint8_t errors;
		
		// There is flags of: enabling, inversing, ...
		uint8_t settings;
		uint16_t current;
			
		double kForward;
		double kBackward;
	} VMA[VMA_NUMBER];
  
  struct robotSensors{
    int16_t roll;
    int16_t pitch;
    int16_t yaw;
    int16_t rollSpeed;
    int16_t pitchSpeed;
    int16_t yawSpeed;
		int16_t accelX;
		int16_t accelY;
		int16_t accelZ;
		int16_t magX;
		int16_t magY;
		int16_t magZ;
		int16_t quatA;
		int16_t quatB;
		int16_t quatC;
		int16_t quatD;
		bool resetIMU;
    
    uint16_t pressure;
  } sensors;
	
	struct robotBluetooth{
		uint8_t message[BT_SIZE];
	} bluetooth;

	struct RobotMovement{
		int16_t march;
    int16_t lag;
    int16_t depth;
    int16_t roll;
    int16_t pitch;
    int16_t yaw;
	} movement;
	
	struct RobotDevice {
		
		struct RobotLight{
			uint8_t address;
			uint8_t settings;
			int8_t brightness;
			uint16_t current;
		} light, bottomLight;
		
		struct RobotGrab{
			uint8_t squeezeAddress;
			uint8_t rotationAddress;
			uint8_t settings;
			int8_t squeeze;
			int8_t rotation;
			uint16_t squeezeCurrent;
			uint16_t rotationCurrent;
		} grab;
		
		struct RobotTilt{
			uint8_t address;
			uint8_t settings;
			int8_t rotation;
			uint16_t current;
		} tilt;
		
		struct RobotAgarGrab{
			uint8_t address;
			uint8_t settings;
			uint8_t opening;
			uint16_t current;
		} agar;
		uint8_t errors;
	} device;
  
	struct RobotStabilization {
		bool enable;
		bool iPartEnable;
		float pGain;
		float iGain;
		float iMax;
		float iMin;
		float positionFeedbackCoef;
		float speedFeedbackCoef;
		int16_t speedError;
  } depthStabilization, rollStabilization, pitchStabilization, yawStabilization;
	
}; 

#endif
