#ifndef ROBOT_H
#define ROBOT_H

#include <stdbool.h>
#include <math.h>
#include <stdint.h>

#define VMA_NUMBER              8
#define VMA_DRIVER_NUMBER       8
#define DEV_DRIVER_NUMBER       6    
#define BLUETOOTH_MESSAGE_SIZE  8
 

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



struct Robot{
	
	struct robotVMA{
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
	}VMA[VMA_NUMBER];
  
  struct robotSensors{
    int16_t roll;
    int16_t pitch;
    int16_t yaw;
    int16_t rollSpeed;
    int16_t pitchSpeed;
    int16_t yawSpeed;
		bool resetIMU;
    
    uint16_t pressure;
  } sensors;
	
	struct robotBluetooth{
		char message[BLUETOOTH_MESSAGE_SIZE];
	} bluetooth;

	struct RobotMovement{
		int16_t march;
    int16_t lag;
    int16_t depth;
    int16_t roll;
    int16_t pitch;
    int16_t yaw;
	} movement;
	
	struct RobotDevice{
		struct RobotLight{
			uint8_t address;
			uint8_t settings;
			uint8_t brightness;
			uint16_t current;
		} light, bottomLight;
		
		struct RobotGrab{
			uint8_t squeezeAddress;
			uint8_t rotationAddress;
			uint8_t settings;
			uint8_t squeeze;
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
	}device;
  
  struct RobotStabilization{
    bool enable;
    bool constTime;
    float K1;
    float K2;
    float startValue;
    float gain;
    float speedError;
  } depthStabilization, rollStabilization, pitchStabilization, yawStabilization;
	
}; 

#endif
