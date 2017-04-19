#ifndef ROBOT_H
#define ROBOT_H

#include <stdbool.h>
#include <math.h>
#include <stdint.h>

#define VMA_NUMBER            8
#define VMA_DRIVER_NUMBER     8
#define DEV_DRIVER_NUMBER     4    
 

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
	GRUB,
	GRUBROTATION,
	TILT
};



struct Robot{
	
	struct robotVMA{
		uint8_t address;
		int8_t speed;
		
		// There is flags of: enabling, inversing, ...
		uint8_t settings;  
			
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
    
    uint16_t pressure;
  } sensors;
  
  struct robotErrors{
    uint16_t motorErrors;
  } errors;
  
	struct shorePositionControl{
		int16_t march;
    int16_t lag;
    int16_t depth;
    int16_t roll;
    int16_t pitch;
    int16_t yaw;
	} movement;
  
  struct deviceControl{
    
		bool resetIMU;
    
		uint8_t light;
		uint8_t bottomLight;
		uint8_t bluetoothLight;
    uint8_t grab;
    int8_t grabRotate;
    int8_t tilt;
  } device;
  
  struct stabilization{
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
