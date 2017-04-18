#ifndef ROBOT_H
#define ROBOT_H

#include <stdbool.h>
#include <math.h>
#include <stdint.h>

enum VMA{
	HLF = 1,
	HLB, 
	HRB, 
	HRF, 
	VL, 
	VB, 
	VR, 
	VF
};


enum DEV{
	SUCKER = 1,
	GRUB,
	GRUBROTATION,
	TILT
};
	


struct Robot{
	
	struct robotVMA{
		uint8_t address;
		
		int8_t speed;
		bool inverse;
		bool enabled;
			
		double k_forward;
		double k_backward;
	}HLF, HLB, HRB, HRF, VL, VB, VR, VF;
  
  struct robotSensors{
    int16_t roll;
    int16_t pitch;
    int16_t yaw;
    int16_t roll_speed;
    int16_t pitch_speed;
    int16_t yaw_speed;
    
    uint16_t pressure;
  } sensors;
  
  struct robotErrors{
    uint16_t motor_errors;
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
		uint8_t bottom_light;
		uint8_t bluetooth_light;
    uint8_t grab;
    int8_t grab_rotate;
    int8_t tilt;
  } device;
  
  struct stabilization{
    bool enable;
    bool const_time;
    float K1;
    float K2;
    float start_value;
    float gain;
    float error_speed;
  } depth_stabilization, roll_stabilization, pitch_stabilization, yaw_stabilization;
	
}; 

#endif
