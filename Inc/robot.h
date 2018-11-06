#ifndef ROBOT_H
#define ROBOT_H

#include <stdbool.h>
#include <math.h>
#include <stdint.h>

#include "messages.h"

enum VMA {
    HLB = 0,
    HLF,
    HRB,
    HRF,
    VB,
    VF,
    VL,
    VR
};

#define DEV_AMOUNT 7

enum DEV {
    AGAR = 0,
    GRAB,
    GRAB_ROTATION,
    TILT,
    LIGHT,
    DEV1,
	DEV2
};

#define LOGDEV_AMOUNT 1

enum LOGDEV {
	ACOUSTIC = 0
};

enum I2C {
	I2C_SENSORS,
	I2C_PC
};

struct Robot {

	uint8_t cameraNum;

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

  struct f_Sensors {
    float roll;
    float pitch;
    float yaw;
    float rollSpeed;
    float pitchSpeed;
    float yawSpeed;
    float accelX;
    float accelY;
    float accelZ;
    float magX;
    float magY;
    float magZ;
    float quatA;
    float quatB;
    float quatC;
    float quatD;
    bool resetIMU;
    float pressure;
    float in_pressure;
		float in_temp;
    float leak;
  } f_sensors;

  struct i_Sensors {
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
    uint32_t in_pressure;
		uint32_t in_temp;
    uint16_t leak;
  } i_sensors;

    struct robotWifi {
        uint8_t type;
        uint8_t tickrate;
        uint8_t voltage;
        float x;
        float y;
    } wifi;

    struct robotPC {
    	uint8_t reset;
    	uint8_t errors;
    } pc;

    struct i_JoystickRobotSpeed {
        int16_t march;
        int16_t lag;
        int16_t depth;
        int16_t roll;
        int16_t pitch;
        int16_t yaw;
    } i_joySpeed;

    struct f_JoystickRobotSpeed {
        float march;
        float lag;
        float depth;
        float roll;
        float pitch;
        float yaw;
    } f_joySpeed;

    struct i_PositionRobotMovement {
    	int16_t march;
    	int16_t lag;
    	int16_t depth;
    	int16_t roll;
    	int16_t pitch;
    	int16_t yaw;
    } i_posMov;

    struct f_PositionRobotMovement {
        float march;
        float lag;
        float depth;
        float roll;
        float pitch;
        float yaw;
    } f_posMov;


    struct RobotDev {
    	uint8_t address;
    	uint8_t settings;
    	int8_t force;
    	uint16_t current;
    	uint8_t errors;
    } device[DEV_AMOUNT];

    struct RobotLogDev {
    	uint8_t state;
    	uint8_t control;
        uint8_t errors;
    } logdevice[LOGDEV_AMOUNT];

    struct RobotStabilizationConstants {
        bool enable;
        // Before P
    	uint8_t cameras;
        float iJoySpeed;
        float pSpeedDyn;
        float pErrGain;
        // Feedback
        float pSpeedFback;
        // PID
        float pid_pGain;
        float pid_iGain;
        float pid_iMax;
        float pid_iMin;
    } depthStabCons, rollStabCons, pitchStabCons, yawStabCons;

    struct RobotStabilizationState {
        float posError;
        float speedError;
        float dynSummator;
        float pidValue;
        float posErrorAmp;

        float joy_iValue;
        float joy_iLastTick;
    } depthStabSt, rollStabSt, pitchStabSt, yawStabSt;
		
		struct SensorsConstants {
			uint8_t sensorsAddr[SENSORS_DEVICES_NUM];
			uint8_t sensorsDelays[SENSORS_DEVICES_NUM];
			uint8_t sensorsResponseLength[SENSORS_DEVICES_NUM];
			uint8_t sensorsRequestAddr[SENSORS_DEVICES_NUM];
			uint8_t sensorsRequestStartByte[SENSORS_DEVICES_NUM];
			
			struct PressureConstants {
		
			} PressureCons;
	
			struct InPressureConstants {
				int16_t AC1;
				int16_t AC2;
				int16_t AC3;
				uint16_t AC4;
				uint16_t AC5; 
				uint16_t AC6;
				int16_t B1;
				int16_t B2;
				int32_t B3;
				uint32_t B4;
				int16_t MB;
				int16_t MC;
				int16_t MD;
				uint8_t oss;
			} InPressureCons;
			
			struct LeakConstants {
				
			} LeakCons;
			
		} SensCons;

};

#endif
