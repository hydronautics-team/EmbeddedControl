#ifndef SENSORS_H
#define SENSORS_H

#include "robot.h"


#define PRESSURE_Addr
#define IN_PRESSURE_Addr 0xEE
#define LEAK_0_Addr
#define LEAK_1_Addr

#define IN_PRESSURE_CONTR_REG 0xF4
#define IN_PRESSURE_DATA_REG 0xF6
#define IN_PRESSURE_CALIB_REG 0xAA
#define IN_PRESSURE_ID_REG 0xD0

#define IN_PRESSURE_MEASURMENT_PRES 0x34
#define IN_PRESSURE_MEASURMENT_TEMP 0x2E
#define IN_PRESSURE_MEASURMENT_COMPLITE 0x20
#define IN_PRESSURE_ID 0x55

#define MEASUREMENT_DELAY 8

enum STATUSES {
	S_OK = 0,
	ERR_SENS_NOT_DETECTED,
	ERR_SENS_NOT_REQUEST,
	ERR_SENS_WRONG_ANSWER
};

enum SENS {
	SENS_PRESSURE = 0b1,
	SENS_IN_PRESSURE = 0b10,
	SENS_LEAK_0 = 0b100,
	SENS_LEAK_1 = 0b1000
};

struct SensorsConstants {
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
		uint32_t UT;
		uint32_t UP;
		uint8_t oss;
	} InPressureCons;
	
	struct LeakConstants {
		
	} LeakCons;
};

uint8_t SensorsInit(struct Robot *robot, uint8_t sensors);
uint8_t StartMeansurements(struct Robot *robot, uint8_t sensors);
uint8_t ReliseMeansurements(struct Robot *robot, uint8_t sensors);
void CulculationResults(struct Robot *robot, uint8_t sensors);

#endif