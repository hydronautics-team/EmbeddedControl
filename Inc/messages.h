#ifndef MESSAGES_H
#define MESSAGES_H

#include "stdint.h"

#pragma pack(push, 1)

/* STM send requests and VMA send responses */
#define THRUSTERS_NUMBER             		8

#define THRUSTERS_REQUEST_LENGTH   			5
#define THRUSTERS_CONFIG_REQUEST_LENGTH  	13
#define THRUSTERS_RESPONSE_LENGTH  			9

#define THRUSTERS_NORMAL_REQUEST_TYPE 		0x01
#define THRUSTERS_CONFIG_REQUEST_TYPE 		0x02

struct thrustersRequest_s
{
	uint8_t AA;
	uint8_t type; // 0x01
	uint8_t address;
	int8_t velocity;
	uint8_t crc;
};

struct thrustersConfigRequest_s
{
	uint8_t AA;
	uint8_t type; // 0x02
	uint8_t update_firmware; // (bool) set new address or update firmware even if old address doesn't equal BLDC address
	uint8_t forse_setting; // (bool) set new address even if old address doesn't equal BLDC address
	uint8_t old_address;
	uint8_t new_address;
	uint16_t high_threshold;
	uint16_t low_threshold;
	uint16_t average_threshold;
	uint8_t crc;
};

struct thrustersResponse_s
{
	uint8_t AA;
	uint8_t type; // 0x01
	uint8_t address;
	uint8_t state;
	uint16_t current;
	uint16_t speed_period;
	uint8_t crc;
};

/* STM send requests and DEV send responses */

#define DEVICES_REQUEST_LENGTH 			7
#define DEVICES_NULL					-1
#define DEVICES_RESPONSE_LENGTH			10
#define DEVICES_NUMBER      			4

struct devicesRequest_s
{
	uint8_t AA1;
	uint8_t AA2;
	uint8_t address;
	uint8_t setting;
	uint8_t velocity1;
	uint8_t velocity2;
	uint8_t checksum;
};

struct devicesResponse_s
{
    uint8_t AA;
    uint8_t address;
    uint8_t errors;
    uint16_t current1;
    uint16_t current2;
    uint8_t velocity1;
    uint8_t velocity2;
    uint8_t checksum;
};

///* Shore send requests and STM send responses */
///* --- SHORE REQUEST NORMAL MODE --- */
#define SHORE_REQUEST_CODE             0xA5

#define SHORE_REQUEST_LENGTH           26

#define SHORE_STABILIZE_DEPTH_BIT 		0
#define SHORE_STABILIZE_ROLL_BIT 		1
#define SHORE_STABILIZE_PITCH_BIT 		2
#define SHORE_STABILIZE_YAW_BIT 		3
#define SHORE_STABILIZE_IMU_BIT 		4

#define SHORE_DEVICE_AC_BIT 			0

#define PC_ON_CODE 0xAA
#define PC_OFF_CODE 0x00

struct shoreRequest_s
{
	uint8_t type;
	uint8_t flags;
	int16_t march;
	int16_t lag;
	int16_t depth;
	int16_t roll;
	int16_t pitch;
	int16_t yaw;
	int8_t light;
	int8_t grab;
	int8_t tilt;
	int8_t grab_rotate;
	int8_t dev1;
	int8_t dev2;
	uint8_t dev_flags;
	uint8_t stabilize_flags;
	uint8_t cameras;
	uint8_t pc_reset;
	uint16_t checksum;
};

#define REQUEST_CONFIG_CODE             0x55
#define REQUEST_CONFIG_LENGTH           72

struct shoreConfigRequest_s
{
    uint8_t type;
    uint8_t contour;

    int16_t march;
    int16_t lag;
    int16_t depth;
    int16_t roll;
    int16_t pitch;
    int16_t yaw;

	float pJoyUnitCast;
	float pSpeedDyn;
	float pErrGain;

	float posFilterT;
	float posFilterK;
	float speedFilterT;
	float speedFilterK;

	float pid_pGain;
	float pid_iGain;
	float pid_iMax;
	float pid_iMin;

	float pThrustersCast;
	float pThrustersMin;
	float pThrustersMax;

    uint16_t checksum;
};

#define SHORE_RESPONSE_LENGTH			70

struct shoreResponse_s
{
    float roll;
    float pitch;
    float yaw;

    float rollSpeed;
    float pitchSpeed;
    float yawSpeed;

    float pressure;
    float in_pressure;

    uint8_t dev_state;
    int16_t leak_data;

    uint16_t vma_current_hlb;
    uint16_t vma_current_hlf;
    uint16_t vma_current_hrb;
    uint16_t vma_current_hrf;
    uint16_t vma_current_vb;
    uint16_t vma_current_vf;
    uint16_t vma_current_vl;
    uint16_t vma_current_vr;

    uint16_t dev_current_light;
    uint16_t dev_current_tilt;
    uint16_t dev_current_grab;
    uint16_t dev_current_grab_rotate;
    uint16_t dev_current_dev1;
    uint16_t dev_current_dev2;

    uint16_t vma_errors;
    uint16_t dev_errors;
    uint8_t pc_errors;

    uint16_t checksum;
};

#define SHORE_CONFIG_RESPONSE_LENGTH			91

struct shoreConfigResponse_s
{
	uint8_t code;

	float roll;
	float pitch;
	float yaw;
	float raw_yaw;

	float rollSpeed;
	float pitchSpeed;
	float yawSpeed;

	float pressure;
	float in_pressure;

	float inputSignal;
	float speedSignal;
	float posSignal;

	float joyUnitCasted;
	float joy_iValue;
	float posError;
	float speedError;
	float dynSummator;
	float pidValue;
	float posErrorAmp;
	float speedFiltered;
	float posFiltered;
	float pid_iValue;

    uint16_t checksum;
};

#define SHORE_REQUEST_MODES_NUMBER 2

enum ShoreRequestModes {
	SHORE_REQUEST_NORMAL = 0,
	SHORE_REQUEST_CONFIG,
	SHORE_REQUEST_DIRECT
};

/* --- IMU package and parsing info  --- */
#define IMU_REQUEST_LENGTH 7 // size of transmit package
#define IMU_RESPONSE_LENGTH 15 // size of receive package
#define IMU_CHECKSUMS 5 // amount of packages

struct imuResponse_s
{
    uint16_t gyro_x;
    uint16_t gyro_y;
    uint16_t gyro_z;

    uint16_t accel_x;
    uint16_t accel_y;
    uint16_t accel_z;

    uint16_t mag_x;
    uint16_t mag_y;
    uint16_t mag_z;

    uint16_t euler_x;
    uint16_t euler_y;
    uint16_t euler_z;

    uint16_t quat_a;
    uint16_t quat_b;
    uint16_t quat_c;
    uint16_t quat_d;
};

#define GYRO_PROC_X 5 // 0x5C
#define GYRO_PROC_Y 7 // 0x5C
#define GYRO_PROC_Z 9 // 0x5D
#define ACCEL_PROC_X 20 // 0x5E
#define ACCEL_PROC_Y 22 // 0x5E
#define ACCEL_PROC_Z 24 // 0x5F
#define MAG_PROC_X 35 // 0x60
#define MAG_PROC_Y 37 // 0x60
#define MAG_PROC_Z 39 // 0x61
#define EULER_PHI 50 // 0x62
#define EULER_TETA 52 // 0x62
#define EULER_PSI 54 // 0x63
#define QUAT_A 65 // 0x64
#define QUAT_B 67 // 0x64
#define QUAT_C 69 // 0x65
#define QUAT_D 71 // 0x65

/* --- I2C2 Sensors communication info --- */

#define SENSORS_DEVICES_NUM 		3
#define PRESSURE_SENSOR_SIZE 		4

#define SENSORS_ADC1_ADDR 			0
#define SENSORS_ADC2_ADDR 			1
#define SENSORS_PRESSURE_ADDR 		15

/* --- Delays and waiting rates --- */

#define DELAY_LED_TASK 				1000
#define DELAY_THRUSTERS_TASK 		20
#define DELAY_DEVICES_TASK 			10
#define DELAY_IMU_TASK 				10
#define DELAY_PC_TASK 				10
#define DELAY_SENSOR_TASK 			25
#define DELAY_STABILIZATION_TASK 	10
#define DELAY_TIMER_TASK 			30
#define DELAY_SILENCE    			1000
#define DELAY_UART_TIMEOUT    		5

#define WAITING_DEVICES 			10
#define WAITING_IMU 				10
#define WAITING_SHORE 				10
#define WAITING_THRUSTERS 			10
#define WAITING_SENSORS				5
#define WAITING_PC					10
#define WAITING_TIMER				5
#define UART_SWITCH_DELAY			1000


#pragma pack(pop)

#endif
