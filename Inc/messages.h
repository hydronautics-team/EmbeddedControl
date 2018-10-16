#ifndef MESSAGES_H
#define MESSAGES_H

#include "stdint.h"

#pragma pack(push, 1)

/* STM send requests and VMA send responses */

#define VMA_NUMBER              	8
#define VMA_DRIVER_NUMBER       	8

#define VMA_REQUEST_LENGTH   		14
#define VMA_CONFIG_REQUEST_LENGTH  	13
#define VMA_RESPONSE_LENGTH  		12

#define VMA_NORMAL_REQUEST_TYPE 	0x01
#define VMA_CONFIG_REQUEST_TYPE 	0x02

struct vmaRequest_s
{
	uint8_t AA; // 0xAA doesn't include in CRC calculation
	uint8_t type; // 0x01
	uint8_t address;
	uint8_t update_base_vector; // true or false
	uint8_t position_setting; // enabling of position_setting
	uint16_t angle; // angle - 0..359;
	int8_t velocity;
	uint8_t frequency;
	int16_t outrunning_angle;
	uint16_t speed_k;
	uint8_t crc;
};

struct vmaConfigRequest_s
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

struct vmaResponse_s
{
	uint8_t AA;
	uint8_t type;
	uint8_t address;
	uint8_t state;
	uint8_t position_code;
	uint16_t cur_angle;
	uint16_t current;
	uint16_t speed_period;
	uint8_t crc;
};

/* STM send requests and DEV send responses */

#define DEV_REQUEST_LENGTH 			7
#define DEV_NULL					-1
#define DEV_RESPONSE_LENGTH			10
#define DEV_DRIVER_NUMBER      		4

struct devRequest_s
{
	uint8_t AA1;
	uint8_t AA2;
	uint8_t address;
	uint8_t setting;
	uint8_t velocity1;
	uint8_t velocity2;
	uint8_t checksum;
};

struct devResponse_s
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
#define REQUEST_CONFIG_LENGTH           195

struct shoreConfigRequest_s
{
    uint8_t type;

    float depth_k1;
    float depth_k2;
    float depth_k3;
    float depth_k4;
    float depth_iborders;
    float depth_pgain;
    float depth_igain;

    float roll_k1;
    float roll_k2;
    float roll_k3;
    float roll_k4;
    float roll_iborders;
    float roll_pgain;
    float roll_igain;

    float pitch_k1;
    float pitch_k2;
    float pitch_k3;
    float pitch_k4;
    float pitch_iborders;
    float pitch_pgain;
    float pitch_igain;

    float yaw_k1;
    float yaw_k2;
    float yaw_k3;
    float yaw_k4;
    float yaw_iborders;
    float yaw_pgain;
    float yaw_igain;

    uint8_t position_hlb;
    uint8_t position_hlf;
    uint8_t position_hrb;
    uint8_t position_hrf;
    uint8_t position_vb;
    uint8_t position_vf;
    uint8_t position_vl;
    uint8_t position_vr;

    uint8_t setting_hlb;
    uint8_t setting_hlf;
    uint8_t setting_hrb;
    uint8_t setting_hrf;
    uint8_t setting_vb;
    uint8_t setting_vf;
    uint8_t setting_vl;
    uint8_t setting_vr;

    uint8_t kforward_hlb;
    uint8_t kforward_hlf;
    uint8_t kforward_hrb;
    uint8_t kforward_hrf;
    uint8_t kforward_vb;
    uint8_t kforward_vf;
    uint8_t kforward_vl;
    uint8_t kforward_vr;

    uint8_t kbackward_hlb;
    uint8_t kbackward_hlf;
    uint8_t kbackward_hrb;
    uint8_t kbackward_hrf;
    uint8_t kbackward_vb;
    uint8_t kbackward_vf;
    uint8_t kbackward_vl;
    uint8_t kbackward_vr;

    uint16_t checksum;
};

#define SHORE_RESPONSE_LENGTH			72

struct shoreResponse_s
{
    int16_t roll;
    int16_t pitch;
    int16_t yaw;

    int16_t rollSpeed;
    int16_t pitchSpeed;
    int16_t yawSpeed;

    uint16_t pressure;

    uint8_t wf_type;
    uint8_t wf_tickrate;
    uint8_t wf_voltage;
    float wf_x;
    float wf_y;

    uint8_t dev_state;
    int16_t leak_data;
    int16_t in_pressure;

    uint16_t vma_current_hlb;
    uint16_t vma_current_hlf;
    uint16_t vma_current_hrb;
    uint16_t vma_current_hrf;
    uint16_t vma_current_vb;
    uint16_t vma_current_vf;
    uint16_t vma_current_vl;
    uint16_t vma_current_vr;

    int8_t vma_velocity_hlb;
    int8_t vma_velocity_hlf;
    int8_t vma_velocity_hrb;
    int8_t vma_velocity_hrf;
    int8_t vma_velocity_vb;
    int8_t vma_velocity_vf;
    int8_t vma_velocity_vl;
    int8_t vma_velocity_vr;

    uint16_t dev_current_light;
    uint16_t dev_current_tilt;
    uint16_t dev_current_grab;
    uint16_t dev_current_grab_rotate;
    uint16_t dev_current_dev1;
    uint16_t dev_current_dev2;

    uint16_t vma_errors;
    uint8_t dev_errors;
    uint8_t pc_errors;

    uint16_t checksum;
};

/* --- IMU package and parsing info  --- */
#define IMU_REQUEST_LENGTH 11 // size of transmit package
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
#define SENSORS_PACKAGE_SIZE 		4

#define SENSORS_ADC1_ADDR 			0
#define SENSORS_ADC2_ADDR 			1
#define SENSORS_PRESSURE_ADDR 		2

/* --- Delays and waiting rates --- */

#define DELAY_LED_TASK 				1000
#define DELAY_VMA_TASK 				10
#define DELAY_DEV_TASK 				10
#define DELAY_IMU_TASK 				10
#define DELAY_PC_TASK 				10
#define DELAY_SENSOR_TASK 			10
#define DELAY_STAB_TASK 			10
#define DELAY_TIMER_TASK 			10

#define WAITING_DEV 				10
#define WAITING_IMU 				10
#define WAITING_SHORE 				10
#define WAITING_VMA 				10
#define WAITING_SENSORS				10
#define WAITING_PC					10


#pragma pack(pop)

#endif
