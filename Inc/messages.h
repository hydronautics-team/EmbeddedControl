#ifndef MESSAGES_H
#define MESSAGES_H

#include <stdint.h>
#include <stdlib.h>
#include "robot.h"
#include "checksum.h"

/* STM send requests and VMA send responses */
#define VMA_DEV_REQUEST_LENGTH              7

#define VMA_DEV_REQUEST_AA1                 0
#define VMA_DEV_REQUEST_AA2                 1
#define VMA_DEV_REQUEST_ADDRESS             2
#define VMA_DEV_REQUEST_SETTING             3
#define VMA_DEV_REQUEST_VELOCITY1           4
#define VMA_DEV_REQUEST_VELOCITY2           5
#define VMA_DEV_REQUEST_CHECKSUM            6


#define VMA_DEV_RESPONSE_LENGTH             10

#define VMA_DEV_RESPONSE_AA                 0
#define VMA_DEV_RESPONSE_ADDRESS            1
#define VMA_DEV_RESPONSE_ERRORS             2
#define VMA_DEV_RESPONSE_CURRENT_1H         3
#define VMA_DEV_RESPONSE_CURRENT_1L         4
#define VMA_DEV_RESPONSE_CURRENT_2H         5
#define VMA_DEV_RESPONSE_CURRENT_2L         6
#define VMA_DEV_RESPONSE_VELOCITY1					7
#define VMA_DEV_RESPONSE_VELOCITY2					8
#define VMA_DEV_RESPONSE_CHECKSUM           9

/* Shore send requests and STM send responses */
/* --- SHORE REQUEST NORMAL MODE --- */
#define SHORE_REQUEST_CODE             0xFF

#define SHORE_REQUEST_LENGTH           26

#define SHORE_REQUEST_TYPE             0
#define SHORE_REQUEST_MARCH            1
#define SHORE_REQUEST_LAG              3
#define SHORE_REQUEST_DEPTH            5
#define SHORE_REQUEST_ROLL             7
#define SHORE_REQUEST_PITCH            9
#define SHORE_REQUEST_YAW              11

#define SHORE_REQUEST_LIGHT            13
#define SHORE_REQUEST_GRAB             14
#define SHORE_REQUEST_TILT             15
#define SHORE_REQUEST_GRAB_ROTATE      16
#define SHORE_REQUEST_BLUETOOTH        17
#define SHORE_REQUEST_BOTTOM_LIGHT     18

#define SHORE_REQUEST_STABILIZE_DEPTH  19
#define SHORE_REQUEST_STABILIZE_ROLL   20
#define SHORE_REQUEST_STABILIZE_PITCH  21
#define SHORE_REQUEST_STABILIZE_YAW    22
#define SHORE_REQUEST_RESET_IMU        23

#define SHORE_REQUEST_CHECKSUM         24



/* --- SHORE REQUEST CONFIG MODE --- */
#define REQUEST_CONFIG_CODE             0x55

#define REQUEST_CONFIG_LENGTH           151

#define REQUEST_CONFIG_TYPE             0

#define REQUEST_CONFIG_CONST_TIME_DEPTH 1
#define REQUEST_CONFIG_CONST_TIME_ROLL  2
#define REQUEST_CONFIG_CONST_TIME_PITCH 3
#define REQUEST_CONFIG_CONST_TIME_YAW   4

#define REQUEST_CONFIG_K1_DEPTH         5
#define REQUEST_CONFIG_K2_DEPTH         9
#define REQUEST_CONFIG_START_DEPTH      13
#define REQUEST_CONFIG_GAIN_DEPTH       17

#define REQUEST_CONFIG_K1_ROLL          21
#define REQUEST_CONFIG_K2_ROLL          25
#define REQUEST_CONFIG_START_ROLL       29
#define REQUEST_CONFIG_GAIN_ROLL        33

#define REQUEST_CONFIG_K1_PITCH         37
#define REQUEST_CONFIG_K2_PITCH         41
#define REQUEST_CONFIG_START_PITCH      45
#define REQUEST_CONFIG_GAIN_PITCH       49

#define REQUEST_CONFIG_K1_YAW           53
#define REQUEST_CONFIG_K2_YAW           57
#define REQUEST_CONFIG_START_YAW        61
#define REQUEST_CONFIG_GAIN_YAW         65

#define REQUEST_CONFIG_POSITION_HLB     69
#define REQUEST_CONFIG_POSITION_HLF     70
#define REQUEST_CONFIG_POSITION_HRB     71
#define REQUEST_CONFIG_POSITION_HRF     72
#define REQUEST_CONFIG_POSITION_VB      73
#define REQUEST_CONFIG_POSITION_VF      74
#define REQUEST_CONFIG_POSITION_VL      75
#define REQUEST_CONFIG_POSITION_VR      76

#define REQUEST_CONFIG_SETTING_HLB      77
#define REQUEST_CONFIG_SETTING_HLF      78
#define REQUEST_CONFIG_SETTING_HRB      79
#define REQUEST_CONFIG_SETTING_HRF      80
#define REQUEST_CONFIG_SETTING_VB       81
#define REQUEST_CONFIG_SETTING_VF       82
#define REQUEST_CONFIG_SETTING_VL       83
#define REQUEST_CONFIG_SETTING_VR       84

#define REQUEST_CONFIG_K_FORWARD_HLB    85
#define REQUEST_CONFIG_K_FORWARD_HLF    89
#define REQUEST_CONFIG_K_FORWARD_HRB    93
#define REQUEST_CONFIG_K_FORWARD_HRF    97
#define REQUEST_CONFIG_K_FORWARD_VB     101
#define REQUEST_CONFIG_K_FORWARD_VF     105
#define REQUEST_CONFIG_K_FORWARD_VL     109
#define REQUEST_CONFIG_K_FORWARD_VR     113

#define REQUEST_CONFIG_K_BACKWARD_HLB   117
#define REQUEST_CONFIG_K_BACKWARD_HLF   121
#define REQUEST_CONFIG_K_BACKWARD_HRB   125
#define REQUEST_CONFIG_K_BACKWARD_HRF   129
#define REQUEST_CONFIG_K_BACKWARD_VB    133
#define REQUEST_CONFIG_K_BACKWARD_VF    137
#define REQUEST_CONFIG_K_BACKWARD_VL    141
#define REQUEST_CONFIG_K_BACKWARD_VR    145

#define REQUEST_CONFIG_CHECKSUM         149


/* --- SHORE RESPONSE MODE --- */
#define SHORE_RESPONSE_LENGTH                 20

#define SHORE_RESPONSE_ROLL                   0
#define SHORE_RESPONSE_PITCH                  2
#define SHORE_RESPONSE_YAW                    4

#define SHORE_RESPONSE_ROLL_SPEED             6
#define SHORE_RESPONSE_PITCH_SPEED            8
#define SHORE_RESPONSE_YAW_SPEED              10

#define SHORE_RESPONSE_PRESSURE               14

#define SHORE_RESPONSE_MOTOR_ERRORS           16

#define SHORE_RESPONSE_CHECKSUM               18

// add 8x2 byte current VMA + 8 byte velocity VMA + 8 byte errors + 6x2 Dev current + byte 0x00EEEEEE




/* --- IMU input message --- */
#define IMU_REQUEST_LENGTH                  22
#define IMU_RESPONSE_LENGTH                 22


/* --- IMU command --- */



void DevRequestUpdate(struct Robot *robot, uint8_t *buf, uint8_t DEV);
void VMARequestUpdate(struct Robot *robot, uint8_t *buf, uint8_t DEV);

void ShoreRequest(struct Robot *robot, uint8_t *requestBuf);
void ShoreConfigRequest(struct Robot *robot, uint8_t *requestBuf);

void ShoreResponse(struct Robot *robot, uint8_t *responseBuf);

void IMUResponse(struct Robot *robot, uint8_t *IMUResponseBuf);



#endif
