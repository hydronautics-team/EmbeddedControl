#ifndef MESSAGES_H
#define MESSAGES_H

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

///* Shore send requests and STM send responses */
///* --- SHORE REQUEST NORMAL MODE --- */
//#define SHORE_REQUEST_CODE             0xA5

//#define SHORE_REQUEST_LENGTH           27

//#define SHORE_REQUEST_TYPE             1
//#define SHORE_REQUEST_FLAGS            2
//#define SHORE_REQUEST_MARCH            3
//#define SHORE_REQUEST_LAG              5
//#define SHORE_REQUEST_DEPTH            7
//#define SHORE_REQUEST_ROLL             9
//#define SHORE_REQUEST_PITCH            11
//#define SHORE_REQUEST_YAW              13

//#define SHORE_REQUEST_LIGHT            15
//#define SHORE_REQUEST_GRAB             16
//#define SHORE_REQUEST_TILT             17
//#define SHORE_REQUEST_GRAB_ROTATE      18
//#define SHORE_REQUEST_AGAR             19
//#define SHORE_REQUEST_BOTTOM_LIGHT     20

//#define SHORE_REQUEST_STABILIZE_DEPTH  21
//#define SHORE_REQUEST_STABILIZE_ROLL   22
//#define SHORE_REQUEST_STABILIZE_PITCH  23
//#define SHORE_REQUEST_STABILIZE_YAW    24
//#define SHORE_REQUEST_RESET_IMU        25

//#define SHORE_REQUEST_CHECKSUM         26



/* Shore send requests and STM send responses */
/* --- SHORE REQUEST NORMAL MODE --- */
//#define SHORE_REQUEST_CODE             0xFF
//#define SHORE_REQUEST_CODE             0xA5

//#define SHORE_REQUEST_LENGTH           27

//#define SHORE_REQUEST_TYPE             0
//#define SHORE_REQUEST_FLAGS            1//0
//#define SHORE_REQUEST_MARCH            2//1
//#define SHORE_REQUEST_LAG              4//3
//#define SHORE_REQUEST_DEPTH            6//5
//#define SHORE_REQUEST_ROLL             8//7
//#define SHORE_REQUEST_PITCH            10//9
//#define SHORE_REQUEST_YAW              12//11

//#define SHORE_REQUEST_LIGHT            14//13
//#define SHORE_REQUEST_GRAB             15//14
//#define SHORE_REQUEST_TILT             16//15
//#define SHORE_REQUEST_GRAB_ROTATE      17//16
//#define SHORE_REQUEST_AGAR             18//17
//#define SHORE_REQUEST_BOTTOM_LIGHT     19//18

//#define SHORE_REQUEST_STABILIZE_DEPTH  20//19
//#define SHORE_REQUEST_STABILIZE_ROLL   21//20
//#define SHORE_REQUEST_STABILIZE_PITCH  22//21
//#define SHORE_REQUEST_STABILIZE_YAW    23//22
//#define SHORE_REQUEST_RESET_IMU        24//23

//#define SHORE_REQUEST_CHECKSUM         25//24

#define SHORE_REQUEST_CODE             0xA5

#define SHORE_REQUEST_LENGTH           27

#define SHORE_REQUEST_TYPE             0
#define SHORE_REQUEST_FLAGS            1//0
#define SHORE_REQUEST_MARCH            2//1
#define SHORE_REQUEST_LAG              4//3
#define SHORE_REQUEST_DEPTH            6//5
#define SHORE_REQUEST_ROLL             8//7
#define SHORE_REQUEST_PITCH            10//9
#define SHORE_REQUEST_YAW              12//11

#define SHORE_REQUEST_LIGHT            14//13
#define SHORE_REQUEST_GRAB             15//14
#define SHORE_REQUEST_TILT             16//15
#define SHORE_REQUEST_GRAB_ROTATE      17//16
#define SHORE_REQUEST_AGAR             18//17
#define SHORE_REQUEST_BOTTOM_LIGHT     19//18

#define SHORE_REQUEST_STABILIZE_DEPTH  20//19
#define SHORE_REQUEST_STABILIZE_ROLL   21//20
#define SHORE_REQUEST_STABILIZE_PITCH  22//21
#define SHORE_REQUEST_STABILIZE_YAW    23//22
#define SHORE_REQUEST_RESET_IMU        24//23

#define SHORE_REQUEST_CHECKSUM         25//24






/* --- SHORE REQUEST CONFIG MODE --- */
// ? ??????????
#define REQUEST_CONFIG_CODE             0x55

#define REQUEST_CONFIG_LENGTH           152

#define REQUEST_CONFIG_TYPE             1

#define REQUEST_CONFIG_CONST_TIME_DEPTH 2
#define REQUEST_CONFIG_CONST_TIME_ROLL  3
#define REQUEST_CONFIG_CONST_TIME_PITCH 4
#define REQUEST_CONFIG_CONST_TIME_YAW   5

#define REQUEST_CONFIG_K1_DEPTH         6
#define REQUEST_CONFIG_K2_DEPTH         10
#define REQUEST_CONFIG_START_DEPTH      14
#define REQUEST_CONFIG_GAIN_DEPTH       18

#define REQUEST_CONFIG_K1_ROLL          22
#define REQUEST_CONFIG_K2_ROLL          26
#define REQUEST_CONFIG_START_ROLL       30
#define REQUEST_CONFIG_GAIN_ROLL        34

#define REQUEST_CONFIG_K1_PITCH         38
#define REQUEST_CONFIG_K2_PITCH         42
#define REQUEST_CONFIG_START_PITCH      46
#define REQUEST_CONFIG_GAIN_PITCH       50

#define REQUEST_CONFIG_K1_YAW           54
#define REQUEST_CONFIG_K2_YAW           58
#define REQUEST_CONFIG_START_YAW        62
#define REQUEST_CONFIG_GAIN_YAW         66

#define REQUEST_CONFIG_POSITION_HLB     70
#define REQUEST_CONFIG_POSITION_HLF     71
#define REQUEST_CONFIG_POSITION_HRB     72
#define REQUEST_CONFIG_POSITION_HRF     73
#define REQUEST_CONFIG_POSITION_VB      74
#define REQUEST_CONFIG_POSITION_VF      75
#define REQUEST_CONFIG_POSITION_VL      76
#define REQUEST_CONFIG_POSITION_VR      77

#define REQUEST_CONFIG_SETTING_HLB      78
#define REQUEST_CONFIG_SETTING_HLF      79
#define REQUEST_CONFIG_SETTING_HRB      80
#define REQUEST_CONFIG_SETTING_HRF      81
#define REQUEST_CONFIG_SETTING_VB       82
#define REQUEST_CONFIG_SETTING_VF       83
#define REQUEST_CONFIG_SETTING_VL       84
#define REQUEST_CONFIG_SETTING_VR       85

#define REQUEST_CONFIG_K_FORWARD_HLB    86
#define REQUEST_CONFIG_K_FORWARD_HLF    90
#define REQUEST_CONFIG_K_FORWARD_HRB    94
#define REQUEST_CONFIG_K_FORWARD_HRF    98
#define REQUEST_CONFIG_K_FORWARD_VB     102
#define REQUEST_CONFIG_K_FORWARD_VF     106
#define REQUEST_CONFIG_K_FORWARD_VL     110
#define REQUEST_CONFIG_K_FORWARD_VR     114

#define REQUEST_CONFIG_K_BACKWARD_HLB   118
#define REQUEST_CONFIG_K_BACKWARD_HLF   122
#define REQUEST_CONFIG_K_BACKWARD_HRB   126
#define REQUEST_CONFIG_K_BACKWARD_HRF   130
#define REQUEST_CONFIG_K_BACKWARD_VB    134
#define REQUEST_CONFIG_K_BACKWARD_VF    138
#define REQUEST_CONFIG_K_BACKWARD_VL    142
#define REQUEST_CONFIG_K_BACKWARD_VR    146

#define REQUEST_CONFIG_CHECKSUM         150


/* --- SHORE RESPONSE MODE --- */
#define SHORE_RESPONSE_LENGTH                 63

#define SHORE_RESPONSE_ROLL                   0
#define SHORE_RESPONSE_PITCH                  2
#define SHORE_RESPONSE_YAW                    4

#define SHORE_RESPONSE_ROLL_SPEED             6
#define SHORE_RESPONSE_PITCH_SPEED            8
#define SHORE_RESPONSE_YAW_SPEED              10

#define SHORE_RESPONSE_PRESSURE               12

#define SHORE_RESPONSE_BLUETOOTH              14

#define SHORE_RESPONSE_VMA_CURRENT_HLB        22
#define SHORE_RESPONSE_VMA_CURRENT_HLF        24
#define SHORE_RESPONSE_VMA_CURRENT_HRB        26
#define SHORE_RESPONSE_VMA_CURRENT_HRF        28
#define SHORE_RESPONSE_VMA_CURRENT_VB         30
#define SHORE_RESPONSE_VMA_CURRENT_VF         32
#define SHORE_RESPONSE_VMA_CURRENT_VL         34
#define SHORE_RESPONSE_VMA_CURRENT_VR         36


#define SHORE_RESPONSE_VMA_VELOCITY_HLB       38
#define SHORE_RESPONSE_VMA_VELOCITY_HLF       39
#define SHORE_RESPONSE_VMA_VELOCITY_HRB       40
#define SHORE_RESPONSE_VMA_VELOCITY_HRF       41
#define SHORE_RESPONSE_VMA_VELOCITY_VB        42
#define SHORE_RESPONSE_VMA_VELOCITY_VF        43
#define SHORE_RESPONSE_VMA_VELOCITY_VL        44
#define SHORE_RESPONSE_VMA_VELOCITY_VR        45

#define SHORE_RESPONSE_LIGHT_CURRENT          46
#define SHORE_RESPONSE_BOTTOM_LIGHT_CURRENT   48
#define SHORE_RESPONSE_AGAR_CURRENT           50
#define SHORE_RESPONSE_GRAB_CURRENT           52
#define SHORE_RESPONSE_GRAB_ROTATE_CURRENT    54
#define SHORE_RESPONSE_CURRENT_TILT           56

#define SHORE_RESPONSE_VMA_ERRORS             58

#define SHORE_RESPONSE_DEV_ERRORS             60

#define SHORE_RESPONSE_CHECKSUM               61




/* --- IMU input message --- */
#define IMU_REQUEST_LENGTH                  22
#define IMU_RESPONSE_LENGTH                 22


/* --- IMU command --- */



#endif
