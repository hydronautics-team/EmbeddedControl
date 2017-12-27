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

enum DEV {
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
    int16_t pressure;
  } i_sensors;

    struct robotBluetooth {
        uint8_t message[BT_SIZE];
    } bluetooth;

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

    struct RobotDevice {

        struct RobotLight {
            uint8_t address;
            uint8_t settings;
            int8_t brightness;
            uint16_t current;
        } light, bottomLight;

        struct RobotGrab  {
            uint8_t squeezeAddress;
            uint8_t rotationAddress;
            uint8_t settings;
            int8_t squeeze;
            int8_t rotation;
            uint16_t squeezeCurrent;
            uint16_t rotationCurrent;
        } grab;

        struct RobotTilt {
            uint8_t address;
            uint8_t settings;
            int8_t rotation;
            uint16_t current;
        } tilt;

        struct RobotAgarGrab {
            uint8_t address;
            uint8_t settings;
            uint8_t opening;
            uint16_t current;
        } agar;
        uint8_t errors;
    } device;

    struct RobotStabilizationConstants {
        bool enable;
        bool iPartEnable;
        bool posMode;
        // Before PID
        float iJoySpeed;
        float pSpeedDyn;
        float pErrGain;
        // PID
        float pid_pGain;
        float pid_iGain;
        float pid_iMax;
        float pid_iMin;
        // Feedback
        float pPosFback;
        float pSpeedFback;
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

};

#endif
