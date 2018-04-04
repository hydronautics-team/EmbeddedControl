#include "stabilization.h"

struct PIDRegulator rollPID;
struct PIDRegulator pitchPID;
struct PIDRegulator yawPID;

void stabilizationInit(struct Robot *robot)
{
    PIDRegulatorInit(&rollPID,
            robot->rollStabCons.pid_pGain, 0,
            robot->rollStabCons.pid_iGain,
            robot->rollStabCons.pid_iMax,
            robot->rollStabCons.pid_iMin);

    PIDRegulatorInit(&pitchPID,
            robot->pitchStabCons.pid_pGain, 0,
            robot->pitchStabCons.pid_iGain,
            robot->pitchStabCons.pid_iMax,
            robot->pitchStabCons.pid_iMin);

    PIDRegulatorInit(&yawPID,
            robot->yawStabCons.pid_pGain, 0,
            robot->yawStabCons.pid_iGain,
            robot->yawStabCons.pid_iMax,
            robot->yawStabCons.pid_iMin);
}

void stabilizeRoll(struct Robot *robot)
{
    // Integration
    robot->rollStabSt.joy_iValue += robot->f_joySpeed.roll * fromTickToMs(xTaskGetTickCount() - robot->rollStabSt.joy_iLastTick) / 1000.0f;
    robot->rollStabSt.joy_iLastTick = xTaskGetTickCount();

    // Position feedback
    robot->rollStabSt.posError = robot->rollStabSt.joy_iValue - robot->f_sensors.roll;

    // Feedback amplifier
    robot->rollStabSt.posErrorAmp = robot->rollStabSt.posError * robot->rollStabCons.pErrGain;

    // PID regulation
    robot->rollStabSt.pidValue = update(&rollPID, robot->rollStabSt.posErrorAmp, fromTickToMs(xTaskGetTickCount() - rollPID.lastUpdateTick));

    // Dynamic summator
    robot->rollStabSt.dynSummator = robot->rollStabSt.pidValue + robot->f_joySpeed.roll * robot->rollStabCons.pSpeedDyn;

    // Speed feedback
    robot->rollStabSt.speedError = robot->rollStabSt.dynSummator - robot->f_sensors.rollSpeed * robot->rollStabCons.pSpeedFback;
}


void stabilizePitch(struct Robot *robot)
{
    // Integration
    robot->pitchStabSt.joy_iValue += robot->f_joySpeed.pitch * fromTickToMs(xTaskGetTickCount() - robot->pitchStabSt.joy_iLastTick) / 1000.0f;
    robot->pitchStabSt.joy_iLastTick = xTaskGetTickCount();

    // Position feedback
    robot->pitchStabSt.posError = robot->pitchStabSt.joy_iValue - robot->f_sensors.pitch;

    // Feedback amplifier
    robot->pitchStabSt.posErrorAmp = robot->pitchStabSt.posError * robot->pitchStabCons.pErrGain;

    // PID regulation
    robot->pitchStabSt.pidValue = update(&rollPID, robot->pitchStabSt.posErrorAmp, fromTickToMs(xTaskGetTickCount() - rollPID.lastUpdateTick));

    // Dynamic summator
    robot->pitchStabSt.dynSummator = robot->pitchStabSt.pidValue + robot->f_joySpeed.pitch * robot->pitchStabCons.pSpeedDyn;

    // Speed feedback
    robot->pitchStabSt.speedError = robot->pitchStabSt.dynSummator - robot->f_sensors.pitchSpeed * robot->pitchStabCons.pSpeedFback;
}

void stabilizeYaw(struct Robot *robot)
{
    // Integration
    robot->yawStabSt.joy_iValue += robot->f_joySpeed.yaw * fromTickToMs(xTaskGetTickCount() - robot->yawStabSt.joy_iLastTick) / 1000.0f;
    robot->yawStabSt.joy_iLastTick = xTaskGetTickCount();

    // Position feedback
    robot->yawStabSt.posError = robot->yawStabSt.joy_iValue - robot->f_sensors.yaw;

    // Feedback amplifier
    robot->yawStabSt.posErrorAmp = robot->yawStabSt.posError * robot->yawStabCons.pErrGain;

    // PID regulation
    robot->yawStabSt.pidValue = update(&rollPID, robot->yawStabSt.posErrorAmp, fromTickToMs(xTaskGetTickCount() - rollPID.lastUpdateTick));

    // Dynamic summator
    robot->yawStabSt.dynSummator = robot->yawStabSt.pidValue + robot->f_joySpeed.yaw * robot->yawStabCons.pSpeedDyn;

    // Speed feedback
    robot->yawStabSt.speedError = robot->yawStabSt.dynSummator - robot->f_sensors.yawSpeed * robot->yawStabCons.pSpeedFback;
}
