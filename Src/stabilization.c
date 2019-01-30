#include "stabilization.h"

#include "FreeRTOSTick.h"
#include "math.h"
#include "robot.h"

struct PIDRegulator rollPID;
struct PIDRegulator pitchPID;
struct PIDRegulator yawPID;
struct PIDRegulator depthPID;

void stabilizationInit(struct Robot *robot)
{
    PIDRegulatorInit(&rollPID,
            robot->stabConstants[STAB_ROLL].pid.pGain, 0,
			robot->stabConstants[STAB_ROLL].pid.iGain,
			robot->stabConstants[STAB_ROLL].pid.iMax,
			robot->stabConstants[STAB_ROLL].pid.iMin);

    PIDRegulatorInit(&pitchPID,
            robot->stabConstants[STAB_PITCH].pid.pGain, 0,
			robot->stabConstants[STAB_PITCH].pid.iGain,
			robot->stabConstants[STAB_PITCH].pid.iMax,
			robot->stabConstants[STAB_PITCH].pid.iMin);

    PIDRegulatorInit(&yawPID,
    		robot->stabConstants[STAB_YAW].pid.pGain, 0,
			robot->stabConstants[STAB_YAW].pid.iGain,
			robot->stabConstants[STAB_YAW].pid.iMax,
			robot->stabConstants[STAB_YAW].pid.iMin);

    PIDRegulatorInit(&depthPID,
        	robot->stabConstants[STAB_DEPTH].pid.pGain, 0,
    		robot->stabConstants[STAB_DEPTH].pid.iGain,
    		robot->stabConstants[STAB_DEPTH].pid.iMax,
    		robot->stabConstants[STAB_DEPTH].pid.iMin);

    Q100.stabConstants[STAB_ROLL].enable = false;
	Q100.stabConstants[STAB_ROLL].pJoyUnitCast = 1;
    Q100.stabConstants[STAB_ROLL].pSpeedDyn = 1;
    Q100.stabConstants[STAB_ROLL].pErrGain = 1;
    Q100.stabConstants[STAB_ROLL].aFilter[SPEED_FILTER].T = 0;
	Q100.stabConstants[STAB_ROLL].aFilter[SPEED_FILTER].K = 1;
    Q100.stabConstants[STAB_ROLL].aFilter[POS_FILTER].T = 0;
	Q100.stabConstants[STAB_ROLL].aFilter[POS_FILTER].K = 1;
    Q100.stabConstants[STAB_ROLL].pid.pGain = 1;
    Q100.stabConstants[STAB_ROLL].pid.iGain = 1;
    Q100.stabConstants[STAB_ROLL].pid.iMax = 500;
    Q100.stabConstants[STAB_ROLL].pid.iMin = -500;

    Q100.stabState[STAB_ROLL].inputSignal = &Q100.f_joySpeed.roll;
    Q100.stabState[STAB_ROLL].speedSignal = &Q100.f_sensors.rollSpeed;
    Q100.stabState[STAB_ROLL].posSignal = &Q100.f_sensors.roll;
    Q100.stabState[STAB_ROLL].oldSpeed = 0;
    Q100.stabState[STAB_ROLL].oldPos = 0;

    Q100.stabState[STAB_ROLL].joyUnitCasted = 0;
    Q100.stabState[STAB_ROLL].joy_iValue = 0;
    Q100.stabState[STAB_ROLL].posError = 0;
    Q100.stabState[STAB_ROLL].speedError = 0;
    Q100.stabState[STAB_ROLL].dynSummator = 0;
    Q100.stabState[STAB_ROLL].pidValue = 0;
    Q100.stabState[STAB_ROLL].posErrorAmp = 0;
    Q100.stabState[STAB_ROLL].speedFiltered = 0;
    Q100.stabState[STAB_ROLL].posFiltered = 0;
    Q100.stabState[STAB_ROLL].LastTick = 0; // TODO u need to write start function, lasttick AND OLDPOS needs to be updated before starting!
    /////////////////////////////////////////////////////////////
    Q100.stabConstants[STAB_PITCH].enable = false;
    Q100.stabConstants[STAB_PITCH].pJoyUnitCast = 1;
    Q100.stabConstants[STAB_PITCH].pSpeedDyn = 1;
    Q100.stabConstants[STAB_PITCH].pErrGain = 1;
    Q100.stabConstants[STAB_PITCH].aFilter[SPEED_FILTER].T = 0;
    Q100.stabConstants[STAB_PITCH].aFilter[SPEED_FILTER].K = 1;
    Q100.stabConstants[STAB_PITCH].aFilter[POS_FILTER].T = 0;
    Q100.stabConstants[STAB_PITCH].aFilter[POS_FILTER].K = 1;
    Q100.stabConstants[STAB_PITCH].pid.pGain = 1;
    Q100.stabConstants[STAB_PITCH].pid.iGain = 1;
    Q100.stabConstants[STAB_PITCH].pid.iMax = 500;
    Q100.stabConstants[STAB_PITCH].pid.iMin = -500;

    Q100.stabState[STAB_PITCH].inputSignal = &Q100.f_joySpeed.roll;
    Q100.stabState[STAB_PITCH].speedSignal = &Q100.f_sensors.rollSpeed;
    Q100.stabState[STAB_PITCH].posSignal = &Q100.f_sensors.roll;
    Q100.stabState[STAB_PITCH].oldSpeed = 0;
    Q100.stabState[STAB_PITCH].oldPos = 0;

    Q100.stabState[STAB_PITCH].joyUnitCasted = 0;
    Q100.stabState[STAB_PITCH].joy_iValue = 0;
    Q100.stabState[STAB_PITCH].posError = 0;
    Q100.stabState[STAB_PITCH].speedError = 0;
    Q100.stabState[STAB_PITCH].dynSummator = 0;
    Q100.stabState[STAB_PITCH].pidValue = 0;
    Q100.stabState[STAB_PITCH].posErrorAmp = 0;
    Q100.stabState[STAB_PITCH].speedFiltered = 0;
    Q100.stabState[STAB_PITCH].posFiltered = 0;
    Q100.stabState[STAB_PITCH].LastTick = 0;
    /////////////////////////////////////////////////////////////
    Q100.stabConstants[STAB_YAW].enable = false;
    Q100.stabConstants[STAB_YAW].pJoyUnitCast = 1;
    Q100.stabConstants[STAB_YAW].pSpeedDyn = 1;
    Q100.stabConstants[STAB_YAW].pErrGain = 1;
    Q100.stabConstants[STAB_YAW].aFilter[SPEED_FILTER].T = 0;
    Q100.stabConstants[STAB_YAW].aFilter[SPEED_FILTER].K = 1;
    Q100.stabConstants[STAB_YAW].aFilter[POS_FILTER].T = 0;
    Q100.stabConstants[STAB_YAW].aFilter[POS_FILTER].K = 1;
    Q100.stabConstants[STAB_YAW].pid.pGain = 1;
    Q100.stabConstants[STAB_YAW].pid.iGain = 1;
    Q100.stabConstants[STAB_YAW].pid.iMax = 500;
    Q100.stabConstants[STAB_YAW].pid.iMin = -500;

    Q100.stabState[STAB_YAW].inputSignal = &Q100.f_joySpeed.roll;
    Q100.stabState[STAB_YAW].speedSignal = &Q100.f_sensors.rollSpeed;
    Q100.stabState[STAB_YAW].posSignal = &Q100.f_sensors.roll;
    Q100.stabState[STAB_YAW].oldSpeed = 0;
    Q100.stabState[STAB_YAW].oldPos = 0;

    Q100.stabState[STAB_YAW].joyUnitCasted = 0;
    Q100.stabState[STAB_YAW].joy_iValue = 0;
    Q100.stabState[STAB_YAW].posError = 0;
    Q100.stabState[STAB_YAW].speedError = 0;
    Q100.stabState[STAB_YAW].dynSummator = 0;
    Q100.stabState[STAB_YAW].pidValue = 0;
    Q100.stabState[STAB_YAW].posErrorAmp = 0;
    Q100.stabState[STAB_YAW].speedFiltered = 0;
    Q100.stabState[STAB_YAW].posFiltered = 0;
    Q100.stabState[STAB_YAW].LastTick = 0;
    /////////////////////////////////////////////////////////////
    Q100.stabConstants[STAB_DEPTH].enable = false;
    Q100.stabConstants[STAB_DEPTH].pJoyUnitCast = 1;
    Q100.stabConstants[STAB_DEPTH].pSpeedDyn = 1;
    Q100.stabConstants[STAB_DEPTH].pErrGain = 1;
    Q100.stabConstants[STAB_DEPTH].aFilter[SPEED_FILTER].T = 0;
    Q100.stabConstants[STAB_DEPTH].aFilter[SPEED_FILTER].K = 1;
    Q100.stabConstants[STAB_DEPTH].aFilter[POS_FILTER].T = 0;
    Q100.stabConstants[STAB_DEPTH].aFilter[POS_FILTER].K = 1;
    Q100.stabConstants[STAB_DEPTH].pid.pGain = 1;
    Q100.stabConstants[STAB_DEPTH].pid.iGain = 1;
    Q100.stabConstants[STAB_DEPTH].pid.iMax = 500;
    Q100.stabConstants[STAB_DEPTH].pid.iMin = -500;

    Q100.stabState[STAB_DEPTH].inputSignal = &Q100.f_joySpeed.roll;
    Q100.stabState[STAB_DEPTH].speedSignal = &Q100.f_sensors.rollSpeed;
    Q100.stabState[STAB_DEPTH].posSignal = &Q100.f_sensors.roll;
    Q100.stabState[STAB_DEPTH].oldSpeed = 0;
    Q100.stabState[STAB_DEPTH].oldPos = 0;

    Q100.stabState[STAB_DEPTH].joyUnitCasted = 0;
    Q100.stabState[STAB_DEPTH].joy_iValue = 0;
    Q100.stabState[STAB_DEPTH].posError = 0;
    Q100.stabState[STAB_DEPTH].speedError = 0;
    Q100.stabState[STAB_DEPTH].dynSummator = 0;
    Q100.stabState[STAB_DEPTH].pidValue = 0;
    Q100.stabState[STAB_DEPTH].posErrorAmp = 0;
    Q100.stabState[STAB_DEPTH].speedFiltered = 0;
    Q100.stabState[STAB_DEPTH].posFiltered = 0;
    Q100.stabState[STAB_DEPTH].LastTick = 0;
}

void stabilizationUpdate(uint8_t contour)
{
	struct RobotStabilizationConstants *constants = &Q100.stabConstants[contour];
	struct RobotStabilizationState *state = &Q100.stabState[contour];
	float diffTime = fromTickToMs(xTaskGetTickCount() - state->LastTick) / 1000.0f;
	state->LastTick = xTaskGetTickCount();

	// Input signal unit cast
	state->joyUnitCasted = constants->pJoyUnitCast * *state->inputSignal;

    // Casted input signal integration
    state->joy_iValue += state->joyUnitCasted * diffTime;

    // Position feedback filtering
    struct AperiodicFilter *filter = &constants->aFilter[POS_FILTER];
    state->posFiltered = state->posFiltered*exp(-diffTime/filter->T) + state->oldPos*filter->K*(1-exp(-diffTime/filter->T));
    state->oldPos = *state->posSignal;

    // Position feedback summator
    state->posError = state->joy_iValue - state->posFiltered;

    // Feedback amplifiers
    state->posErrorAmp = state->posError * constants->pErrGain;

    // PID regulation
    state->pidValue = update(&rollPID, state->posErrorAmp, fromTickToMs(xTaskGetTickCount() - rollPID.lastUpdateTick));

    // Dynamic summator
    state->dynSummator = state->pidValue + *state->inputSignal * constants->pSpeedDyn;

    // Speed feedback filtering
    filter = &constants->aFilter[SPEED_FILTER];
    state->speedFiltered = state->speedFiltered*exp(-diffTime/filter->T) + state->oldSpeed*filter->K*(1-exp(-diffTime/filter->T));
    state->oldSpeed = *state->speedSignal;

    // Speed feedback
    state->speedError = state->dynSummator - state->speedFiltered;
}
