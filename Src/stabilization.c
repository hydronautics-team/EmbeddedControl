#include "stabilization.h"

#include "FreeRTOSTick.h"
#include "math.h"
#include "robot.h"

float depthSpeed = 0;
float oldDepthSpeed = 0;

void stabilizationInit()
{
	for(uint8_t i=0; i<STABILIZATION_AMOUNT; i++) {
		rStabState[i].oldSpeed = 0;
		rStabState[i].oldPos = 0;

		rStabState[i].joyUnitCasted = 0;
		rStabState[i].joy_iValue = 0;
		rStabState[i].posError = 0;
		rStabState[i].speedError = 0;
		rStabState[i].dynSummator = 0;
		rStabState[i].pidValue = 0;
		rStabState[i].pid_iValue = 0;
		rStabState[i].posErrorAmp = 0;
		rStabState[i].speedFiltered = 0;
		rStabState[i].posFiltered = 0;
		rStabState[i].LastTick = 0;
	}

    rStabConstants[STAB_ROLL].enable = false;
	rStabConstants[STAB_ROLL].pJoyUnitCast = 1;
    rStabConstants[STAB_ROLL].pSpeedDyn = 1;
    rStabConstants[STAB_ROLL].pErrGain = 1;
    rStabConstants[STAB_ROLL].aFilter[SPEED_FILTER].T = 0;
    rStabConstants[STAB_ROLL].aFilter[SPEED_FILTER].K = 1;
    rStabConstants[STAB_ROLL].aFilter[POS_FILTER].T = 0;
	rStabConstants[STAB_ROLL].aFilter[POS_FILTER].K = 1;
    rStabConstants[STAB_ROLL].pid.pGain = 1;
    rStabConstants[STAB_ROLL].pid.iGain = 1;
    rStabConstants[STAB_ROLL].pid.iMax = 500;
    rStabConstants[STAB_ROLL].pid.iMin = -500;
    rStabConstants[STAB_ROLL].pThrustersCast = 1;
    rStabConstants[STAB_ROLL].pThrustersMax = 127;
    rStabConstants[STAB_ROLL].pThrustersMin = -127;

    rStabState[STAB_ROLL].inputSignal = &rJoySpeed.roll;
    rStabState[STAB_ROLL].speedSignal = &rSensors.rollSpeed;
    rStabState[STAB_ROLL].posSignal = &rSensors.roll;
    /////////////////////////////////////////////////////////////
    rStabConstants[STAB_PITCH].enable = false;
	rStabConstants[STAB_PITCH].pJoyUnitCast = 1;
    rStabConstants[STAB_PITCH].pSpeedDyn = 1;
    rStabConstants[STAB_PITCH].pErrGain = 1;
    rStabConstants[STAB_PITCH].aFilter[SPEED_FILTER].T = 0;
    rStabConstants[STAB_PITCH].aFilter[SPEED_FILTER].K = 1;
    rStabConstants[STAB_PITCH].aFilter[POS_FILTER].T = 0;
	rStabConstants[STAB_PITCH].aFilter[POS_FILTER].K = 1;
    rStabConstants[STAB_PITCH].pid.pGain = 1;
    rStabConstants[STAB_PITCH].pid.iGain = 1;
    rStabConstants[STAB_PITCH].pid.iMax = 500;
    rStabConstants[STAB_PITCH].pid.iMin = -500;
    rStabConstants[STAB_PITCH].pThrustersCast = 1;
    rStabConstants[STAB_PITCH].pThrustersMax = 127;
    rStabConstants[STAB_PITCH].pThrustersMin = -127;

    rStabState[STAB_PITCH].inputSignal = &rJoySpeed.pitch;
    rStabState[STAB_PITCH].speedSignal = &rSensors.pitchSpeed;
    rStabState[STAB_PITCH].posSignal = &rSensors.pitch;
    /////////////////////////////////////////////////////////////
    rStabConstants[STAB_YAW].enable = false;
	rStabConstants[STAB_YAW].pJoyUnitCast = 1;
    rStabConstants[STAB_YAW].pSpeedDyn = 1;
    rStabConstants[STAB_YAW].pErrGain = 1;
    rStabConstants[STAB_YAW].aFilter[SPEED_FILTER].T = 0;
    rStabConstants[STAB_YAW].aFilter[SPEED_FILTER].K = 1;
    rStabConstants[STAB_YAW].aFilter[POS_FILTER].T = 0;
	rStabConstants[STAB_YAW].aFilter[POS_FILTER].K = 1;
    rStabConstants[STAB_YAW].pid.pGain = 1;
    rStabConstants[STAB_YAW].pid.iGain = 1;
    rStabConstants[STAB_YAW].pid.iMax = 500;
    rStabConstants[STAB_YAW].pid.iMin = -500;
    rStabConstants[STAB_YAW].pThrustersCast = 1;
    rStabConstants[STAB_YAW].pThrustersMax = 127;
    rStabConstants[STAB_YAW].pThrustersMin = -127;

    rStabState[STAB_YAW].inputSignal = &rJoySpeed.yaw;
    rStabState[STAB_YAW].speedSignal = &rSensors.yawSpeed;
    rStabState[STAB_YAW].posSignal = &rSensors.yaw;
    /////////////////////////////////////////////////////////////
    rStabConstants[STAB_DEPTH].enable = false;
	rStabConstants[STAB_DEPTH].pJoyUnitCast = 1;
    rStabConstants[STAB_DEPTH].pSpeedDyn = 1;
    rStabConstants[STAB_DEPTH].pErrGain = 1;
    rStabConstants[STAB_DEPTH].aFilter[SPEED_FILTER].T = 0;
    rStabConstants[STAB_DEPTH].aFilter[SPEED_FILTER].K = 1;
    rStabConstants[STAB_DEPTH].aFilter[POS_FILTER].T = 0;
	rStabConstants[STAB_DEPTH].aFilter[POS_FILTER].K = 1;
    rStabConstants[STAB_DEPTH].pid.pGain = 1;
    rStabConstants[STAB_DEPTH].pid.iGain = 1;
    rStabConstants[STAB_DEPTH].pid.iMax = 500;
    rStabConstants[STAB_DEPTH].pid.iMin = -500;
    rStabConstants[STAB_DEPTH].pThrustersCast = 1;
    rStabConstants[STAB_DEPTH].pThrustersMax = 127;
    rStabConstants[STAB_DEPTH].pThrustersMin = -127;

    rStabState[STAB_DEPTH].inputSignal = &rJoySpeed.depth;
    rStabState[STAB_DEPTH].speedSignal = &depthSpeed;
    rStabState[STAB_DEPTH].posSignal = &rSensors.pressure;
}

void stabilizationStart(uint8_t contour)
{
	rStabConstants[contour].enable = true;

	rStabState[contour].oldSpeed = *rStabState[contour].speedSignal;
	rStabState[contour].oldPos = *rStabState[contour].posSignal;

	rStabState[contour].joyUnitCasted = 0;
	rStabState[contour].joy_iValue = *rStabState[contour].inputSignal;
	rStabState[contour].posError = 0;
	rStabState[contour].speedError = 0;
	rStabState[contour].dynSummator = 0;
	rStabState[contour].pidValue = 0;
	rStabState[contour].pid_iValue = 0;
	rStabState[contour].posErrorAmp = 0;
	rStabState[contour].speedFiltered = 0;
	rStabState[contour].posFiltered = 0;
	rStabState[contour].LastTick = xTaskGetTickCount();
}

void stabilizationUpdate(uint8_t contour)
{
	struct robotStabilizationConstants_s *constants = &rStabConstants[contour];
	struct robotStabilizationState_s *state = &rStabState[contour];
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

    // PI integration
    state->pid_iValue += (state->posErrorAmp * diffTime) * constants->pid.iGain;

    // PI integration saturation
    if(state->pid_iValue > constants->pid.iMax) {
    	state->pid_iValue = constants->pid.iMax;
    }
    else if(state->pid_iValue < constants->pid.iMin) {
    	state->pid_iValue = constants->pid.iMin;
    }

    // PI summator
    state->pidValue =  state->pid_iValue + (state->posErrorAmp * constants->pid.pGain);

    // Dynamic summator
    state->dynSummator = state->pidValue + *state->inputSignal * constants->pSpeedDyn;

    // Speed feedback filtering
    filter = &constants->aFilter[SPEED_FILTER];
    if(contour == STAB_DEPTH) {
    	depthSpeed = (depthSpeed - oldDepthSpeed) / diffTime;
    	oldDepthSpeed = depthSpeed;
    }
    state->speedFiltered = state->speedFiltered*exp(-diffTime/filter->T) + state->oldSpeed*filter->K*(1-exp(-diffTime/filter->T));
    state->oldSpeed = *state->speedSignal;

    // Speed feedback
    state->speedError = state->dynSummator - state->speedFiltered;

    // Thrusters unit cast
    state->speedError *= constants->pThrustersCast;
    if(state->speedError > constants->pThrustersMax) {
    	state->speedError = constants->pThrustersMax;
    }
    else if(state->speedError < constants->pThrustersMin) {
    	state->speedError = constants->pThrustersMin;
    }
}

