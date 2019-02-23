#include "stabilization.h"

#include "FreeRTOSTick.h"
#include "math.h"
#include "robot.h"

void stabilizationInit()
{
	for(uint8_t i=0; i<STABILIZATION_AMOUNT; i++) {
		rStabConstants[i].enable = false;

		rStabState[i].posDerivative = 0;
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
		rStabState[i].oldPosFiltered = 0;
		rStabState[i].LastTick = 0;

		if(!rState.flash) {
			rStabConstants[i].pJoyUnitCast = 1;
			rStabConstants[i].pSpeedDyn = 1;
			rStabConstants[i].pErrGain = 1;
			rStabConstants[i].aFilter[SPEED_FILTER].T = 0;
			rStabConstants[i].aFilter[SPEED_FILTER].K = 1;
			rStabConstants[i].aFilter[POS_FILTER].T = 0;
			rStabConstants[i].aFilter[POS_FILTER].K = 1;
			rStabConstants[i].pid.pGain = 1;
			rStabConstants[i].pid.iGain = 1;
			rStabConstants[i].pid.iMax = -1000;
			rStabConstants[i].pid.iMin = 1000;
			rStabConstants[i].pThrustersCast = 1;
			rStabConstants[i].pThrustersMax = 5000;
			rStabConstants[i].pThrustersMin = -5000;
		}
	}
	/////////////////////////////////////////////////////////////
    rStabState[STAB_ROLL].inputSignal = &rJoySpeed.roll;
    rStabState[STAB_ROLL].speedSignal = &rSensors.rollSpeed;
    rStabState[STAB_ROLL].posSignal = &rSensors.roll;
    rStabConstants[STAB_ROLL].joyIntegration = true;
    /////////////////////////////////////////////////////////////
    rStabState[STAB_PITCH].inputSignal = &rJoySpeed.pitch;
    rStabState[STAB_PITCH].speedSignal = &rSensors.pitchSpeed;
    rStabState[STAB_PITCH].posSignal = &rSensors.pitch;
    rStabConstants[STAB_PITCH].joyIntegration = true;
    /////////////////////////////////////////////////////////////
    rStabState[STAB_YAW].inputSignal = &rJoySpeed.yaw;
    rStabState[STAB_YAW].speedSignal = &rSensors.yawSpeed;
    rStabState[STAB_YAW].posSignal = &rSensors.yaw;
    rStabConstants[STAB_YAW].joyIntegration = true;
    /////////////////////////////////////////////////////////////
    rStabState[STAB_DEPTH].inputSignal = &rJoySpeed.depth;
    rStabState[STAB_DEPTH].speedSignal = &rStabState[STAB_DEPTH].posDerivative;
    rStabState[STAB_DEPTH].posSignal = &rSensors.pressure;
    rStabConstants[STAB_DEPTH].joyIntegration = false;
}

void stabilizationStart(uint8_t contour)
{
	rStabConstants[contour].enable = true;

	rStabState[contour].oldSpeed = *rStabState[contour].speedSignal;
	rStabState[contour].oldPos = *rStabState[contour].posSignal;
	rStabState[contour].posDerivative = 0;

	rStabState[contour].joyUnitCasted = 0;
	rStabState[contour].joy_iValue = *rStabState[contour].posSignal;
	rStabState[contour].posError = 0;
	rStabState[contour].speedError = 0;
	rStabState[contour].dynSummator = 0;
	rStabState[contour].pidValue = 0;
	rStabState[contour].pid_iValue = 0;
	rStabState[contour].posErrorAmp = 0;
	rStabState[contour].speedFiltered = 0;
	rStabState[contour].posFiltered = 0;
	rStabState[contour].oldPosFiltered = 0;
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
	if(constants->joyIntegration) {
		state->joy_iValue += state->joyUnitCasted * diffTime;
	}
	else {
		state->joy_iValue = state->joyUnitCasted;
	}

    // Position feedback filtering
    struct AperiodicFilter *filter = &constants->aFilter[POS_FILTER];
    if(filter->T != 0) {
    	state->posFiltered = state->posFiltered*exp(-diffTime/filter->T) + state->oldPos*filter->K*(1-exp(-diffTime/filter->T));
    }
    else {
    	state->posFiltered = *state->posSignal*filter->K;
    }
    state->oldPos = *state->posSignal;

    // Position derviative calculation
    state->posDerivative = (state->posFiltered - state->oldPosFiltered) / diffTime;
    state->oldPosFiltered = state->posFiltered;

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
    if(filter->T != 0) {
    	state->speedFiltered = state->speedFiltered*exp(-diffTime/filter->T) + state->oldSpeed*filter->K*(1-exp(-diffTime/filter->T));
    }
    else {
    	state->speedFiltered = *state->speedSignal*filter->K;
    }
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

