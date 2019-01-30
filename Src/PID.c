#include "PID.h"

void PIDRegulatorInit(struct PIDRegulator *PID, float pGain, float iGain, float iMax, float iMin, float dGain) {
	PID->iGain = iGain;
	PID->dGain = dGain;
	PID->pGain = pGain;
	PID->dState = 0;
	PID->iState = 0;
	PID->iMin = iMin;
	PID->iMax = iMax;

	PID->pTermLast = 0;
	PID->iTermLast = 0;
	PID->dTermLast = 0;

	PID->lastUpdateTick = xTaskGetTickCount();
}

float update(struct PIDRegulator *PID, float error, float deltaTime_ms)
{

	float pTerm, dTerm, iTerm;
	pTerm = PID->pGain * error; //proportional term
	
	PID->iState += error * deltaTime_ms / 1000.0f;
	
	if (PID->iState > PID->iMax) {
		PID->iState = PID->iMax;
	}
	if (PID->iState < PID->iMin) {
		PID->iState = PID->iMin;
	}

	iTerm = PID->iGain * PID->iState; //integral part

	if (deltaTime_ms == 0) {
		dTerm = 0;
	} else {
		dTerm = PID->dGain * (error - PID->dState) * 1000.0f / deltaTime_ms; //differential part
	}

	PID->dState = error;

	PID->pTermLast = pTerm;
	PID->dTermLast = dTerm;
	PID->iTermLast = iTerm;

	PID->lastUpdateTick = xTaskGetTickCount();

	return pTerm + dTerm + iTerm;
}

void reset(struct PIDRegulator *PID)
{
	PID->iState = 0;
	PID->dState = 0;
}
