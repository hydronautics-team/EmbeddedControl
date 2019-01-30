#ifndef PID_H_
#define PID_H_

#include "FreeRTOSTick.h"

struct PIDRegulator{
	TickType_t lastUpdateTick;
	float iGain;
	float dGain;
	float pGain;
	float dState;
	float iState;
	float iMax;
	float iMin;

	float dTermLast;
	float pTermLast;
	float iTermLast;
};

void PIDRegulatorInit(struct PIDRegulator *PID, float pGain, float iGain, float iMax, float iMin, float dGain);

float update(struct PIDRegulator *PID, float error, float deltaTime);

void reset(struct PIDRegulator *PID);

#endif /* PIDREGULATOR_H_ */
