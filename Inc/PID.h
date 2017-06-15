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

TickType_t getLastUpdateTick(struct PIDRegulator *PID);
void setLastUpdateTick(struct PIDRegulator *PID, TickType_t lastUpdateTick);

float getIGain(struct PIDRegulator *PID);
void setIGain(struct PIDRegulator *PID, float gain);

float getPGain(struct PIDRegulator *PID);
void setPGain(struct PIDRegulator *PID, float gain);

float getDGain(struct PIDRegulator *PID);
void setDGain(struct PIDRegulator *PID, float gain);

float update(struct PIDRegulator *PID, float error, float deltaTime);

void reset(struct PIDRegulator *PID);

#endif /* PIDREGULATOR_H_ */
