#include "stabilization.h"

struct PIDRegulator rollPID;
struct PIDRegulator pitchPID;
struct PIDRegulator yawPID;

void stabilizationInit(struct Robot *robot)
{
	PIDRegulatorInit(&rollPID,
			robot->rollStabilization.pGain, 0,
			robot->rollStabilization.iGain,
			robot->rollStabilization.iMax, 
			robot->rollStabilization.iMin);
	
	PIDRegulatorInit(&pitchPID,
			robot->pitchStabilization.pGain, 0,
			robot->pitchStabilization.iGain,
			robot->pitchStabilization.iMax, 
			robot->pitchStabilization.iMin);
	
	PIDRegulatorInit(&yawPID,
			robot->yawStabilization.pGain, 0,
			robot->yawStabilization.iGain,
			robot->yawStabilization.iMax, 
			robot->yawStabilization.iMin);
}

float stabilizeRoll(struct Robot *robot)
{
	float newValue = 0;
	float rollError = robot->movement.roll - robot->sensors.roll * robot->rollStabilization.positionFeedbackCoef;
	
	// PID regulation
	newValue = update(&rollPID, rollError, fromTickToMs(xTaskGetTickCount() - rollPID.lastUpdateTick));

	// speed regulation
	newValue += robot->sensors.rollSpeed * robot->rollStabilization.speedFeedbackCoef;
	
	return newValue;
}


float stabilizePitch(struct Robot *robot)
{
	float newValue = 0;
	float pitchError = robot->movement.pitch - robot->sensors.pitch * robot->pitchStabilization.positionFeedbackCoef;
	
	// PID regulation
	newValue = update(&pitchPID, pitchError, fromTickToMs(xTaskGetTickCount() - pitchPID.lastUpdateTick));
	
	// speed regulation
	newValue += robot->sensors.pitchSpeed * robot->pitchStabilization.speedFeedbackCoef;
	
	return newValue;
}

float stabilizeYaw(struct Robot *robot)
{
	float newValue = 0;
	float yawError = robot->movement.yaw - robot->sensors.yaw * robot->yawStabilization.positionFeedbackCoef;
	
	// PID regulation
	newValue = update(&pitchPID, yawError, fromTickToMs(xTaskGetTickCount() - pitchPID.lastUpdateTick));
	
	// speed regulation
	newValue += robot->sensors.yawSpeed * robot->yawStabilization.speedFeedbackCoef;
	
	return newValue;
}
