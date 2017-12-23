#include "stabilization.h"

struct PIDRegulator rollPID;
struct PIDRegulator pitchPID;
struct PIDRegulator yawPID;

void stabilizationInit(struct Robot *robot)
{
	PIDRegulatorInit(&rollPID,
			robot->rollStabilization.pid_pGain, 0,
			robot->rollStabilization.pid_iGain,
			robot->rollStabilization.pid_iMax, 
			robot->rollStabilization.pid_iMin);
	
	PIDRegulatorInit(&pitchPID,
			robot->pitchStabilization.pid_pGain, 0,
			robot->pitchStabilization.pid_iGain,
			robot->pitchStabilization.pid_iMax, 
			robot->pitchStabilization.pid_iMin);
	
	PIDRegulatorInit(&yawPID,
			robot->yawStabilization.pid_pGain, 0,
			robot->yawStabilization.pid_iGain,
			robot->yawStabilization.pid_iMax, 
			robot->yawStabilization.pid_iMin);
}

float stabilizeRoll(struct Robot *robot)
{
	float newValue = 0;
	
	// Integration
	robot->rollStabilization.aiValue += robot->j_movement.roll * fromTickToMs(xTaskGetTickCount() - robot->rollStabilization.aiLastTick) / 1000.0f;
	robot->rollStabilization.aiLastTick = xTaskGetTickCount();
	
	// Position feedback
	float rollError = robot->rollStabilization.aiValue - robot->sensors.roll * robot->rollStabilization.positionFeedbackCoef;
	
	// PID regulation
	float pidValue = update(&rollPID, rollError, fromTickToMs(xTaskGetTickCount() - rollPID.lastUpdateTick));
	
	// Speed feedback
	newValue = pidValue - robot->sensors.rollSpeed * robot->rollStabilization.speedFeedbackCoef;
	
	return newValue;
}


float stabilizePitch(struct Robot *robot)
{
	float newValue = 0;
	
	// Integration
	robot->pitchStabilization.aiValue += robot->j_movement.pitch * fromTickToMs(xTaskGetTickCount() - robot->pitchStabilization.aiLastTick) / 1000.0f;
	robot->pitchStabilization.aiLastTick = xTaskGetTickCount();
	
	// Position feedback
	float pitchError = robot->pitchStabilization.aiValue - robot->sensors.pitch * robot->pitchStabilization.positionFeedbackCoef;
	
	// PID regulation
	float pidValue = update(&pitchPID, pitchError, fromTickToMs(xTaskGetTickCount() - pitchPID.lastUpdateTick));
	
	// speed regulation
	newValue = pidValue - robot->sensors.pitchSpeed * robot->pitchStabilization.speedFeedbackCoef;
	
	return newValue;
}

float stabilizeYaw(struct Robot *robot)
{
	// Integration
	robot->yawStabilization.aiValue += robot->j_movement.yaw * fromTickToMs(xTaskGetTickCount() - robot->yawStabilization.aiLastTick) / 1000.0f;
	robot->yawStabilization.aiLastTick = xTaskGetTickCount();
	
	// Position feedback
	float yawError = robot->yawStabilization.aiValue - robot->sensors.yaw * robot->yawStabilization.positionFeedbackCoef;
	
	// PID regulation
	float pidValue = update(&pitchPID, yawError, fromTickToMs(xTaskGetTickCount() - pitchPID.lastUpdateTick));
	
	// Speed feedback
	float newValue = pidValue - robot->sensors.yawSpeed * robot->yawStabilization.speedFeedbackCoef;
	
	return newValue;
}
