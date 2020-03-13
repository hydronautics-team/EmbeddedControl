#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "robot.h"
#include "messages.h"
#include "global.h"
#include "communication.h"
#include "checksum.h"

void addMarchToSumm(float *velocity);
void addLagToSumm(float *velocity);
void addDepthToSumm(float *velocity);
void addYawToSumm(float *velocity);
void addRollToSumm(float *velocity);
void addPitchToSumm(float *velocity);

uint8_t resizeFloatToUint8(float input);

void thrustersInit()
{
	rThrusters[HRB].address = 1;
	rThrusters[HRF].address = 2;
	rThrusters[HLB].address = 3;
	rThrusters[HLF].address = 4;
	rThrusters[VL].address 	= 5;
	rThrusters[VR].address 	= 6;
	rThrusters[VB].address 	= 7;
	rThrusters[VF].address 	= 8;

	rThrusters[HRB].address = 1;
	rThrusters[HRF].address = 2;
	rThrusters[HLB].inverse = false;
	rThrusters[HLF].inverse = false;
	rThrusters[VL].inverse 	= false;
	rThrusters[VR].inverse 	= false;
	rThrusters[VB].inverse 	= false;
	rThrusters[VF].inverse 	= false;

	for(uint8_t i=0; i<THRUSTERS_NUMBER; i++) {
		rThrusters[i].desiredSpeed = 0;
		rThrusters[i].kForward = 1;
		rThrusters[i].kBackward = 1;
		rThrusters[i].sForward = 127;
		rThrusters[i].sBackward = 127;
	}
}

void resetThrusters()
{
	rJoySpeed.depth = 0;
	rJoySpeed.lag = 0;
	rJoySpeed.march = 0;
	rJoySpeed.pitch = 0;
	rJoySpeed.roll = 0;
	rJoySpeed.yaw = 0;

	rThrusters[HRB].desiredSpeed = 0;
	rThrusters[HRF].desiredSpeed = 0;
	rThrusters[HLB].desiredSpeed = 0;
	rThrusters[HLF].desiredSpeed = 0;
	rThrusters[VL].desiredSpeed = 0;
	rThrusters[VR].desiredSpeed = 0;
	rThrusters[VB].desiredSpeed = 0;
	rThrusters[VF].desiredSpeed = 0;
}

void fillThrustersRequest(uint8_t *buf, uint8_t thruster)
{
    struct thrustersRequest_s res;

    res.AA = 0xAA;
    res.type = 0x01;
    res.address = rThrusters[thruster].address;
    int16_t velocity = rThrusters[thruster].desiredSpeed;

    // Inverting
    if(rThrusters[thruster].inverse) {
    	velocity *= -1;
    }

    // Multiplier constants
    if(velocity > 0) {
    	velocity = (int16_t) ( (float) (velocity) * rThrusters[thruster].kForward);
    }
    else if(velocity < 0) {
    	velocity = (int16_t) ((float) (velocity) * rThrusters[thruster].kBackward);
    }

    // Saturation
    if(velocity > rThrusters[thruster].sForward) {
    	velocity = rThrusters[thruster].sForward;
    }
    else if(velocity < -rThrusters[thruster].sBackward) {
    	velocity = -rThrusters[thruster].sBackward;
    }
    res.velocity = velocity;

    memcpy((void*)buf, (void*)&res, THRUSTERS_REQUEST_LENGTH);
    AddChecksumm8bVma(buf, THRUSTERS_REQUEST_LENGTH);
}

void fillThrustersResponse(uint8_t *buf, uint8_t thruster)
{
	//TODO errors parsing! and what is all this new stuff means
    if(IsChecksumm8bCorrectVma(buf, THRUSTERS_RESPONSE_LENGTH) && buf[0] != 0) {
    	struct thrustersResponse_s res;
    	memcpy((void*)&res, (void*)buf, THRUSTERS_RESPONSE_LENGTH);

        rThrusters[thruster].current = res.current;

        ++uartBus[THRUSTERS_UART].successRxCounter;
    }
    else {
    	++uartBus[THRUSTERS_UART].brokenRxCounter;
    }
}

void formThrustVectors()
{
	float velocity[THRUSTERS_NUMBER];
	for(uint8_t i=0; i<THRUSTERS_NUMBER; i++) {
		velocity[i] = 0;
	}
	// March thrusters1
	addMarchToSumm(velocity);
	// Lag Thrusters
	addYawToSumm(velocity);
	addLagToSumm(velocity);
	// Two vertical thrusters
	addDepthToSumm(velocity);
	addRollToSumm(velocity);
	// One vertical corrective thruster
	addPitchToSumm(velocity);

	for (uint8_t i = 0; i < THRUSTERS_NUMBER; ++i) {
		rThrusters[i].desiredSpeed = resizeFloatToUint8(velocity[i]);
	}
}

void addMarchToSumm(float *velocity)
{
	float value = 0;
	// Choosing source of the signal
	if(rStabConstants[STAB_MARCH].enable) {
		value = rStabState[STAB_MARCH].outputSignal;
	}
	else {
		value = rJoySpeed.march;
	}
	// March contour summ
	velocity[HLB] += value;
	velocity[HLF] += value;
	velocity[HRB] += value;
	velocity[HRF] += value;
	// March summ saturation
	for(uint8_t i=HLB; i<HRF+1; i++) {
		if(velocity[i] > rStabConstants[STAB_MARCH].sOutSummatorMax) {
			velocity[i] = rStabConstants[STAB_MARCH].sOutSummatorMax;
		}
		else if(velocity[i] < rStabConstants[STAB_MARCH].sOutSummatorMin) {
			velocity[i] = rStabConstants[STAB_MARCH].sOutSummatorMin;
		}
	}
}

void addLagToSumm(float *velocity)
{
	float value = 0;
	// Choosing source of the signal
	if(rStabConstants[STAB_LAG].enable) {
		value = rStabState[STAB_LAG].outputSignal;
	}
	else {
		value = rJoySpeed.lag;
	}
	// Lag contour summ
	velocity[HLB] -= value;
	velocity[HLF] += value;
	velocity[HRB] += value;
	velocity[HRF] -= value;
	// Lag summ saturation
	for(uint8_t i=HLB; i<HRF+1; i++) {
		if(velocity[i] > rStabConstants[STAB_LAG].sOutSummatorMax) {
			velocity[i] = rStabConstants[STAB_LAG].sOutSummatorMax;
		}
		else if(velocity[i] < rStabConstants[STAB_LAG].sOutSummatorMin) {
			velocity[i] = rStabConstants[STAB_LAG].sOutSummatorMin;
		}
	}
}

void addDepthToSumm(float *velocity)
{
	float value = 0;
	// Choosing source of the signal
	if(rStabConstants[STAB_DEPTH].enable) {
		value = rStabState[STAB_DEPTH].outputSignal;
	}
	else {
		value = rJoySpeed.depth;
	}
	// Depth contour summ
	velocity[VL] -= value;
	velocity[VR] -= value;
	velocity[VB] -= value;
	velocity[VF] -= value;
	// Depth summ saturation
	for(uint8_t i=VL; i<VF+1; i++) {
		if(velocity[i] > rStabConstants[STAB_DEPTH].sOutSummatorMax) {
			velocity[i] = rStabConstants[STAB_DEPTH].sOutSummatorMax;
		}
		else if(velocity[i] < rStabConstants[STAB_DEPTH].sOutSummatorMin) {
			velocity[i] = rStabConstants[STAB_DEPTH].sOutSummatorMin;
		}
	}
}

void addYawToSumm(float *velocity)
{
	float value = 0;
	// Choosing source of the signal
	if(rStabConstants[STAB_YAW].enable) {
		value = rStabState[STAB_YAW].outputSignal;
	}
	else {
		value = rJoySpeed.yaw;
	}
	// Yaw contour summ
	velocity[HLB] += value;
	velocity[HLF] += value;
	velocity[HRB] -= value;
	velocity[HRF] -= value;
	// Yaw summ saturation
	for(uint8_t i=HLB; i<HRF+1; i++) {
		if(velocity[i] > rStabConstants[STAB_YAW].sOutSummatorMax) {
			velocity[i] = rStabConstants[STAB_YAW].sOutSummatorMax;
		}
		else if(velocity[i] < rStabConstants[STAB_YAW].sOutSummatorMin) {
			velocity[i] = rStabConstants[STAB_YAW].sOutSummatorMin;
		}
	}
}

void addRollToSumm(float *velocity)
{
	float value = 0;
	// Choosing source of the signal
	if(rStabConstants[STAB_ROLL].enable) {
		value = rStabState[STAB_ROLL].outputSignal;
	}
	else {
		value = rJoySpeed.roll;
	}
	// Yaw contour summ
	velocity[VL] += value;
	velocity[VR] -= value;
	// Yaw summ saturation
	for(uint8_t i=VL; i<VR+1; i++) {
		if(velocity[i] > rStabConstants[STAB_ROLL].sOutSummatorMax) {
			velocity[i] = rStabConstants[STAB_ROLL].sOutSummatorMax;
		}
		else if(velocity[i] < rStabConstants[STAB_ROLL].sOutSummatorMin) {
			velocity[i] = rStabConstants[STAB_ROLL].sOutSummatorMin;
		}
	}
}

void addPitchToSumm(float *velocity)
{
	float value = 0;
	// Choosing source of the signal
	if(rStabConstants[STAB_PITCH].enable) {
		value = rStabState[STAB_PITCH].outputSignal;
	}
	else {
		value = rJoySpeed.pitch;
	}
	// Pitch contour summ
	velocity[VB] += value;
	velocity[VF] -= value;
	// Pitch summ saturation
	for(uint8_t i=VB; i<VF+1; i++) {
		if(velocity[i] > rStabConstants[STAB_PITCH].sOutSummatorMax) {
			velocity[i] = rStabConstants[STAB_PITCH].sOutSummatorMax;
		}
		else if(velocity[i] < rStabConstants[STAB_PITCH].sOutSummatorMin) {
			velocity[i] = rStabConstants[STAB_PITCH].sOutSummatorMin;
		}
	}
}

uint8_t resizeFloatToUint8(float input)
{
	int32_t cast = (int32_t) input;
	cast = cast / 0xFF;
	if (cast > 127) {
		cast = 127;
	}
	else if(cast < -127) {
		cast = -127;
	}
	return (int8_t) cast;
}
