#ifndef STABILIZATION_H
#define STABILIZATION_H

#include <math.h>
#include <stdbool.h>
#include "u_PID.h"
#include "u_robot.h"
#include "u_global.h"


#define STUBILIZATION_BUFF_SIZE 10
#define KREN_INDEX 1
#define DIFF_INDEX 2

extern struct PIDRegulator rollPID;
extern struct PIDRegulator pitchPID;

void stabilizationInit(struct Robot *robot);
float stabilizeRoll(struct Robot *robot);
float stabilizePitch(struct Robot *robot);

#endif
