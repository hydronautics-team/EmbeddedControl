#ifndef STABILIZATION_H
#define STABILIZATION_H

#include <math.h>
#include <stdbool.h>
#include "PID.h"
#include "robot.h"
#include "global.h"


#define STUBILIZATION_BUFF_SIZE 10
#define KREN_INDEX 1
#define DIFF_INDEX 2

extern struct PIDRegulator rollPID;
extern struct PIDRegulator pitchPID;
extern struct PIDRegulator yawPID;
extern struct PIDRegulator depthsPID;

void stabilizationInit(struct Robot *robot);
void computeVmaValues(struct Robot *robot);
void stabilizationUpdate(uint8_t contour);

#endif
