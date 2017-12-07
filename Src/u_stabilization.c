#include "u_stabilization.h"

struct PIDRegulator rollPID;
struct PIDRegulator pitchPID;

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
}

float stabilizeRoll(struct Robot *robot)
{
	float newValue = 0;
	float rollError = robot->movement.roll - robot->sensors.roll * robot->rollStabilization.positionFeedbackCoef;
	
	// PID regulation
	newValue = update(&rollPID, rollError, fromTickToMs(xTaskGetTickCount() - rollPID.lastUpdateTick));

	// speed regulation
	newValue += robot->sensors.rollSpeed * robot->rollStabilization.speedFeedbackCoef;
	//robot->rollStabilization.speedError = newValue;
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
	//robot->pitchStabilization.speedError = newValue;
	return newValue;
}








// Show these and change
bool PI_config = 0;
float different, kren, different_speed, kren_speed;  // in degrees
float const_time = 1;  
float proportional_koff = 1;
float k1_kren = 15;
float k2_kren = 1.5;
float k1_diff = 15;
float k2_diff = 1.5;

int time_interval = 1;

float start_value_kren = 0.3;
float start_value_diff = 1.7;

// Doesn't show
float error_speed_kren;
float error_speed_diff;

float Integrator(float new_value, int index, long time_interval)
{
  static float old_value[STUBILIZATION_BUFF_SIZE] = {0}; 
  static float return_value[STUBILIZATION_BUFF_SIZE] = {0};
  
  return_value[index] = return_value[index] + ((fmax(new_value,old_value[index]) - fmin(new_value, old_value[index])) / 2) * time_interval;
  old_value[index] = new_value; 
  return return_value[index];
}

//t2 - const_time - show and change 

float PIregulator(float new_value, int index, long time_interval, float proportional, bool PI_config, float const_time)
{
  static float return_value[STUBILIZATION_BUFF_SIZE];
  return_value[index] = new_value * proportional + Integrator(new_value, index, time_interval) * PI_config / const_time;
  return return_value[index];
}

//void parse_feedback_data()  - IMU data
//{
//Different = (float)(word(toVerVmaPacket[TO_VER_FEEDBACK_PITCH_HIGH],toVerVmaPacket[TO_VER_FEEDBACK_PITCH_LOW]))/100.0;
  
//Kren =int(word(toVerVmaPacket[TO_VER_FEEDBACK_ROLL_HIGH],toVerVmaPacket[TO_VER_FEEDBACK_ROLL_LOW]))/100.0;

//DifferentSpeed =int(word(toVerVmaPacket[TO_VER_FEEDBACK_PITCH_SPEED_HIGH],toVerVmaPacket[TO_VER_FEEDBACK_PITCH_SPEED_LOW]))/10000.0;
//DifferentSpeed = DifferentSpeed*180/3.1415;

//KrenSpeed =int(word(toVerVmaPacket[TO_VER_FEEDBACK_ROLL_SPEED_HIGH],toVerVmaPacket[TO_VER_FEEDBACK_ROLL_SPEED_LOW]))/10000.0;
//KrenSpeed = KrenSpeed*180/3.1415;

//}

void tau_kren()
{
  float error = start_value_kren - kren;
  float voltage_before_pi = error * k1_kren;
  float voltage_after_pi = PIregulator(voltage_before_pi, KREN_INDEX, time_interval, proportional_koff, PI_config, const_time);

  error_speed_kren = voltage_after_pi - kren_speed * k2_kren;
  if (error_speed_kren > 127) error_speed_kren = 127;  // Does it need?
  if (error_speed_kren < -128) error_speed_kren = -128;
}

void tau_diff()
{
 //   Joy=((char)toHorVmaPacket[TO_HOR_YAW_SPEED_OFFSET]);  
  float error = start_value_diff - different;
  float voltage_before_pi = error * k1_diff;
  float voltage_after_pi = PIregulator(voltage_before_pi, DIFF_INDEX, time_interval, proportional_koff, PI_config, const_time);

  error_speed_diff = voltage_after_pi - different_speed * k2_diff;
  if (error_speed_diff > 127){
    error_speed_diff = 127;
  }
  if (error_speed_diff < -128){
    error_speed_diff = -128;
  }
}
