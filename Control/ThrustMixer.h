#ifndef __THRUSTMIXER_H
#define __THRUSTMIXER_H

#include "Task.h"
#include "DronePara.h"
#include "Board.h"
#include <math.h>

typedef struct{
	float f1;
	float f2;
	float f3;
	float f4;
	float t1;
	float t2;
	float t3;
	float t4;
}ThrustUav;

typedef struct
{
	int M1;
	int M2;
	int M3;
	int M4;
}Throttle;


void ThrustMixer(float arm_length);
void PreTakeOff(uint16_t Time);
void MotorThrust(float f1,float f2,float f3,float f4);
void PWM_OUTPUT(unsigned int Motor1,unsigned int Motor2,
									unsigned int Motor3,unsigned int Motor4);

#endif

