#ifndef __MAHONYAHRS_H
#define __MAHONYAHRS_H
#include "stm32f4xx.h"
#include "Vector3.h"
#include "Task.h"
#include <math.h>
//----------------------------------------------------------------------------------------------------
// Variable declaration

typedef struct {
	Vector3angle_t CopterAngle;
	Vector3f_t AHRSAngle;
	Vector4q_t QuaternionValue;
	Vector3f_t EarthAcc;
} MahonyAHRS_t;

//---------------------------------------------------------------------------------------------------
// Function declarations

void MahonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);
Vector3angle_t GetCopterAngle(void);
Vector4q_t GetCopterQuaternion(void);
Vector3f_t EarthAccGetData(void);
#endif

