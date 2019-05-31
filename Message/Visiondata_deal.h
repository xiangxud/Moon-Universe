#ifndef __VISIONDATA_DEAL_H
#define __VISIONDATA_DEAL_H

#include "stm32f4xx.h"
#include "DronePara.h"
#include "Task.h"

//DeepLearning Image Size
#define ImageCenterX 		480.0f
#define ImageCenterY    240.0f
#define ImageX          960.0f
#define ImageY          480.0f

typedef struct {
	float CurrentTime;
	uint64_t LastTime;
	Vector3f_t VIOPosValue;
	Vector3f_t VIOPosTransValue;
	Vector3f_t VIOPosTransValueLpf;
	Vector3f_t VIOVelValue;
	Vector3angle_t VIOAttitude;
}VisualOdometryValue;

typedef struct {
	float CurrentTime;
	uint64_t LastTime;
	Vector3f_t FormationPosValue;
	Vector3f_t Bbox;
	Vector3f_t Size;
	float RefYawRate;
}ReferenceRouteValue;

void Vision_DataDeal(Receive_VisualOdometry rx);
Vector3f_t GetVisualOdometryPos(void);
Vector3f_t GetVisualOdometryVel(void);
Vector3angle_t GetVisualOdometryAngle(void);
Vector3f_t GetWayPointRefPos(void);
Vector3f_t GetWayPointRefVel(void);
Vector3f_t GetWayPointRefAcc(void);
float GetFPSVisualOdometry(void);

#endif


