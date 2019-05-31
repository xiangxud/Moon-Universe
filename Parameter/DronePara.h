#ifndef __DRONWPARA_H
#define __DRONWPARA_H

#include "stm32f4xx.h"
#include "Vector3.h"
#include <stdbool.h>
#define TX_LEN  160
#define RX_LEN  128
//本地重力加速度
#define Gravity_Acceleration  9.794f
//转动惯量设置  转动惯量 = M*R^2  对于质点而言。这里无法测得转动惯量大小，变为一个可调参数设置
#define Inertia_Wx    0.002f
#define Inertia_Wy    0.002f
#define Inertia_Wz    0.004f

//定义机型
#define Model330
#ifdef Model330
	#define ARM_Length 0.165f
	#define Drone_Mass 1.350f
	#define Drag_Coeff 0.016f
#else
	#define ARM_Length 0.125f
	#define Drone_Mass 1.500f          //实际质量只有一半，但好盈这个电机官方参数表不正确  只能依靠提高质量来弥补悬停油门
#endif

typedef enum{ 
	Drone_Mode_None = 0,
	Drone_Mode_RatePitch = 1, 
  Drone_Mode_RateRoll = 2, 
  Drone_Mode_Pitch = 3,		 	
  Drone_Mode_Roll = 4, 
	Drone_Mode_4Axis = 5,	
}DroneFlightMode_TypeDef;

typedef enum{  
	Drone_Off  = 0x00,
  Drone_On   = 0x01,
}DroneFlightOnOff_TypeDef;

typedef enum{  
	Nothing = 0x00,
	Land  	= 0x01,
}DroneFlightStatus_TypeDef;

typedef enum{
  Report_RESET    = 0x00,   
	Report_SET      = 0x01,
}DroneReportSW_TypeDef;

typedef struct{
	DroneFlightOnOff_TypeDef OnOff;
	DroneFlightMode_TypeDef DroneMode;
	DroneFlightStatus_TypeDef DroneStatus;
	DroneReportSW_TypeDef ReportSW;
}DroneFlightControl;

typedef enum{
	PurePosture = 0,
	FixedHeight,
	FixedPoint,
	StepResponse,
	TrajectoryTracking,
}DroneFlightMethod;

typedef struct{
	Vector3angle_t TargetAngle;
	Vector3f_t TargetW;
	Vector3pos_t TargetPos;
}DroneTargetInfo;

typedef struct{
    float kP;
    float kI;
    float kD;

    float imax;
    float integrator;
    float lastError;
    float lastDerivative;
} PID_t;

typedef struct{
    float Kp;
    float Ki;
    float Kd;
} PID;

typedef struct{
	PID Pitch;
	PID Roll;
	PID Yaw;
	
	PID WxRate;
	PID WyRate;
	PID WzRate;
	
  PID PosX;
	PID PosY;
	PID PosZ;
	
	PID VelX;
	PID VelY;
	PID VelZ;
}PIDPara;

typedef struct{
	float acc_offectx;
	float acc_offecty;
	float acc_offectz;
	float acc_scalex;
	float acc_scaley;
	float acc_scalez;
	float gyro_offectx;
	float gyro_offecty;
	float gyro_offectz;
	float gyro_scalex;
	float gyro_scaley;
	float gyro_scalez;
	Vector3f_t level_scale;
	unsigned int acc_calibra_cnt;
	unsigned int gyro_calibra_cnt;
	bool acc_success;
	bool gyro_success;
	bool level_success;
}OffsetInfo;


typedef struct{
    float Xaxis;
    float Yaxis;
    float Zaxis;
    float Navigation;
}RemoteControl;

typedef struct{
	float worldX;
	float worldY;
	float worldZ;
}Desired_acceleration;

typedef struct{
	unsigned int len;
	unsigned char buf[64];
}Data_Rx;

typedef struct{
	unsigned char buf[32];
}Receive_GroundStation;

typedef struct{
	unsigned char buf[64];
}Receive_VisualOdometry;

typedef struct{
	unsigned char buf[18];
}Receive_TOFData;

typedef union{
	float fvalue;
	unsigned char cv[4];
}float_union;

#endif

