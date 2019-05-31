#ifndef __TASK_H
#define __TASK_H
#include "os_app_hooks.h"
#include "os.h"
#include "explore_system.h"
#include "explore_systick.h"
#include "stm32f4xx.h"
#include "DronePara.h"
#include "Data_PC.h"
#include "Data_deal.h"
#include "Visiondata_deal.h"
#include "Adc_Battery.h"
#include "Usart3toBluetooth.h"
#include "General_Gpio.h"
#include "Attitude_control.h"
#include "Position_control.h"
#include "Vector3.h"
#include "MPU6000.h"
#include "MPU6500.h"
#include "MahonyAHRS.h"
#include "ThrustMixer.h"
#include "Gyroscope.h"
#include "Accelerometer.h"
#include "Parameter.h"
#include "SafeControl.h"
#include "Board.h"
#include "Navigation.h"
#include "FlightStatus.h"
#include "TofMiniPlus.h"
extern OffsetInfo OffsetData;  										 
extern PID_t OriginalPitch,OriginalRoll,OriginalYaw,OriginalPosX,OriginalPosY,OriginalPosZ,
					OriginalWxRate,OriginalWyRate,OriginalWzRate,OriginalVelX,OriginalVelY,OriginalVelZ;
extern PIDPara PID_ParaInfo;


#endif 

