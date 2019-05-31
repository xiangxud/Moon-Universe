/******************* (C) COPYRIGHT 2015-20~~ HappyMoon **************************
 * @文件     Task.c
 * @说明     任务编写函数
 * @作者     YuyingJin
 * @网站     https://yuyingjin0111.github.io/
 * @日期     2018 ~
*********************************************************************************/
#include "Task.h"
/* 全局变量初始化 */ 
PID_t OriginalPitch,OriginalRoll,OriginalYaw,OriginalPosX,OriginalPosY,OriginalPosZ,
					OriginalWxRate,OriginalWyRate,OriginalWzRate,OriginalVelX,OriginalVelY,OriginalVelZ;
PIDPara PID_ParaInfo;
OffsetInfo OffsetData;
/**
 * @Description 传感器数据读取 1khz读取
 */
void IMUSensorRead_task(void *p_arg){
	OS_ERR err;
	p_arg = p_arg;
	// 申请内存
	Vector3f_t *accRawData = OSMemGet(&memoryInfo[ACC_SENSOR_RAW],&err);
	Vector3f_t *gyroRawData = OSMemGet(&memoryInfo[GYRO_SENSOR_RAW],&err);
	while(1){
#ifdef SpeedyBeeF4
		//读取加速计 陀螺仪 传感器
		MPU6000_ReadAccGyro(accRawData ,gyroRawData);
#else
		//读取加速计 陀螺仪 传感器
		MPU6500_ReadAccGyro(accRawData ,gyroRawData);
#endif
		//更新消息队列
		OSQPost(&messageQueue[ACC_SENSOR_READ],accRawData,sizeof(Vector3f_t),OS_OPT_POST_FIFO,&err);
		OSQPost(&messageQueue[GYRO_SENSOR_READ],gyroRawData,sizeof(Vector3f_t),OS_OPT_POST_FIFO,&err);
		//睡眠一个时钟节拍1ms
		OSTimeDlyHMSM(0,0,0,1,OS_OPT_TIME_HMSM_STRICT,&err);
	}
}

/**
 * @Description 传感器数据预处理
 */
void IMUSensorPreDeal_task(void *p_arg){
	OS_ERR err;
	p_arg = p_arg;
	void   *p_msg;
	CPU_SR_ALLOC();
	OS_MSG_SIZE  msg_size;
	CPU_TS       ts;
	Vector3f_t accRawData,gyroRawData;
	//申请内存
	Vector3f_t *accCalibData = OSMemGet(&memoryInfo[ACC_SENSOR_PRETREAT],&err);
	Vector3f_t *gyroCalibData = OSMemGet(&memoryInfo[GYRO_SENSOR_PRETREAT],&err);
	Vector3f_t *gyroLpfData = OSMemGet(&memoryInfo[GYRO_SENSOR_LPF],&err);
	//默认不进行传感器校准
	OffsetData.acc_success = false;
	OffsetData.gyro_success = false;	
	OffsetData.level_success = false;
	//进入临界区
	OS_CRITICAL_ENTER();
	//陀螺仪预处理初始化
	GyroPreTreatInit();
	//加速度预处理初始化
	AccPreTreatInit();	
	//离开临界区 	
	OS_CRITICAL_EXIT();	
	while(1){
		//消息队列信息提取
		p_msg = OSQPend(&messageQueue[ACC_SENSOR_READ],0,OS_OPT_PEND_BLOCKING,&msg_size,&ts,&err);
		if(err == OS_ERR_NONE){
			accRawData = *((Vector3f_t *)p_msg);
		}
		p_msg = OSQPend(&messageQueue[GYRO_SENSOR_READ],0,OS_OPT_PEND_BLOCKING,&msg_size,&ts,&err);
		if(err == OS_ERR_NONE){
			gyroRawData = *((Vector3f_t *)p_msg);
		}
		//加速计校准
		AccCalibration(accRawData);
		//陀螺仪校准
		GyroCalibration(gyroRawData);
		//IMU安装误差校准
		ImuLevelCalibration();
		//加速计数据处理
		AccDataPreTreat(accRawData, accCalibData,OffsetData.level_scale);
		//陀螺仪数据处理
		GyroDataPreTreat(gyroRawData, gyroCalibData, gyroLpfData,OffsetData.level_scale);
		//更新下一级消息队列
		OSQPost(&messageQueue[ACC_DATA_PRETREAT],accCalibData,sizeof(Vector3f_t),OS_OPT_POST_FIFO,&err);
		OSQPost(&messageQueue[GYRO_DATA_PRETREAT],gyroCalibData,sizeof(Vector3f_t),OS_OPT_POST_FIFO,&err);
		OSQPost(&messageQueue[GYRO_FOR_CONTROL],gyroLpfData,sizeof(Vector3f_t),OS_OPT_POST_FIFO,&err);	
	}
}

/**
 * @Description 全局导航任务（姿态解算，速度融合，位置融合）
 */
void Navigation_task(void *p_arg){
	OS_ERR err;
	p_arg = p_arg;
	void   *p_msg;
	CPU_SR_ALLOC();
	OS_MSG_SIZE  msg_size;
	CPU_TS       ts;
	Vector3f_t accCalibData,gyroCalibData;
	OS_CRITICAL_ENTER();
	//导航参数初始化
	NavigationInit();
	//离开临界区 	
	OS_CRITICAL_EXIT();		
	while(1){
		//消息队列信息提取
		p_msg = OSQPend(&messageQueue[ACC_DATA_PRETREAT],0,OS_OPT_PEND_BLOCKING,&msg_size,&ts,&err);
		if(err == OS_ERR_NONE){
			accCalibData = *((Vector3f_t *)p_msg);
		}
		p_msg = OSQPend(&messageQueue[GYRO_DATA_PRETREAT],0,OS_OPT_PEND_BLOCKING,&msg_size,&ts,&err);
		if(err == OS_ERR_NONE){
			gyroCalibData = *((Vector3f_t *)p_msg);
		}
		//飞行姿态估计
		MahonyAHRSupdateIMU(gyroCalibData.x,gyroCalibData.y,gyroCalibData.z,
															accCalibData.x,accCalibData.y,accCalibData.z);
		//飞行速度估计
		VelocityEstimate();
		//飞行位置估计
		PositionEstimate();
	}
}

/**
 * @Description 飞行控制任务
 */
void FlightControl_task(void *p_arg){
	OS_ERR err;
	p_arg = p_arg;
	void   *p_msg;
	OS_MSG_SIZE  msg_size;
	CPU_TS       ts;
	Vector3f_t Estimate_Gyro;
	static uint64_t count = 0;
	SetCopterFlightMethod();
	SetStepSignalValue(0.0f,0.0f,1.0f);
	while(1){
		//消息队列信息提取
		p_msg = OSQPend(&messageQueue[GYRO_FOR_CONTROL],0,OS_OPT_PEND_BLOCKING,&msg_size,&ts,&err);
		if(err == OS_ERR_NONE){
			Estimate_Gyro = *((Vector3f_t *)p_msg);
		}
		//起飞检测
		if(GetCopterStatus() == Drone_Off){
			//置位所有控制参数
			ResetControlPara();
			//电机停止
			PWM_OUTPUT(0,0,0,0);
			count = 0;
		}
		else{
			if(count < 3000 && GetCopterTest() == Drone_Mode_4Axis){
				PreTakeOff(count);
			}
			else{
				//100hz
				if(count % 10 == 0){
					//安全保护
					SafeControl();
					//位置外环控制
					Position_OuterController();
				}
				//200hz
				if(count % 5 == 0){
					//位置内环控制
					Position_InnerController();
					//飞行角度控制
					Attitude_OuterController();
					//飞行高度控制
					Altitude_Controller();
				}
				//飞行角速率控制
				Attitude_InnerController(Estimate_Gyro);
				//推力融合
				ThrustMixer(ARM_Length);
			}
			count ++;
		}
	}
}

/**
	* @Description 视觉惯性里程计任务
 */
void VisualOdometry_task(void *p_arg){
	OS_ERR err;
	CPU_SR_ALLOC();
	p_arg = p_arg;
	void   *p_msg;
	OS_MSG_SIZE  msg_size;
	CPU_TS       ts;
	Receive_VisualOdometry  VisualOdometry;
	while(1){
		//消息队列信息提取
		p_msg = OSQPend(&messageQueue[VISUAL_ODOMETRY],0,OS_OPT_PEND_BLOCKING,&msg_size,&ts,&err);
		if(err == OS_ERR_NONE){
			VisualOdometry = *((Receive_VisualOdometry *)p_msg);
		}
		// 进入临界区
		OS_CRITICAL_ENTER();										                 
		Vision_DataDeal(VisualOdometry);
		// 退出临界区
		OS_CRITICAL_EXIT();											                 
	}
}

/**
	* @Description TOF激光数据读取
 */
void TOFMiniPlus_task(void *p_arg){
	OS_ERR err;
	CPU_SR_ALLOC();
	p_arg = p_arg;
	void   *p_msg;
	OS_MSG_SIZE  msg_size;
	CPU_TS       ts;
	Receive_TOFData  TOFData;
	while(1){
		//消息队列信息提取
		p_msg = OSQPend(&messageQueue[TIMEOFFLY_DATA],0,OS_OPT_PEND_BLOCKING,&msg_size,&ts,&err);
		if(err == OS_ERR_NONE){
			TOFData = *((Receive_TOFData *)p_msg);
		}
		// 进入临界区
		OS_CRITICAL_ENTER();
		//TOF激光传感器更新
		TOF_DataDeal(TOFData);
		// 退出临界区
		OS_CRITICAL_EXIT();	
	}
}
/**
 * @Description 地面站处理任务
 */
void GroundStation_task(void *p_arg){
	OS_ERR err;
	CPU_SR_ALLOC();
	p_arg = p_arg;
	void   *p_msg;
	OS_MSG_SIZE  msg_size;
	CPU_TS       ts;
	Receive_GroundStation GroundStation;
	while(1){
		//消息队列信息提取
		p_msg = OSQPend(&messageQueue[GROUND_STATION],0,OS_OPT_PEND_BLOCKING,&msg_size,&ts,&err);
		if(err == OS_ERR_NONE){
			GroundStation = *((Receive_GroundStation *)p_msg);
		}
		// 进入临界区
		OS_CRITICAL_ENTER();
		GroundStationDataDeal(GroundStation);
		// 退出临界区
		OS_CRITICAL_EXIT();	
	}
}

/**
 * @Description 飞行器状态任务 50hz
 */
void FlightStatus_task(void *p_arg){
	OS_ERR err;
	p_arg = p_arg;
	while(1){
		//飞行器放置状态检测
    PlaceStausCheck(GyroLpfGetData());
		//传感器方向检测
    ImuOrientationDetect(AccGetData());
		//电池电压电流采样更新
    BatteryVoltageUpdate();
		OSTimeDlyHMSM(0,0,0,20,OS_OPT_TIME_HMSM_STRICT,&err);
	}
}

/**
 * @Description Message任务 20hz
 */
void Message_task(void *p_arg){
	OS_ERR err;
	p_arg = p_arg;
	while(1){
		//发送参数信息
		if(SendPID() == Report_SET){
			SendParaInfo();
			ResetSendPID();
		}
		//发送传感器数据
		SendRTInfo();
		OSTimeDlyHMSM(0,0,0,50,OS_OPT_TIME_HMSM_STRICT,&err);
	}
}


