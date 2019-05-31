#include "Position_control.h"
Vector4PosController PosControllerOut;
FPS_PositionControl FPSPositionControl;
FPS_PositionControl FPSAltitudeControl;


bool almostZero(float value){
  return fabs(value) < kAlmostZeroValueThreshold;
}

bool almostZeroThrust(float thrust_value){
  return fabs(thrust_value) < kAlmostZeroThrustThreshold;
}

bool almostZeroRocker(float rocker_value){
  return fabs(rocker_value) < kAlmostZeroRockerThreshold;
}

/*******************************************************************************************************
*函 数 名: Position_OuterController
*功能说明: 位置外环控制
*形    参: 无
*返 回 值: 无
********************************************************************************************************/	
void Position_OuterController(void){
	Vector3f_t ExpectPos;
	//获取阶跃信号
	ExpectPos.x = GetWayPointRefPos().x;
	ExpectPos.y = GetWayPointRefPos().y;
	ExpectPos.z = GetStepSignalValue().z;
	//速度限幅在1.5m/s
	PosControllerOut.ExpectVel.x = ExpectPos.x - GetVisualOdometryPos().x;
	PosControllerOut.ExpectVel.x = ConstrainFloat(PosControllerOut.ExpectVel.x,-1.5,1.5);
	//速度限幅在1.5m/s
	PosControllerOut.ExpectVel.y = ExpectPos.y - GetVisualOdometryPos().y;
	PosControllerOut.ExpectVel.y = ConstrainFloat(PosControllerOut.ExpectVel.y,-1.5,1.5);	
	//速度限幅在1.5m/s
	PosControllerOut.ExpectVel.z = OriginalPosZ.kP * (ExpectPos.z - GetCopterPosition().z);
	PosControllerOut.ExpectVel.z = ConstrainFloat(PosControllerOut.ExpectVel.z,-1.5,1.5);	
}

/**********************************************************************************************************
*函 数 名: Position_InnerController
*功能说明: 位置内环控制器
*形    参: 期望位置，期望速度，估计位置，估计数据
*返 回 值: 无
**********************************************************************************************************/
 void Position_InnerController(void){
	OS_ERR err;
	Vector3f_t ErrorVel;
	static uint8_t FlightMethod;
	//计算函数运行时间间隔
	FPSPositionControl.CurrentTime = (OSTimeGet(&err) - FPSPositionControl.LastTime) * 1e-3;
	FPSPositionControl.CurrentTime = ConstrainFloat(FPSPositionControl.CurrentTime, 0.005, 0.02);
  FPSPositionControl.LastTime = OSTimeGet(&err);
	static float PositionCycleTime = 0.005f;
	//获取飞行模式
	FlightMethod = GetCopterFlightMethod();
	//模式选择
	switch(FlightMethod){
		//姿态飞行控制
		case 0:
			//姿态
			PureAttitude_Control();
			break;
		//定高飞行
		case 1:
			PureAttitude_Control();
			break;
		//遥控飞行
		case 2:
			//转化到速度为2.0m/s最高
			PosControllerOut.ExpectVel.x = -GetRemoteControlFlyData().Xaxis * 0.008f; 
			PosControllerOut.ExpectVel.y = -GetRemoteControlFlyData().Yaxis * 0.008f;
			//速度误差计算
			ErrorVel.x = PosControllerOut.ExpectVel.x - GetVisualOdometryVel().x;
			ErrorVel.y = PosControllerOut.ExpectVel.y - GetVisualOdometryVel().y;
			//角度转化为rad弧度
			PosControllerOut.ExpectAngle.roll = - PID_GetPID(&OriginalVelY, ErrorVel.y, PositionCycleTime) * PI/180;
			PosControllerOut.ExpectAngle.roll = ConstrainFloat(PosControllerOut.ExpectAngle.roll, -0.35, 0.35);
			PosControllerOut.ExpectAngle.pitch = PID_GetPID(&OriginalVelX, ErrorVel.x, PositionCycleTime) * PI/180;
			PosControllerOut.ExpectAngle.pitch = ConstrainFloat(PosControllerOut.ExpectAngle.pitch, -0.35, 0.35); //角度不大于20度
			PosControllerOut.ExpectAngle.yaw = 0;	
			break;
		//阶跃响应
		case 3:
			//速度误差计算
			ErrorVel.x = PosControllerOut.ExpectVel.x - GetVisualOdometryVel().x;
			ErrorVel.y = PosControllerOut.ExpectVel.y - GetVisualOdometryVel().y;
			//角度转化为rad弧度
			PosControllerOut.ExpectAngle.roll = - PID_GetPID(&OriginalVelY, ErrorVel.y, PositionCycleTime) * PI/180;
			PosControllerOut.ExpectAngle.roll = ConstrainFloat(PosControllerOut.ExpectAngle.roll, -0.35, 0.35);
			PosControllerOut.ExpectAngle.pitch = PID_GetPID(&OriginalVelX, ErrorVel.x, PositionCycleTime) * PI/180;
			PosControllerOut.ExpectAngle.pitch = ConstrainFloat(PosControllerOut.ExpectAngle.pitch, -0.35, 0.35); //角度不大于20度
			PosControllerOut.ExpectAngle.yaw = 0;	
			break;
		//轨迹追踪
		case 4:		
			HighLevel_Control();
			break;
		   
		default:
			break;
		}
}

/*********************************************************************************************************
*函 数 名: PureAttitude_Control
*功能说明: 纯姿态控制
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/	
void PureAttitude_Control(void){
	//姿态
	PosControllerOut.ExpectAngle.pitch = -GetRemoteControlFlyData().Xaxis * 0.04f * PI/180;

	PosControllerOut.ExpectAngle.roll = GetRemoteControlFlyData().Yaxis * 0.04f * PI/180;

	PosControllerOut.ExpectAngle.yaw = 0;	
}

/********************************************************************************************************
*函 数 名: Altitude_Controller
*功能说明: 高度控制
*形    参: 无
*返 回 值: 无
*********************************************************************************************************/	
void Altitude_Controller(void){
	OS_ERR err;
	static uint8_t AltitudeFlightMethod;
	static float EstimateAltitudeVelLpf;
	float ErrorAltitude;
	//计算函数运行时间间隔
	FPSAltitudeControl.CurrentTime = (OSTimeGet(&err) - FPSAltitudeControl.LastTime) * 1e-3;
	FPSAltitudeControl.CurrentTime = ConstrainFloat(FPSAltitudeControl.CurrentTime, 0.0005, 0.01);
  FPSAltitudeControl.LastTime = OSTimeGet(&err);
	static float AltitudeCycleTime = 0.005f;
	//获取飞行模式
	AltitudeFlightMethod = GetCopterFlightMethod();
	if(AltitudeFlightMethod == PurePosture){
		//纯姿态
		PosControllerOut.ExpectAcc = GetRemoteControlFlyData().Zaxis * 2.5f + Gravity_Acceleration;
	}else{
		/******* 降落控制 ********/	
		if(GetCopterFlyMode() == Land){
			PosControllerOut.ExpectVel.z = -0.15f;
			if(GetCopterPosition().z < 0.1f){
				SetCopterFlyMode(Nothing);
				SetCopterStatus(Drone_Off);
			}
		}
		//对速度测量值进行低通滤波，减少数据噪声对控制器的影响
		EstimateAltitudeVelLpf = EstimateAltitudeVelLpf * 0.9f + GetCopterVelocity().z * 0.1f;
		//速度误差计算
		ErrorAltitude = PosControllerOut.ExpectVel.z - EstimateAltitudeVelLpf;
		//PID计算
		PosControllerOut.ExpectAcc = PID_GetPID(&OriginalVelZ, ErrorAltitude, AltitudeCycleTime) + Gravity_Acceleration;
	}
}

/**********************************************************************************************************
*函 数 名: ComputeNominalReferenceInputs
*功能说明: 计算参考输入作为前馈项
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/	
void ComputeNominalReferenceInputs(void){
	
}

/**********************************************************************************************************
*函 数 名: ComputePIDErrorAcc
*功能说明: 利用PID计算误差加速度
*形    参: 
*返 回 值: 
**********************************************************************************************************/	
Vector3f_t ComputePIDErrorAcc(void){
	
	//PID计算世界框架下的ACC误差
	Vector3f_t acc_error;
	// X轴加速度
	float x_pos_error = GetWayPointRefPos().x - GetVisualOdometryPos().x;
	x_pos_error = ConstrainFloat(x_pos_error,-pxy_error_max,pxy_error_max);
	float x_vel_error = PosControllerOut.ExpectVel.x - GetVisualOdometryVel().x;
	x_vel_error = ConstrainFloat(x_vel_error,-vxy_error_max,vxy_error_max);
	
	acc_error.x = PID_GetPID(&OriginalPosX,x_pos_error,FPSPositionControl.CurrentTime)
									+ PID_GetPID(&OriginalVelX,x_vel_error,FPSPositionControl.CurrentTime);
	
	// Y轴加速度
	float y_pos_error = GetWayPointRefPos().y - GetVisualOdometryPos().y;
	y_pos_error = ConstrainFloat(y_pos_error,-pxy_error_max,pxy_error_max);
	float y_vel_error = PosControllerOut.ExpectVel.y - GetVisualOdometryVel().y;
	y_vel_error = ConstrainFloat(y_vel_error,-vxy_error_max,vxy_error_max);
	
	acc_error.y = PID_GetPID(&OriginalPosY,y_pos_error,FPSPositionControl.CurrentTime)
									+ PID_GetPID(&OriginalVelY,y_vel_error,FPSPositionControl.CurrentTime);

	// Z轴加速度 来自TOF
	float z_pos_error = GetStepSignalValue().z - GetCopterPosition().z;
	z_pos_error = ConstrainFloat(z_pos_error,-pz_error_max,pz_error_max);
	float z_vel_error = PosControllerOut.ExpectVel.z - GetVisualOdometryVel().z;
	z_vel_error = ConstrainFloat(z_vel_error,-vz_error_max,vz_error_max);
	
//	acc_error.z = PID_GetPID(&OriginalPosZ,z_pos_error,FPSPositionControl.CurrentTime)
//									+ PID_GetPID(&OriginalVelZ,z_vel_error,FPSPositionControl.CurrentTime);
	acc_error.z = 15.0f * z_pos_error + 10.0f * z_vel_error;
	
	return acc_error;
}

/**********************************************************************************************************
*函 数 名: ComputeRobustBodyXAxis
*功能说明: 计算body坐标系
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/	
Vector3f_t ComputeRobustBodyXAxis(Vector3f_t x_B_prototype,Vector3f_t x_C,Vector3f_t y_C){
	Vector3f_t x_B = x_B_prototype;
	float x_BNorm = x_B.x * x_B.x + x_B.y * x_B.y + x_B.z * x_B.z;
	if(almostZero(x_BNorm)){
		// 若y_c × z_b == 0 则它们共线 此时是垂直运动，暂不考虑
		x_B.x = x_B.x / sqrtf(x_BNorm);
		x_B.y = x_B.y / sqrtf(x_BNorm);
		x_B.z = x_B.z / sqrtf(x_BNorm);
	}else{
		x_B.x = x_B.x / sqrtf(x_BNorm);
		x_B.y = x_B.y / sqrtf(x_BNorm);
		x_B.z = x_B.z / sqrtf(x_BNorm);
	}
	return x_B;
	
}

/**********************************************************************************************************
*函 数 名: ComputeDesiredAttitude
*功能说明: 计算期望姿态
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/	
void ComputeDesiredAttitude(Vector3f_t DesiredAcceleration,float reference_heading){
	//若航向产生旋转 则需计算
	float headingOrientation[9];
	headingOrientation[0] = cosf(reference_heading);
	headingOrientation[1] = -sinf(reference_heading);
	headingOrientation[2] = 0.0f;
	headingOrientation[3] = sinf(reference_heading);
	headingOrientation[4] = cosf(reference_heading);
	headingOrientation[5] = 0.0f;
	headingOrientation[6] = 0.0f;
	headingOrientation[7] = 0.0f;
	headingOrientation[8] = 1.0f;
	float UnitX[3] = {1,0,0};
	float UnitY[3] = {0,1,0};
	Vector3f_t X_C,Y_C;
	//计算期望方向
	Matrix3_Mul_Matrix1(headingOrientation,UnitX,&X_C);
	Matrix3_Mul_Matrix1(headingOrientation,UnitY,&Y_C);
	
	Vector3f_t Z_B;
	float AccNorm = DesiredAcceleration.x * DesiredAcceleration.x 
										+ DesiredAcceleration.y * DesiredAcceleration.y 
												+ DesiredAcceleration.z + DesiredAcceleration.z;
	//自由落体情况 加速计失去作用
	if(almostZero(AccNorm)){
		/**此处假设不可能产生自由落体运动**/
		AccNorm = 0.05f;
		Z_B.x = DesiredAcceleration.x / sqrtf(AccNorm);
		Z_B.y = DesiredAcceleration.y / sqrtf(AccNorm);
		Z_B.z = DesiredAcceleration.z / sqrtf(AccNorm);
	}else{
		Z_B.x = DesiredAcceleration.x / sqrtf(AccNorm);
		Z_B.y = DesiredAcceleration.y / sqrtf(AccNorm);
		Z_B.z = DesiredAcceleration.z / sqrtf(AccNorm);
	}
/*
	
	单位向量 a = a1*I + a2*J + a3*K   b = b1*I + b2*I2 + b3*I3
	a×b = (a2b3 - a3b2)I + (a3b1 - a1b3)J + (a1b2 - a2b1)K
	
				a2b3 - a3b2
	a×b = a3b1 - a1b3
				a1b2 - a2b1
	
*/
	Vector3f_t x_B_prototype;
	x_B_prototype.x = Y_C.y * Z_B.z - Y_C.z * Z_B.y;
	x_B_prototype.y = Y_C.z * Z_B.x - Y_C.x * Z_B.z;
	x_B_prototype.z = Y_C.x * Z_B.y - Y_C.y * Z_B.x;
	Vector3f_t X_B = ComputeRobustBodyXAxis(x_B_prototype,X_C,Y_C);
	
	Vector3f_t Y_B;
	Y_B.x = Z_B.y * X_B.z - Z_B.z * X_B.y;
	Y_B.y = Z_B.z * X_B.x - Z_B.x * X_B.z;
	Y_B.z = Z_B.x * X_B.y - Z_B.y * X_B.x;
	float Y_BNorm = Y_B.x * Y_B.x + Y_B.y * Y_B.y + Y_B.z * Y_B.z;
	Y_B.x = Y_B.x / sqrtf(Y_BNorm);
	Y_B.y = Y_B.y / sqrtf(Y_BNorm);
	Y_B.z = Y_B.z / sqrtf(Y_BNorm);
	
	float R_W_B[9];
	R_W_B[0] = X_B.x;R_W_B[1] = Y_B.x;R_W_B[2] = Z_B.x;
	R_W_B[3] = X_B.y;R_W_B[4] = Y_B.y;R_W_B[5] = Z_B.y;
	R_W_B[6] = X_B.z;R_W_B[7] = Y_B.z;R_W_B[8] = Z_B.z;
	
	if(almostZeroRocker(GetRemoteControlFlyData().Xaxis) && almostZeroRocker(GetRemoteControlFlyData().Yaxis)){
		PosControllerOut.ExpectAngle.roll = atan2(R_W_B[7],R_W_B[8]);
		PosControllerOut.ExpectAngle.roll = ConstrainFloat(PosControllerOut.ExpectAngle.roll, -0.2, 0.2);
		PosControllerOut.ExpectAngle.pitch = -atan2(R_W_B[6],sqrtf(R_W_B[7]*R_W_B[7] + R_W_B[8]*R_W_B[8]));
		PosControllerOut.ExpectAngle.pitch = ConstrainFloat(PosControllerOut.ExpectAngle.pitch, -0.2, 0.2);
		PosControllerOut.ExpectAngle.yaw = atan2(R_W_B[3],R_W_B[0]);
	}else{
		PureAttitude_Control();
	}
}

/**********************************************************************************************************
*函 数 名: ComputeDesiredCollectiveMassNormalizedThrust
*功能说明: 
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/	
void ComputeDesiredCollectiveMassNormalizedThrust(Vector3f_t DesiredAcceleration){
	
	Vector4q_t CopterQuaternion = GetCopterQuaternion();
	PosControllerOut.ExpectAcc = DesiredAcceleration.x * (2 * CopterQuaternion.qw * CopterQuaternion.qy + 2 * CopterQuaternion.qx * CopterQuaternion.qz)
							+ DesiredAcceleration.y * (2 * CopterQuaternion.qy * CopterQuaternion.qz - 2 * CopterQuaternion.qw * CopterQuaternion.qx)
	+ DesiredAcceleration.z * (CopterQuaternion.qw * CopterQuaternion.qw - CopterQuaternion.qx * CopterQuaternion.qx - CopterQuaternion.qy * CopterQuaternion.qy + CopterQuaternion.qz * CopterQuaternion.qz);
	
	PosControllerOut.ExpectAcc -= 0.005 * (pow(GetVisualOdometryVel().x, 2.0) + pow(GetVisualOdometryVel().y, 2.0));
	
}

/**********************************************************************************************************
*函 数 名: HighLevelControl
*功能说明: 高阶位置控制器
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/	
void HighLevel_Control(void){
	//计算期望控制命令 Compute desired control commands
	Vector3f_t PidErrorAccelerations,DesiredAcceleration;
	PidErrorAccelerations = ComputePIDErrorAcc();
	//目前没有来自轨迹的参考加速度
	DesiredAcceleration .x = PidErrorAccelerations.x; // + GetWayPointRefAcc().x
	DesiredAcceleration .y = PidErrorAccelerations.y; // + GetWayPointRefAcc().y
	DesiredAcceleration .z = PidErrorAccelerations.z + Gravity_Acceleration; // + GetWayPointRefAcc().z
	ComputeDesiredAttitude(DesiredAcceleration,0);	
}

/**********************************************************************************************************
*函 数 名: GetDesiredControlAcc
*功能说明: 获取期望加速度
*形    参: 
*返 回 值: 无
**********************************************************************************************************/	
float GetDesiredControlAcc(void){
	return PosControllerOut.ExpectAcc;
}
/**********************************************************************************************************
*函 数 名: GetDesiredControlAngle
*功能说明: 获取期望角度值
*形    参: 期望位置，期望速度，估计位置，估计数据
*返 回 值: 无
**********************************************************************************************************/	
Vector3angle_t GetDesiredControlAngle(void){
	return PosControllerOut.ExpectAngle;
}
/**********************************************************************************************************
*函 数 名: ResetPositionPara
*功能说明: 置位位置参数
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/	
void ResetPositionControlValue(void){
	//位移积分归零
	OriginalPosX.integrator = 0;
	OriginalPosY.integrator = 0;
	OriginalPosZ.integrator = 0;
	//位置积分最大
	OriginalPosX.imax = 10; 
	OriginalPosY.imax = 10;
	//速度积分归零
	OriginalVelX.integrator = 0;
	OriginalVelY.integrator = 0;
	OriginalVelZ.integrator = 0;
	//位置速度积分最大
	OriginalVelX.imax = 10;
	OriginalVelY.imax = 10;
	//高度速度积分最大
	OriginalVelZ.imax = 10;
	//位置输出归零
	PosControllerOut.ExpectAcc = 0;
	PosControllerOut.ExpectAngle.pitch = 0;
	PosControllerOut.ExpectAngle.roll = 0;
	PosControllerOut.ExpectAngle.yaw = 0;
	PosControllerOut.ExpectVel.x = 0;
	PosControllerOut.ExpectVel.y = 0;
	PosControllerOut.ExpectVel.z = 0;
}
