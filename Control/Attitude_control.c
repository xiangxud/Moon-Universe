#include "Attitude_control.h"
FPS_AttitudeControl FPSAttitudeControl;
AttitudeControl AttitudeControlValue;
/**********************************************************************************************************
*函 数 名: Attitude_Control
*功能说明: 飞行内环控制，包括姿态内环和高度内环控制
*形    参: 角速度测量值
*返 回 值: 无
**********************************************************************************************************/
void Attitude_InnerController(Vector3f_t EstimateGyro){
	OS_ERR err;
	Vector3f_t Expect_Gyro,ErrorGyro,ErrorThrust;
	//计算函数运行时间间隔
	FPSAttitudeControl.CurrentTime = (OSTimeGet(&err) - FPSAttitudeControl.LastTime) * 1e-3;
	FPSAttitudeControl.CurrentTime = ConstrainFloat(FPSAttitudeControl.CurrentTime, 0.0005, 0.003);
  FPSAttitudeControl.LastTime = OSTimeGet(&err);
	static float AttitudeCycleTime = 0.002f;
	//期望角速率选择
	if(GetCopterTest() == Drone_Mode_RatePitch || 
							GetCopterTest() == Drone_Mode_RateRoll){
		Expect_Gyro = GetRemoteControlAngleVel();
	}else{
		Expect_Gyro = GetExpectAnguleRate();
	}
	
	//计算角速度环控制误差：目标角速度 - 实际角速度（低通滤波后的陀螺仪测量值）
	ErrorGyro.x = Expect_Gyro.x - EstimateGyro.x;
	ErrorGyro.y = Expect_Gyro.y - EstimateGyro.y;
	ErrorGyro.z = Expect_Gyro.z - EstimateGyro.z;

	//角速度前馈20hz低通滤波
	float GyroxFeedforwardLpf = (EstimateGyro.x - AttitudeControlValue.LastEstimateGyro.x) / AttitudeCycleTime;
	float GyroyFeedforwardLpf = (EstimateGyro.y - AttitudeControlValue.LastEstimateGyro.y) / AttitudeCycleTime;
	float GyrozFeedforwardLpf = (EstimateGyro.z - AttitudeControlValue.LastEstimateGyro.z) / AttitudeCycleTime;
	GyroxFeedforwardLpf = AttitudeControlValue.LastDerivative.x + 
			(AttitudeCycleTime / (7.9577e-3f + AttitudeCycleTime)) * (GyroxFeedforwardLpf - AttitudeControlValue.LastDerivative.x);
	GyroyFeedforwardLpf = AttitudeControlValue.LastDerivative.y + 
			(AttitudeCycleTime / (7.9577e-3f + AttitudeCycleTime)) * (GyroyFeedforwardLpf - AttitudeControlValue.LastDerivative.y);
	GyrozFeedforwardLpf = AttitudeControlValue.LastDerivative.z + 
			(AttitudeCycleTime / (7.9577e-3f + AttitudeCycleTime)) * (GyrozFeedforwardLpf - AttitudeControlValue.LastDerivative.z);
	
	AttitudeControlValue.LastEstimateGyro.x = EstimateGyro.x;
	AttitudeControlValue.LastEstimateGyro.y = EstimateGyro.y;
	AttitudeControlValue.LastEstimateGyro.z = EstimateGyro.z;
	AttitudeControlValue.LastDerivative.x = GyroxFeedforwardLpf;
	AttitudeControlValue.LastDerivative.y = GyroyFeedforwardLpf;
	AttitudeControlValue.LastDerivative.z = GyrozFeedforwardLpf;
	
	//计算扭矩误差
	ErrorThrust.x = Expect_Gyro.y * (Inertia_Wz * Expect_Gyro.z) - (Inertia_Wy * Expect_Gyro.y) * Expect_Gyro.z 
											+ Inertia_Wx * GyroxFeedforwardLpf
													 - AttitudeControlValue.Thrust.x;
	ErrorThrust.y = -(Expect_Gyro.x * (Inertia_Wz * Expect_Gyro.z) - (Inertia_Wx * Expect_Gyro.x) * Expect_Gyro.z)
											+ Inertia_Wy * GyroyFeedforwardLpf
													 - AttitudeControlValue.Thrust.y;
	ErrorThrust.z = Expect_Gyro.x * (Inertia_Wy * Expect_Gyro.y) - (Inertia_Wx * Expect_Gyro.x) * Expect_Gyro.y
											+ Inertia_Wz * GyrozFeedforwardLpf
													 - AttitudeControlValue.Thrust.z;
	
	//计算出角速度环的控制量
	AttitudeControlValue.Thrust.x = PID_GetP(&OriginalWxRate, ErrorGyro.x) + OriginalWxRate.kD * ErrorThrust.x
												//轴间耦合
												+ EstimateGyro.y * (Inertia_Wz * EstimateGyro.z) - (Inertia_Wy * EstimateGyro.y) * EstimateGyro.z
															//角速度前馈
															+ Inertia_Wx * GyroxFeedforwardLpf;
	
	AttitudeControlValue.Thrust.y = PID_GetP(&OriginalWyRate, ErrorGyro.y) + OriginalWyRate.kD * ErrorThrust.y
												+ (-(EstimateGyro.x * (Inertia_Wz * EstimateGyro.z) - (Inertia_Wx * EstimateGyro.x) * EstimateGyro.z))
															+ Inertia_Wy * GyroyFeedforwardLpf;
															
	AttitudeControlValue.Thrust.z = PID_GetP(&OriginalWzRate, ErrorGyro.z) + OriginalWzRate.kD * ErrorThrust.z
												+ EstimateGyro.x * (Inertia_Wy * EstimateGyro.y) - (Inertia_Wx * EstimateGyro.x) * EstimateGyro.y
												      + Inertia_Wz * GyrozFeedforwardLpf;
															
}

/**********************************************************************************************************
*函 数 名: AttitudeOuterControl
*功能说明: 姿态外环控制
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void Attitude_OuterController(void){
  Vector3angle_t Angle,VIOAngle,ErrorAngle;
	Vector3angle_t Expect_Angle;
	//期望角度选择
	if(GetCopterTest() == Drone_Mode_Pitch || 
							GetCopterTest() == Drone_Mode_Roll){
		//手柄控制
		Expect_Angle = GetRemoteControlAngle();
	}else{
		//位置控制
		Expect_Angle = GetDesiredControlAngle();
	}

	//获取当前飞机的姿态角(mahany filter)
  Angle = GetCopterAngle();
	//获取视觉里程计姿态角
	VIOAngle = GetVisualOdometryAngle();
	//计算姿态外环控制误差：目标角度 - 实际角度
	ErrorAngle.roll = Expect_Angle.roll - Angle.roll;
	ErrorAngle.pitch = Expect_Angle.pitch - Angle.pitch;
	ErrorAngle.yaw = Expect_Angle.yaw - VIOAngle.yaw;
	//PID算法，计算出姿态外环的控制量 限幅 2.5rad/s
	AttitudeControlValue.ExpectWRate.x = PID_GetP(&OriginalRoll,  ErrorAngle.roll);
	AttitudeControlValue.ExpectWRate.x = ConstrainFloat(AttitudeControlValue.ExpectWRate.x,-2.5,2.5); 
	AttitudeControlValue.ExpectWRate.y = PID_GetP(&OriginalPitch, ErrorAngle.pitch);
	AttitudeControlValue.ExpectWRate.y = ConstrainFloat(AttitudeControlValue.ExpectWRate.y,-2.5,2.5);
	AttitudeControlValue.ExpectWRate.z = PID_GetP(&OriginalYaw, ErrorAngle.yaw);
	AttitudeControlValue.ExpectWRate.z = ConstrainFloat(AttitudeControlValue.ExpectWRate.z,-2.5,2.5);
}
/**********************************************************************************************************
*函 数 名: GetExpectAnguleRate
*功能说明: 获取期望角速度
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
Vector3f_t GetExpectAnguleRate(void){
	return AttitudeControlValue.ExpectWRate;
}
/**********************************************************************************************************
*函 数 名: GetExpectThrust
*功能说明: 获取期望推力
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
Vector3f_t GetExpectThrust(void){
	return AttitudeControlValue.Thrust;
}

/**********************************************************************************************************
*函 数 名: GetFPSAttitudeControl
*功能说明: 获取姿态控制频率
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
float GetFPSAttitudeControl(void){
	return FPSAttitudeControl.CurrentTime;
}

/**********************************************************************************************************
*函 数 名: ResetAttitudeControlValue
*功能说明: 置位姿态控制里面的部分参数
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void ResetAttitudeControlValue(void){
	//期望角速度置位
	AttitudeControlValue.ExpectWRate.x = 0;
	AttitudeControlValue.ExpectWRate.y = 0;
	AttitudeControlValue.ExpectWRate.z = 0;
	
	AttitudeControlValue.LastEstimateGyro.x = 0;
	AttitudeControlValue.LastEstimateGyro.y = 0;
	AttitudeControlValue.LastEstimateGyro.z = 0;
	
	AttitudeControlValue.Thrust.x = 0;
	AttitudeControlValue.Thrust.y = 0;
	AttitudeControlValue.Thrust.z = 0;
}



