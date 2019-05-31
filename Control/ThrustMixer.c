/**********************************************************************************************************
 * @文件     ThrustMixer.c
 * @说明     推力融合函数
 * @作者     YuyingJin
 * @网站     https://yuyingjin0111.github.io/
 * @日期     2018 ~
**********************************************************************************************************/
#include "ThrustMixer.h"
Throttle ThrottleInfo;
ThrustUav UavThrust;
/***********************************************************************************************
*函 数 名: ThrustMixer
*功能说明: 推力融合
*形    参: 四旋翼臂长arm_length . 转轴扭矩RotateThrust
*返 回 值: 无
************************************************************************************************/
/*
													   T-motor 转矩到力的转化
	%拟合二次函数
	x=[0.00849; 0.01643; 0.02385; 0.03312; 0.04725; 0.05911; 0.06824; 0.07682; 0.0892; 0.09991; 0.11171; 0.1238; 0.14093; 0.14872; 0.16151; 0.17324; 0.18627; 0.19669; 0.18676]; %转矩
	y=[0.1672; 0.4243; 0.7632; 1.1474; 1.6233; 2.1165; 2.7504; 3.3215; 4.0629; 4.8842; 5.7705; 6.6019; 7.3509; 8.1856; 9.0618; 9.7409; 10.6358; 11.3843; 12.1291]; %推力数据 单位N
	p=polyfit(x,y,2);  % 拟合出的二次函数的系数
	ye=y-polyval(p,x);  % 计算误差
	ye2s=sum(ye.^2); % 误差的平方和
	disp(sprintf('误差的平方和=%d',ye2s));
	xx=linspace(min(x),max(x));  % 绘图用到的点的横坐标
	yy=polyval(p,xx);   % 拟合曲线的纵坐标
	plot(x,y,'o',xx,yy);  % 绘图，原始数据+拟合曲线
	legend('原始数据','拟合曲线');  % 图示
	s=char(vpa(poly2sym(p,'x'),5));  % 二次函数式转换为字符串，vpa转换小数，保留5位有效数字
	title(['y=' s]);
	
	A = 115.74
	B = 39.352
	C = -0.31518
*/

/*
													   T-motor 力到转矩的转化
	%拟合二次函数
	x=[0.1672; 0.4243; 0.7632; 1.1474; 1.6233; 2.1165; 2.7504; 3.3215; 4.0629; 4.8842; 5.7705; 6.6019; 7.3509; 8.1856; 9.0618; 9.7409; 10.6358; 11.3843; 12.1291]; %推力数据 单位N
	y=[0.00849; 0.01643; 0.02385; 0.03312; 0.04725; 0.05911; 0.06824; 0.07682; 0.0892; 0.09991; 0.11171; 0.1238; 0.14093; 0.14872; 0.16151; 0.17324; 0.18627; 0.19669; 0.18676]; %转矩
	p=polyfit(x,y,2);  % 拟合出的二次函数的系数
	ye=y-polyval(p,x);  % 计算误差
	ye2s=sum(ye.^2); % 误差的平方和
	disp(sprintf('误差的平方和=%d',ye2s));
	xx=linspace(min(x),max(x));  % 绘图用到的点的横坐标
	yy=polyval(p,xx);   % 拟合曲线的纵坐标
	plot(x,y,'o',xx,yy);  % 绘图，原始数据+拟合曲线
	legend('原始数据','拟合曲线');  % 图示
	s=char(vpa(poly2sym(p,'x'),5));  % 二次函数式转换为字符串，vpa转换小数，保留5位有效数字
	title(['y=' s]);
	
	A = -0.00050758
	B = 0.021521
	C = 0.0093181
*/



void ThrustMixer(float arm_length){
	// 获取期望推力
	Vector3f_t RotateThrust = GetExpectThrust();
	// 获取期望加速度
	float HeightAccValue = GetDesiredControlAcc();
	
	if(GetCopterTest()==Drone_Mode_Pitch || GetCopterTest()==Drone_Mode_RatePitch){
		UavThrust.f1 = +1.414f / (arm_length * 4.0f) * RotateThrust.y + Gravity_Acceleration * Drone_Mass / 4.0f;	
		UavThrust.f2 = -1.414f / (arm_length * 4.0f) * RotateThrust.y + Gravity_Acceleration * Drone_Mass / 4.0f;
		UavThrust.f3 = +1.414f / (arm_length * 4.0f) * RotateThrust.y + Gravity_Acceleration * Drone_Mass / 4.0f;
		UavThrust.f4 = -1.414f / (arm_length * 4.0f) * RotateThrust.y + Gravity_Acceleration * Drone_Mass / 4.0f;
	}else if(GetCopterTest()==Drone_Mode_Roll || GetCopterTest()==Drone_Mode_RateRoll){
		UavThrust.f1 = -1.414f / (arm_length * 4.0f) * RotateThrust.x + Gravity_Acceleration * Drone_Mass / 4.0f;
		UavThrust.f2 = -1.414f / (arm_length * 4.0f) * RotateThrust.x + Gravity_Acceleration * Drone_Mass / 4.0f;
		UavThrust.f3 = +1.414f / (arm_length * 4.0f) * RotateThrust.x + Gravity_Acceleration * Drone_Mass / 4.0f;
		UavThrust.f4 = +1.414f / (arm_length * 4.0f) * RotateThrust.x + Gravity_Acceleration * Drone_Mass / 4.0f;
	}else if(GetCopterTest()==Drone_Mode_4Axis){
		UavThrust.f1 = - 1.414f / (arm_length * 4.0f) * RotateThrust.x  																		//roll
										+ 1.414f / (arm_length * 4.0f) * RotateThrust.y                                  		//pitch
											+ 1.0f / (Drag_Coeff * 4.0f) * RotateThrust.z                                     //yaw	
												+ HeightAccValue * Drone_Mass / 4.0f;			  											  						//mass		 																									
		
		UavThrust.f2 = - 1.414f / (arm_length * 4.0f) * RotateThrust.x
										- 1.414f / (arm_length * 4.0f) * RotateThrust.y
											- 1.0f / (Drag_Coeff * 4.0f) * RotateThrust.z
												+ HeightAccValue * Drone_Mass / 4.0f;									

		UavThrust.f3 = + 1.414f / (arm_length * 4.0f) * RotateThrust.x
										+ 1.414f / (arm_length * 4.0f) * RotateThrust.y
											- 1.0f / (Drag_Coeff * 4.0f) * RotateThrust.z
												+ HeightAccValue * Drone_Mass / 4.0f;

		UavThrust.f4 = + 1.414f / (arm_length * 4.0f) * RotateThrust.x 
										- 1.414f / (arm_length * 4.0f) * RotateThrust.y
											+ 1.0f / (Drag_Coeff * 4.0f) * RotateThrust.z
											  + HeightAccValue * Drone_Mass / 4.0f;
		
	}
	MotorThrust(UavThrust.f1,UavThrust.f2,UavThrust.f3,UavThrust.f4);
}
/**********************************************************************************************************
*函 数 名: PreTakeOff
*功能说明: 预起飞函数
*形    参: 
*返 回 值: 无
**********************************************************************************************************/
void PreTakeOff(uint16_t Time){
	float ThurstValue = sqrtf(Time/50) * Drone_Mass / 4.0f;
	MotorThrust(ThurstValue,ThurstValue,ThurstValue,ThurstValue);
}
/***********************************************************************************************
*函 数 名: MotorThrust
*功能说明: 电机推力生成
*形    参: 无
*返 回 值: 无
************************************************************************************************/

/*
															T-Motor 2214 电机 
	%拟合二次函数
	14.8V
	x= %推力数据 单位N
	y= %油门数据 占空比
	p=polyfit(x,y,2);  % 拟合出的二次函数的系数
	ye=y-polyval(p,x);  % 计算误差
	ye2s=sum(ye.^2); % 误差的平方和
	disp(sprintf('误差的平方和=%d',ye2s));
	xx=linspace(min(x),max(x));  % 绘图用到的点的横坐标
	yy=polyval(p,xx);   % 拟合曲线的纵坐标
	plot(x,y,'o',xx,yy);  % 绘图，原始数据+拟合曲线
	legend('原始数据','拟合曲线');  % 图示
	s=char(vpa(poly2sym(p,'x'),5));  % 二次函数式转换为字符串，vpa转换小数，保留5位有效数字
	title(['y=' s]);
	
	A = 0.002805
	B = 0.033446
	C = 0.329380
	
*/


void MotorThrust(float f1,float f2,float f3,float f4){
	
	float M1,M2,M3,M4;

#ifdef T_Motor 
	M1 = (0.002805f * f1 * f1 + 0.033446f * f1 + 0.329380f);
	M2 = (0.002805f * f2 * f2 + 0.033446f * f2 + 0.329380f);
	M3 = (0.002805f * f3 * f3 + 0.033446f * f3 + 0.329380f);
	M4 = (0.002805f * f4 * f4 + 0.033446f * f4 + 0.329380f);
#else
	M1 = 0.06531f * f1 - 0.00288f;
	M2 = 0.06531f * f2 - 0.00288f;
	M3 = 0.06531f * f3 - 0.00288f;
	M4 = 0.06531f * f4 - 0.00288f;
#endif
	
	ThrottleInfo.M1 = (int)(M1 * 1000.0f);
	ThrottleInfo.M2 = (int)(M2 * 1000.0f);
	ThrottleInfo.M3 = (int)(M3 * 1000.0f);
	ThrottleInfo.M4 = (int)(M4 * 1000.0f);
	
	if(ThrottleInfo.M1 > 850)  ThrottleInfo.M1=850;
	if(ThrottleInfo.M2 > 850)  ThrottleInfo.M2=850;
	if(ThrottleInfo.M3 > 850)  ThrottleInfo.M3=850;
	if(ThrottleInfo.M4 > 850)  ThrottleInfo.M4=850;

	if(ThrottleInfo.M1 < 100)  ThrottleInfo.M1=100;
	if(ThrottleInfo.M2 < 100)  ThrottleInfo.M2=100;
	if(ThrottleInfo.M3 < 100)  ThrottleInfo.M3=100;
	if(ThrottleInfo.M4 < 100)  ThrottleInfo.M4=100;
	
	PWM_OUTPUT(ThrottleInfo.M1,ThrottleInfo.M2,ThrottleInfo.M3,ThrottleInfo.M4);
	
}
/***********************************************************************************************
*函 数 名: PWM_OUTPUT
*功能说明: 电调信号输出
*形    参: 电机参数
*返 回 值: 无
************************************************************************************************/
void PWM_OUTPUT(unsigned int Motor1,unsigned int Motor2,
								 unsigned int Motor3,unsigned int Motor4){
	Motor1+=1000;
	Motor2+=1000;
	Motor3+=1000;
	Motor4+=1000;
#ifdef SpeedyBeeF4
	//实际输出到电机
	TIM8->CCR1=Motor1;
	TIM8->CCR2=Motor2;
	TIM8->CCR3=Motor3;
	TIM8->CCR4=Motor4;
#else
	TIM8->CCR1=Motor3;
	TIM8->CCR2=Motor4;
	TIM8->CCR3=Motor2;
	TIM8->CCR4=Motor1;		 
#endif
}
