/**********************************************************************************************************
 * @文件     Visiondata_deal.c
 * @说明     视觉数据接收
 * @作者     YuyingJin
 * @网站     https://yuyingjin0111.github.io/
 * @日期     2018 ~
**********************************************************************************************************/
#include "Visiondata_deal.h"
VisualOdometryValue VisualOdometryData;
ReferenceRouteValue ReferenceRouteData;

/**********************************************************************************************************
*函 数 名: Vision_datadeal
*功能说明: 接收视觉里程计数据
*形    参: 无
*返 回 值: Position
**********************************************************************************************************/
void Vision_DataDeal(Receive_VisualOdometry rx){
	OS_ERR err;
	//估计状态定义
	float_union position_x,position_y,position_z,                       //估计位置
								velocity_x,velocity_y,velocity_z,										  //估计速度
									Quaternion0,Quaternion1,Quaternion2,Quaternion3,	  //估计姿态
										reference_posx,reference_posy,reference_posz;			//参考航点
//											reference_velx,reference_vely,reference_velz,   //参考速度
//												reference_accx,reference_accy,reference_accz; //参考加速度
	if(rx.buf[0]==0x55 && rx.buf[1]==0xAA && rx.buf[43]==0xAA){
		if(rx.buf[2] == 0x10){
			//计算函数运行时间间隔
			VisualOdometryData.CurrentTime = (OSTimeGet(&err) - VisualOdometryData.LastTime) * 1e-3;
			VisualOdometryData.CurrentTime = ConstrainFloat(VisualOdometryData.CurrentTime, 0.001, 0.05);
			VisualOdometryData.LastTime = OSTimeGet(&err);
			
			//X轴位置数据
			position_x.cv[0] = rx.buf[3];
			position_x.cv[1] = rx.buf[4];
			position_x.cv[2] = rx.buf[5];
			position_x.cv[3] = rx.buf[6];
			//Y轴位置数据
			position_y.cv[0] = rx.buf[7];
			position_y.cv[1] = rx.buf[8];
			position_y.cv[2] = rx.buf[9];
			position_y.cv[3] = rx.buf[10];
			//Z轴位置数据
			position_z.cv[0] = rx.buf[11];
			position_z.cv[1] = rx.buf[12];
			position_z.cv[2] = rx.buf[13];
			position_z.cv[3] = rx.buf[14];
			//X轴速度数据
			velocity_x.cv[0] = rx.buf[15];
			velocity_x.cv[1] = rx.buf[16];
			velocity_x.cv[2] = rx.buf[17];
			velocity_x.cv[3] = rx.buf[18];
			//Y轴速度数据
			velocity_y.cv[0] = rx.buf[19];
			velocity_y.cv[1] = rx.buf[20];
			velocity_y.cv[2] = rx.buf[21];
			velocity_y.cv[3] = rx.buf[22];	
			//Z轴速度数据
			velocity_z.cv[0] = rx.buf[23];
			velocity_z.cv[1] = rx.buf[24];
			velocity_z.cv[2] = rx.buf[25];
			velocity_z.cv[3] = rx.buf[26];
			//视觉里程计的姿态数据
			Quaternion0.cv[0] = rx.buf[27];
			Quaternion0.cv[1] = rx.buf[28];
			Quaternion0.cv[2] = rx.buf[29];
			Quaternion0.cv[3] = rx.buf[30];
			
			Quaternion1.cv[0] = rx.buf[31];
			Quaternion1.cv[1] = rx.buf[32];
			Quaternion1.cv[2] = rx.buf[33];
			Quaternion1.cv[3] = rx.buf[34];
			
			Quaternion2.cv[0] = rx.buf[35];
			Quaternion2.cv[1] = rx.buf[36];
			Quaternion2.cv[2] = rx.buf[37];
			Quaternion2.cv[3] = rx.buf[38];
			
			Quaternion3.cv[0] = rx.buf[39];
			Quaternion3.cv[1] = rx.buf[40];
			Quaternion3.cv[2] = rx.buf[41];
			Quaternion3.cv[3] = rx.buf[42];
			
			//简单滤波
			VisualOdometryData.VIOPosValue.x = position_x.fvalue;
			VisualOdometryData.VIOPosValue.y = position_y.fvalue;
			VisualOdometryData.VIOPosValue.z = position_z.fvalue;
			
			VisualOdometryData.VIOVelValue.x = velocity_x.fvalue;
			VisualOdometryData.VIOVelValue.y = velocity_y.fvalue;
			VisualOdometryData.VIOVelValue.z = velocity_z.fvalue;
			
			VisualOdometryData.VIOAttitude.roll = atan2(2.0f*(Quaternion0.fvalue*Quaternion1.fvalue + Quaternion2.fvalue*Quaternion3.fvalue), 
																1 - 2.0f*(Quaternion1.fvalue*Quaternion1.fvalue + Quaternion2.fvalue*Quaternion2.fvalue));
																
			VisualOdometryData.VIOAttitude.pitch = safe_asin(2.0f*(Quaternion0.fvalue*Quaternion2.fvalue - Quaternion1.fvalue*Quaternion3.fvalue));
			
			VisualOdometryData.VIOAttitude.yaw = atan2(2.0f*Quaternion1.fvalue*Quaternion2.fvalue + 2.0f*Quaternion0.fvalue*Quaternion3.fvalue, 
																-2.0f*Quaternion2.fvalue*Quaternion2.fvalue - 2.0f*Quaternion3.fvalue*Quaternion3.fvalue + 1);	
	
		}
		if(rx.buf[2] == 0x20){
			//计算函数运行时间间隔
			ReferenceRouteData.CurrentTime = (OSTimeGet(&err) - ReferenceRouteData.LastTime) * 1e-3;
			ReferenceRouteData.CurrentTime = ConstrainFloat(ReferenceRouteData.CurrentTime, 0.5, 2.0);
			ReferenceRouteData.LastTime = OSTimeGet(&err);
			//期望的位置数据
			reference_posx.cv[0] = rx.buf[3];
			reference_posx.cv[1] = rx.buf[4];
			reference_posx.cv[2] = rx.buf[5];
			reference_posx.cv[3] = rx.buf[6];

			reference_posy.cv[0] = rx.buf[7];
			reference_posy.cv[1] = rx.buf[8];
			reference_posy.cv[2] = rx.buf[9];
			reference_posy.cv[3] = rx.buf[10];

			reference_posz.cv[0] = rx.buf[11];
			reference_posz.cv[1] = rx.buf[12];
			reference_posz.cv[2] = rx.buf[13];
			reference_posz.cv[3] = rx.buf[14];
//			//期望的速度数据
//			reference_velx.cv[0] = rx.buf[15];
//			reference_velx.cv[1] = rx.buf[16];
//			reference_velx.cv[2] = rx.buf[17];
//			reference_velx.cv[3] = rx.buf[18];

//			reference_vely.cv[0] = rx.buf[19];
//			reference_vely.cv[1] = rx.buf[20];
//			reference_vely.cv[2] = rx.buf[21];
//			reference_vely.cv[3] = rx.buf[22];

//			reference_velz.cv[0] = rx.buf[23];
//			reference_velz.cv[1] = rx.buf[24];
//			reference_velz.cv[2] = rx.buf[25];
//			reference_velz.cv[3] = rx.buf[26];
//			//期望的加速度数据
//			reference_accx.cv[0] = rx.buf[27];
//			reference_accx.cv[1] = rx.buf[28];
//			reference_accx.cv[2] = rx.buf[29];
//			reference_accx.cv[3] = rx.buf[30];
//			
//			reference_accy.cv[0] = rx.buf[31];
//			reference_accy.cv[1] = rx.buf[32];
//			reference_accy.cv[2] = rx.buf[33];
//			reference_accy.cv[3] = rx.buf[34];
//			
//			reference_accz.cv[0] = rx.buf[35];
//			reference_accz.cv[1] = rx.buf[36];
//			reference_accz.cv[2] = rx.buf[37];
//			reference_accz.cv[3] = rx.buf[38];
			
			ReferenceRouteData.FormationPosValue.x = reference_posx.fvalue;
			ReferenceRouteData.FormationPosValue.y = reference_posy.fvalue;
			ReferenceRouteData.FormationPosValue.z = reference_posz.fvalue;
		}
	}
}


/**********************************************************************************************************
*函 数 名: GetVisualOdometryPos
*功能说明: 获取视觉里程计的Pos
*形    参: 无
*返 回 值: Position
**********************************************************************************************************/
Vector3f_t GetVisualOdometryPos(void){
  return VisualOdometryData.VIOPosValue;
}
/**********************************************************************************************************
*函 数 名: GetVisualOdometryVel
*功能说明: 获取视觉里程计的Vel
*形    参: 无
*返 回 值: Velocity
**********************************************************************************************************/
Vector3f_t GetVisualOdometryVel(void){
  return VisualOdometryData.VIOVelValue;
}
/**********************************************************************************************************
*函 数 名: GetVisualOdometryAngle
*功能说明: 获取视觉里程计的Angle
*形    参: 无
*返 回 值: Attitude
**********************************************************************************************************/
Vector3angle_t GetVisualOdometryAngle(void){
  return VisualOdometryData.VIOAttitude;
}

/**********************************************************************************************************
*函 数 名: GetWayPointRefPos
*功能说明: 获取航线规划里面的位移
*形    参: 无
*返 回 值: Velocity
**********************************************************************************************************/
Vector3f_t GetWayPointRefPos(void){
  return ReferenceRouteData.FormationPosValue;
}

/**********************************************************************************************************
*函 数 名: GetWayPointRefAcc
*功能说明: 获取航向规划里面的速度
*形    参: 无
*返 回 值: Acceleration
**********************************************************************************************************/
Vector3f_t GetWayPointRefAcc(void){
	Vector3f_t RefAcceleration;

  return RefAcceleration;
}

/**********************************************************************************************************
*函 数 名: GetFPSVisualOdometry
*功能说明: 返回视觉里程计的FPS
*形    参: 无
*返 回 值: Velocity
**********************************************************************************************************/
float GetFPSVisualOdometry(void){
  return VisualOdometryData.CurrentTime;
}
/**********************************************************************************************************
*函 数 名: GetFPSWayPointNav
*功能说明: 返回的FPS
*形    参: 无
*返 回 值: Velocity
**********************************************************************************************************/
float GetFPSWayPointNav(void){
  return ReferenceRouteData.CurrentTime;
}
