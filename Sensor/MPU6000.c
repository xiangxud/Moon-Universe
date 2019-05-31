/**********************************************************************************************************
 * @文件     MPU6000.c
 * @说明     MPU6000读取函数
 * @作者     YuyingJin
 * @网站     https://yuyingjin0111.github.io/
 * @日期     2018 ~
**********************************************************************************************************/
#include "MPU6000.h"

/***********************************************************************************
*函 数 名: MPU6000_writeReg
*功能说明: spi读写MPU6000
*形    参: 
*返 回 值: 状态
************************************************************************************/
void MPU6000_writeReg(u8 reg, u8 data)
{
	MPU6000_CSL();
	SPI1_ReadWrite_Byte(reg);
	SPI1_ReadWrite_Byte(data);
	MPU6000_CSH();	
}
/***********************************************************************************
*函 数 名: MPU6000_Initialize
*功能说明: 初始化MPU6000
*形    参: 
*返 回 值: 状态
************************************************************************************/
void MPU6000_Initialize(void){ 
	MPU6000_writeReg(MPU_RA_PWR_MGMT_1, BIT_H_RESET);
	delay_ms(150);

	MPU6000_writeReg(MPU_RA_SIGNAL_PATH_RESET, BIT_GYRO | BIT_ACC | BIT_TEMP);
	delay_ms(150);

	MPU6000_writeReg(MPU_RA_PWR_MGMT_1, MPU_CLK_SEL_PLLGYROZ);
	delay_ms(15);

	MPU6000_writeReg(MPU_RA_USER_CTRL, BIT_I2C_IF_DIS);
	delay_ms(15);

	MPU6000_writeReg(MPU_RA_PWR_MGMT_2, 0x00);
	delay_ms(15);

	//陀螺仪采样率0x00(1000Hz)   采样率 = 陀螺仪的输出率 / (1 + SMPLRT_DIV)
	MPU6000_writeReg(MPU_RA_SMPLRT_DIV, (1000/1000 - 1));
	delay_ms(15);
	
	//陀螺仪自检及测量范围，典型值0x18(不自检，2000deg/s) (0x10 1000deg/s) (0x10 1000deg/s) (0x08 500deg/s)
	MPU6000_writeReg(MPU_RA_GYRO_CONFIG, 0x10);
	delay_ms(15);

	//加速度自检、测量范围(不自检，+-8G)
	MPU6000_writeReg(MPU_RA_ACCEL_CONFIG, 2 << 3);

	//i2c旁路模式
	// INT_PIN_CFG   -- INT_LEVEL_HIGH, INT_OPEN_DIS, LATCH_INT_DIS, INT_RD_CLEAR_DIS, FSYNC_INT_LEVEL_HIGH, FSYNC_INT_DIS, I2C_BYPASS_EN, CLOCK_DIS
	MPU6000_writeReg(MPU_RA_INT_PIN_CFG, 0 << 7 | 0 << 6 | 0 << 5 | 1 << 4 | 0 << 3 | 0 << 2 | 0 << 1 | 0 << 0);
	delay_ms(15);

//	//低通滤波频率 当有很大的噪声时才需要打开
//	MPU6000_writeReg(MPU_RA_CONFIG, MPU_LPF_188HZ);
//	delay_ms(15);

	delay_ms(150);
}
/**********************************************************************************************************
*函 数 名: MPU6000_ReadAcc
*功能说明: MPU6000读取加速度陀螺仪传感器，并转化为标准单位
*形    参: 读出数据指针
*返 回 值: 无
**********************************************************************************************************/
void MPU6000_ReadAccGyro(Vector3f_t *acc ,Vector3f_t *gyro){
	static uint8_t buffer[14];
	static Vector3i_t accRaw,gyroRaw;
	MPU6000_CSL();
	SPI1_readRegs(MPU_RA_ACCEL_XOUT_H,14,buffer);
	MPU6000_CSH();
	//acc
	accRaw.x = ((((int16_t)buffer[0]) << 8) | buffer[1]);
	accRaw.y = ((((int16_t)buffer[2]) << 8) | buffer[3]);
	accRaw.z = ((((int16_t)buffer[4]) << 8) | buffer[5]);
	//gyro
	gyroRaw.x = ((((int16_t)buffer[8]) << 8) | buffer[9]);
  gyroRaw.y = ((((int16_t)buffer[10]) << 8) | buffer[11]);
  gyroRaw.z = ((((int16_t)buffer[12]) << 8) | buffer[13]);

	acc->x = ((float)accRaw.x) * MPU_A_8mg;
	acc->y = ((float)accRaw.y) * MPU_A_8mg;
	acc->z = ((float)accRaw.z) * MPU_A_8mg;
	
	gyro->x = ((float)gyroRaw.x) * MPU_G_s1000dps * PI/180;
	gyro->y = ((float)gyroRaw.y) * MPU_G_s1000dps * PI/180;
	gyro->z = ((float)gyroRaw.z) * MPU_G_s1000dps * PI/180;
	
}
///******************************************************************************************************
//*函 数 名: MPU6000_ReadTemp
//*功能说明: MPU6000读取温度传感器
//*形    参: 读出数据指针
//*返 回 值: 无
//*******************************************************************************************************/
//void MPU6000_ReadTemp(float* temp)
//{
//    uint8_t buffer[2];
//    static int16_t temperature_temp;
//		MPU6000_CSL();
//		SPI1_readRegs(MPU_RA_TEMP_OUT_H,2,buffer);
//		MPU6000_CSH();
//    temperature_temp = ((((int16_t)buffer[0]) << 8) | buffer[1]);
//    *temp = 36.53f + (float)temperature_temp / 340;
//}



