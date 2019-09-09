#ifndef _MPU6050_H_
#define _MPU6050_H_
#include "stm32f10x.h"


#define I2C_Speed 						400000
#define I2C2_MPU6050					0xd0


/***********�궨��MPU6050�ڲ��Ĵ�����ַ****************/
#define	SMPLRT_DIV					0x19	//�����ǲ����ʣ�����ֵ��0x07(125Hz)
#define	CONFIG							0x1A	//��ͨ�˲�Ƶ�ʣ�����ֵ��0x06(5Hz)
#define	GYRO_CONFIG					0x1B	//�������Լ켰������Χ������ֵ��0x18(���Լ죬2000deg/s)
#define	ACCEL_CONFIG				0x1C	//���ټ��Լ졢������Χ����ͨ�˲�Ƶ�ʣ�����ֵ��0x01(���Լ죬2G��5Hz)
#define	ACCEL_XOUT_H				0x3B
#define	ACCEL_XOUT_L				0x3C
#define	ACCEL_YOUT_H				0x3D
#define	ACCEL_YOUT_L				0x3E
#define	ACCEL_ZOUT_H				0x3F
#define	ACCEL_ZOUT_L				0x40
#define	TEMP_OUT_H					0x41
#define	TEMP_OUT_L					0x42
#define	GYRO_XOUT_H					0x43
#define	GYRO_XOUT_L					0x44	
#define	GYRO_YOUT_H					0x45
#define	GYRO_YOUT_L					0x46
#define	GYRO_ZOUT_H					0x47
#define	GYRO_ZOUT_L					0x48
#define	PWR_MGMT_1					0x6B	//��Դ��������ֵ��0x00(��������)
#define	WHO_AM_I						0x75	//IIC��ַ�Ĵ���(Ĭ����ֵ0x68��ֻ��)

void I2C_Configuration(void);
void MPU6050_Init(void);

void I2C_WriteByte(uint8_t Write_Data, uint8_t REG_Address);
void I2C_ReadBuffer(uint8_t* Data_Buffer, uint8_t REG_Adress, uint8_t Num_Byte);
void I2C_WaitEepromStandbyState(void);      
uint8_t I2C_ReadByte(uint8_t REG_Address);
void Angle_Calculate(void);
#endif

