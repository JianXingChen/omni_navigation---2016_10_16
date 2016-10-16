
#ifndef __MPU6050_H_
#define __MPU6050_H_
#include<stdint.h>
#include "stm32f4xx.h" 
#include "I2C.h"

//****************************************
// ����MPU6050�ڲ���ַ
//****************************************

#define	SMPLRT_DIV		0x19	//�����ǲ����ʣ�����ֵ��0x07(125Hz)
#define	CONFIG			  0x1A	//��ͨ�˲�Ƶ�ʣ�����ֵ��0x06(5Hz)
#define	GYRO_CONFIG		0x1B	//�������Լ켰������Χ������ֵ��0x18(���Լ죬2000deg/s)
#define	ACCEL_CONFIG	0x1C	//���ټ��Լ졢������Χ����ͨ�˲�Ƶ�ʣ�����ֵ��0x01(���Լ죬2G��5Hz)

#define	ACCEL_XOUT_H	0x3B
#define	ACCEL_XOUT_L	0x3C
#define	ACCEL_YOUT_H	0x3D
#define	ACCEL_YOUT_L	0x3E
#define	ACCEL_ZOUT_H	0x3F
#define	ACCEL_ZOUT_L	0x40

#define	TEMP_OUT_H		0x41
#define	TEMP_OUT_L		0x42

#define	GYRO_XOUT_H		0x43
#define	GYRO_XOUT_L		0x44	
#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46
#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48

#define MAG_XOUT_L		0x03
#define MAG_XOUT_H		0x04
#define MAG_YOUT_L		0x05
#define MAG_YOUT_H		0x06
#define MAG_ZOUT_L		0x07
#define MAG_ZOUT_H		0x08

#define	PWR_MGMT_1		0x6B	 //��Դ��������ֵ��0x00(��������)
#define	WHO_AM_I		  0x75	   //IIC��ַ�Ĵ���(Ĭ����ֵ0x68��ֻ��)

#define	MPU6050_ADDRESS  0xD0	  //���ݵ�ַ
#define MAG_ADDRESS    0x18       //�ų���ַ
#define ACCEL_ADDRESS  0xD0 

#define	MPU6050_ADDRESS2  0xD2	 


#define	GYRO_Y		0x45
#define	GYRO_Z		0x47


typedef struct{
				float X;
				float Y;
				float Z;}FLOAT_XYZ;

struct _float{
	      float x;
				float y;
				float z;};

struct _int16{
       int16_t x;
	     int16_t y;
	     int16_t z;};		

struct _trans{
     struct _int16 origin;  //ԭʼֵ
	   struct _int16 averag;  //ƽ��ֵ
	   struct _int16 histor;  //��ʷֵ
	   struct _int16 quiet;   //��ֵ̬
	   struct _float radian;  //����ֵ 
          };

struct _sensor{   
	struct _trans acc;
	struct _trans gyro;
              };

extern struct _sensor sensor;					

extern u8		 mpu6050_buffer[14];					
							
u8 InitMPU6050(void);					   
void Gyro_OFFEST(void);													
							
void Get_Gyro_data(float *G_X,float *G_Z);
#endif // __MPU6050_H__
