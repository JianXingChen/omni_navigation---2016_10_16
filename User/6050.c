#include "6050.h"
#include "delay.h"
#include "Motor_out.h"
#include "Wheel_Speed.h"


#define uint8_t u8
#define uint16_t u16

struct _sensor sensor;	
struct _sensor sensor2;



u8	mpu6050_buffer[16];					


/***************************************************************************************
  * @����������  MPU6050��ʼ��
  * @��ڲ�����  ��.
  * @����ֵ  :   ��ʼ����ɱ�־��0----��ɣ�!0----δ���.
****************************************************************************************/
u8 InitMPU6050(void)
{
	unsigned char Addr;
	u8 date;
	do
	{
		
		Holder_Motor_output(0);//���Ϊ0�����͸���
		Wheel_Speed_control(0);
		
		date = Single_Write(MPU6050_ADDRESS, PWR_MGMT_1, 0x01);  	   //����
		delay_ms(500);
		date = Single_Write(MPU6050_ADDRESS, PWR_MGMT_1, 0x00);  	   //�������״̬0x00
		delay_ms(50);
		
		Addr=Single_Read(MPU6050_ADDRESS,WHO_AM_I); //count++;
		                                  
	}while(Addr!=0x68);
	

	
	
	//��ʼ��MPU6050 1
//	date = Single_Write(MPU6050_ADDRESS, PWR_MGMT_1, 0x01);  	   //����
//	delay_ms(500);
//	
	
	date = Single_Write(MPU6050_ADDRESS, PWR_MGMT_1, 0x00);  	   //�������״̬0x00
	delay_ms(50);
	date += Single_Write(MPU6050_ADDRESS, SMPLRT_DIV, 0x00);     //����Ƶ�ʣ�1KHz��
	delay_ms(50);
	date += Single_Write(MPU6050_ADDRESS, CONFIG, 0x01);         //��ͨ�˲�0x00
	delay_ms(50);
	date += Single_Write(MPU6050_ADDRESS, GYRO_CONFIG, 0x10);    //���������� 
	delay_ms(50);
	date += Single_Write(MPU6050_ADDRESS, ACCEL_CONFIG, 0x09);   //���ٶ����� 
	delay_ms(50);
	

	//��ʼ��MPU6050 2
//	date = Single_Write(MPU6050_ADDRESS2, PWR_MGMT_1, 0x01);  	   //�������״̬0x00
//	delay_ms(500);
//	
//	
//	date = Single_Write(MPU6050_ADDRESS2, PWR_MGMT_1, 0x00);  	   //�������״̬0x00
//	delay_ms(50);
//	date += Single_Write(MPU6050_ADDRESS2, SMPLRT_DIV, 0x00);     //����Ƶ�ʣ�1KHz��
//	delay_ms(50);
//	date += Single_Write(MPU6050_ADDRESS2, CONFIG, 0x01);         //��ͨ�˲�0x00
//	delay_ms(50);
//	date += Single_Write(MPU6050_ADDRESS2, GYRO_CONFIG, 0x10);    //���������� 
//	delay_ms(50);
//	date += Single_Write(MPU6050_ADDRESS2, ACCEL_CONFIG, 0x09);   //���ٶ����� 
//	delay_ms(50);
	
	
	
	return date;
}
/***************************************************************************************
  * @����������  ��ȡ���м��ٶȼơ�������
  * @��ڲ�����  ��.
  * @����ֵ  :   ��.
****************************************************************************************/
void MPU6050_Read(void)
{
	mpu6050_buffer[0]=Single_Read(MPU6050_ADDRESS, ACCEL_XOUT_H);
	mpu6050_buffer[1]=Single_Read(MPU6050_ADDRESS, ACCEL_XOUT_L);
	
	
	mpu6050_buffer[2]=Single_Read(MPU6050_ADDRESS, ACCEL_YOUT_H);
	mpu6050_buffer[3]=Single_Read(MPU6050_ADDRESS, ACCEL_YOUT_L);
	
	mpu6050_buffer[4]=Single_Read(MPU6050_ADDRESS, ACCEL_ZOUT_H);
	mpu6050_buffer[5]=Single_Read(MPU6050_ADDRESS, ACCEL_ZOUT_L);
	
	mpu6050_buffer[8]=Single_Read(MPU6050_ADDRESS, GYRO_XOUT_H);
	mpu6050_buffer[9]=Single_Read(MPU6050_ADDRESS, GYRO_XOUT_L);
	
	
	mpu6050_buffer[10]=Single_Read(MPU6050_ADDRESS, GYRO_YOUT_H);
	mpu6050_buffer[11]=Single_Read(MPU6050_ADDRESS, GYRO_YOUT_L);
	
	
	mpu6050_buffer[12]=Single_Read(MPU6050_ADDRESS, GYRO_ZOUT_H);//������2��z��������
	mpu6050_buffer[13]=Single_Read(MPU6050_ADDRESS, GYRO_ZOUT_L);//
	
	//mpu6050_buffer[14]=Single_Read(MPU6050_ADDRESS2, GYRO_ZOUT_H);//������2��z��������
	//mpu6050_buffer[15]=Single_Read(MPU6050_ADDRESS2, GYRO_ZOUT_L);//
	
	
	
	
}


/***************************************************************************************
  * @����������  ���������У׼
  * @��ڲ�����  ��.
  * @����ֵ  :   ��.
****************************************************************************************/

void Gyro_OFFEST(void)
{
   int cnt_g=1000;
	 int cnt = cnt_g;
	 float  tempgx=0,tempgy=0,tempgz , tempgz2=0;
	
	
	 sensor.gyro.averag.x=0;    //���ƫ������
	 sensor.gyro.averag.y=0;  
	 sensor.gyro.averag.z=0;
	 sensor2.gyro.averag.z=0;
	
	 while(cnt_g--)       //ѭ���ɼ�1000��   ��ƽ��
	 {
		  MPU6050_Read();
		 
		  sensor.gyro.origin.x = ((((int16_t)mpu6050_buffer[8]) << 8) | mpu6050_buffer[9]);
	    sensor.gyro.origin.y = ((((int16_t)mpu6050_buffer[10]) << 8)| mpu6050_buffer[11]);
	    sensor.gyro.origin.z = ((((int16_t)mpu6050_buffer[12]) << 8)| mpu6050_buffer[13]);
		  sensor2.gyro.origin.z = ((((int16_t)mpu6050_buffer[14]) << 8)| mpu6050_buffer[15]);
		 
		 
      tempgx+= sensor.gyro.origin.x;
			tempgy+= sensor.gyro.origin.y;
			tempgz+= sensor.gyro.origin.z;
		 
		  tempgz2+= sensor2.gyro.origin.z;
		 
   }
	 sensor.gyro.quiet.x=tempgx/cnt;
	 sensor.gyro.quiet.y=tempgy/cnt;
	 sensor.gyro.quiet.z=tempgz/cnt;
	 
	 sensor2.gyro.quiet.z=tempgz2/cnt;
	 
}




