
#include "mpu6050.h"
#include "delay.h"

struct _sensor sensor;	
	
u8 mpu6050_buffer[14];					

/***************************************************************************************
  * @����������  MPU6050��ʼ��
  * @��ڲ�����  ��.
  * @����ֵ  :   ��ʼ����ɱ�־��0----��ɣ�!0----δ���.
****************************************************************************************/
u8 InitMPU6050(void)
{
	unsigned char Addr;
	u8 date,count = 0;
	do
	{
		Addr=Single_Read(MPU6050_ADDRESS,WHO_AM_I); count++;
		if(count>8) return 0;                                      //MPU6050����ʧ��
	}while(Addr!=0x68);
	delay_ms(2);
	
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
	
//	count = 0;
//	do
//	{
//		Addr=Single_Read(MPU6050_ADDRESS2,WHO_AM_I); count++;
//		if(count>8) return 0;                                      //MPU6050����ʧ��
//	}while(Addr!=0x68);
//	date += Single_Write(MPU6050_ADDRESS, PWR_MGMT_1, 0x01);  	   //�������״̬
//	date += Single_Write(MPU6050_ADDRESS, SMPLRT_DIV, 0x00);     //����Ƶ�ʣ�1KHz��
//	date += Single_Write(MPU6050_ADDRESS, CONFIG, 0x01);         //��ͨ�˲�
//	date += Single_Write(MPU6050_ADDRESS, GYRO_CONFIG, 0x08);    //���������� 
//	date += Single_Write(MPU6050_ADDRESS, ACCEL_CONFIG, 0x08);   //���ٶ����� 
	
	return date;
}
/***************************************************************************************
  * @����������  ��ȡ���м��ٶȼơ�������
  * @��ڲ�����  ��.
  * @����ֵ  :   ��.
****************************************************************************************/
void MPU6050_Read(void)
{
	
	mpu6050_buffer[8]=Single_Read(MPU6050_ADDRESS, GYRO_XOUT_H);
	mpu6050_buffer[9]=Single_Read(MPU6050_ADDRESS, GYRO_XOUT_L);
	
	
	mpu6050_buffer[10]=Single_Read(MPU6050_ADDRESS, GYRO_YOUT_H);
	mpu6050_buffer[11]=Single_Read(MPU6050_ADDRESS, GYRO_YOUT_L);
	
	mpu6050_buffer[12]=Single_Read(MPU6050_ADDRESS, GYRO_ZOUT_H);
	mpu6050_buffer[13]=Single_Read(MPU6050_ADDRESS, GYRO_ZOUT_L);
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
	 float  tempgx=0,tempgy=0,tempgz=0;
	 sensor.gyro.averag.x=0;    //���ƫ������
	 sensor.gyro.averag.y=0;  
	 sensor.gyro.averag.z=0;
	 while(cnt_g--)       //ѭ���ɼ�1000��   ��ƽ��
	 {
		  MPU6050_Read();
		 
		  sensor.gyro.origin.x = ((((int16_t)mpu6050_buffer[8]) << 8) | mpu6050_buffer[9]);
	    sensor.gyro.origin.y = ((((int16_t)mpu6050_buffer[10]) << 8)| mpu6050_buffer[11]);
	    sensor.gyro.origin.z = ((((int16_t)mpu6050_buffer[12]) << 8)| mpu6050_buffer[13]);
      tempgx+= sensor.gyro.origin.x;
			tempgy+= sensor.gyro.origin.y;
			tempgz+= sensor.gyro.origin.z;
   }
	 sensor.gyro.quiet.x=tempgx/cnt;
	 sensor.gyro.quiet.y=tempgy/cnt;
	 sensor.gyro.quiet.z=tempgz/cnt;
	
}
/***************************************************************************************
  * @����������  ��ȡ������ԭʼ����
  * @��ڲ�����  ����ȡ��.
  * @����ֵ  :   ��ǰ���ٶ�.
****************************************************************************************/
void Get_Gyro_data(float *G_Y,float *G_Z)
{
		int i;  float sum=0;
		static float GyroY_data[4]={0,0,0,0},GyroZ_data[4]={0,0,0,0};
	
		MPU6050_Read();
		sensor.gyro.origin.x  = ((((int16_t)mpu6050_buffer[8]) << 8) | mpu6050_buffer[9]) - sensor.gyro.quiet.x;
		sensor.gyro.origin.y  = ((((int16_t)mpu6050_buffer[10]) << 8)| mpu6050_buffer[11]) - 6;
		sensor.gyro.origin.z  = ((((int16_t)mpu6050_buffer[12]) << 8)| mpu6050_buffer[13]) - 62;
		
		if((sensor.gyro.origin.y>-30)&(sensor.gyro.origin.y<30)) sensor.gyro.origin.y=0;
		if((sensor.gyro.origin.z>-30)&(sensor.gyro.origin.z<30)) sensor.gyro.origin.z=0;
		
		for(i=3;i>=1;i--)
		{
			GyroZ_data[i] = GyroZ_data[i-1];
			sum+=GyroZ_data[i];
		}
		GyroZ_data[0] = sensor.gyro.origin.z;
		*G_Z = (sum + GyroZ_data[0]) / 4;
		
		for(i=3;i>=1;i--)
		{
			GyroY_data[i] = GyroY_data[i-1];
			sum+=GyroY_data[i];
		}
		GyroY_data[0] = sensor.gyro.origin.y;
		*G_Y = (sum + GyroY_data[0]) / 4;
		
}
