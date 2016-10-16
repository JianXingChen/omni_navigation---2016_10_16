#include "Chassis_Control.h"
#include "Wheel_Speed.h"
#include "Remote.h"
#include "imu.h"

#define SPEED_MAX 100
#define SPEED_MAX2 350
#define SPEED_K 0.020f
#define l 0.4//��λ��m���������
extern	int get_yaw_flag;	
struct Chassis_Info Chassis_Control_Info;
struct PID_Info Chasis_Angular;

float AmplitudeCheck(float Angle , float limit)//�޷�
{
	if(Angle>limit)  Angle=limit;
	if(Angle<-limit)  Angle=-limit;
	return Angle;
}

float yaw_back(void )
{
	static int count=0;
	float yaw_int;
	if(get_yaw_flag)
		{ 
			if(count==0)
			{	
				yaw_int=yaw_angle;//��¼�ȶ������ĵ�һ���Ƕȣ���Ϊ��׼��
			  count++;
			}
			Chassis_Control_Info.Chassis_angle=yaw_angle-yaw_int;
			if(abs(Chassis_Control_Info.Chassis_angle)<1.5) Chassis_Control_Info.Chassis_angle=0;//������ֹʱ��Ķ���
		}
	else Chassis_Control_Info.Chassis_angle=0;
	return Chassis_Control_Info.Chassis_angle;
}


int Usart_Chassis_Control(uint8_t flag,float liner_x,float liner_y,float angular_z)//���뷶Χ�ǡ�100rpm,��λrpm
{
	uint8_t i;
	float yawback;
	float	direction[4]={0,0,0,0};//���1�����2�����3�����4	


	direction[0]=(-liner_x+liner_y)+angular_z;
	direction[1]=(liner_x+liner_y)+angular_z;
	direction[2]=(liner_x-liner_y)+angular_z;
	direction[3]=(-liner_x-liner_y)+angular_z;
	if( flag != 0 )
	{
		for(i=0;i<4;i++)//�޷����
		{
			Four_Wheel_Info.Target_speed[i] = AmplitudeCheck( direction[i] , SPEED_MAX);	
		}	
	}		
	else
	{
		for(i=0;i<4;i++)//�޷����
		{
			Four_Wheel_Info.Target_speed[i] = 0;	
		}	
	}	
	return 1;
}



int8_t Remote_Chassis_Control(uint8_t flag)//���뷶Χ�ǡ�400rpm
{

	uint8_t i;
  float  ky=5;     
	float yawback=0,EK0=0;
	short	direction[4]={0,0,0,0};//���1�����2�����3�����4		
	
  short	    f0=0.56*(RC_Ctl.rc.ch1-1024),
				    r0=0.56*(RC_Ctl.rc.ch0-1024);
	Chasis_Angular.Target=(RC_Ctl.rc.ch2-1024)/10; 
	

  Chasis_Angular.Target += Chasis_Angular.Target;//ң��������������ʽ
	Chasis_Angular.Target=AmplitudeCheck(Chasis_Angular.Target,180);
  Chassis_Control_Info.Chassis_angle=yaw_back();	

	Chasis_Angular.Ek=(Chasis_Angular.Target-Chassis_Control_Info.Chassis_angle);
	Chasis_Angular.P_Part  = Chasis_Angular.Ek/100;
	Chasis_Angular.I_Part += Chasis_Angular.Ek;
	Chasis_Angular.D_Part  = Chasis_Angular.Ek-Chasis_Angular.Ek1;

  Chasis_Angular.Out=  Chassis_para.shell_P * Chasis_Angular.P_Part
									   + Chassis_para.shell_I * Chasis_Angular.I_Part
                     + Chassis_para.shell_D *	Chasis_Angular.D_Part ;//λ��ʽpid
  Chasis_Angular.Ek1=Chasis_Angular.Ek;
	
//	direction[0]=(-f0+r0)+ Chasis_Angular.Out;
//	direction[1]=(f0+r0) + Chasis_Angular.Out;
//	direction[2]=(f0-r0) + Chasis_Angular.Out;
//	direction[3]=(-f0-r0)+ Chasis_Angular.Out;
	
	direction[0]=(-f0+r0);
	direction[1]=(f0+r0) ;
	direction[2]=(f0-r0) ;
	direction[3]=(-f0-r0);
	if( flag != 0 )
	{
		for(i=0;i<4;i++)//�޷����
		{
			Four_Wheel_Info.Target_speed[i] = AmplitudeCheck( direction[i] , SPEED_MAX2);	
		}	
	}		
	else
	{
		for(i=0;i<4;i++)//�޷����
		{
			Four_Wheel_Info.Target_speed[i] = 0;	
		}	
	}	
	return 1;
}