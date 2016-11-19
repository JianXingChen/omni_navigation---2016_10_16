#include "Wheel_Speed.h"
#include "Chassis_Control.h"
#define pi 3.1415926;

CanTxMsg TxMessage;

float delta[4]={0} ;
extern	int get_yaw_flag;	
struct Wheel_Info Four_Wheel_Info;
extern struct PID_PARA_Temp Pid_temp;
struct PID_Info Wheel_Speed;
struct PID_PARA Chassis_para = //������������⻷�ڻ����� 
{
  12000,5,100,//�ڻ��ٶ�
	10000,0,0     //�⻷�Ƕ�
};

				

uint8_t Wheel_Speed_control(uint8_t flag)
{


	static int8_t num = 0;
	int8_t i,wheel_cnt;
	float sum[4];

	/***********NUM1 �����ĸ����ӵĵ�����*************/
	for( wheel_cnt = 0;wheel_cnt<4;wheel_cnt++ )
	{		
		/*(2)����PID*/
	  Wheel_Speed.Ek = Four_Wheel_Info.Target_speed[wheel_cnt] - Four_Wheel_Info.speed_raw[wheel_cnt];
//			  Wheel_Speed.Ek = 50- Four_Wheel_Info.speed_raw[wheel_cnt];
		delta[wheel_cnt]=Wheel_Speed.Ek;//�����۲��ֵ
		
		Wheel_Speed.P_Part  = Wheel_Speed.Ek /100;//P PART ��С10��
	  Wheel_Speed.I_Part += Wheel_Speed.Ek ;
		Wheel_Speed.D_Part  = Wheel_Speed.Ek-Wheel_Speed.Ek1;
		
		Four_Wheel_Info.out[wheel_cnt] = Chassis_para.core_P * Wheel_Speed.P_Part 
		                               + Chassis_para.core_I * Wheel_Speed.I_Part/10000
																	 + Chassis_para.core_D * Wheel_Speed.D_Part/100;

		
		Wheel_Speed.Ek1=Wheel_Speed.Ek;
		
		Four_Wheel_Info.out[wheel_cnt]=AmplitudeCheck(Four_Wheel_Info.out[wheel_cnt],4000 );
	}
		//��������
		if( flag == DISABLE_MOTOR_OUT)	Wheel_out( NO_OUT , Four_Wheel_Info.out);
		else 	Wheel_out( ALL_OUT , Four_Wheel_Info.out);
		return 1;
}

/*****************************************************
 * *��������Wheel_out
 * *����:      �����ĸ����̵�������
 * *����:      out_mode
		��ѡ������NO_OUT  : �޵�����ģʽ
**************ALL_OUT  :���е�����ģʽ
**************SIGNAL_OUT	:����������ģʽ
**���:��

 ****************************************************/	 
void Wheel_out(uint8_t out_mode , float * speed_list )
{
	int16_t i16_speed_list[4];
	int8_t i;
	uint8_t TransmitMailbox;
	uint16_t t;

	TxMessage.StdId = 0x200;	
	TxMessage.IDE=CAN_ID_STD;					 //STANDARD MODE
  TxMessage.RTR=CAN_RTR_DATA;				 //���͵�������
  TxMessage.DLC=8;							 //���ݳ���Ϊ2�ֽ�

	if( out_mode == ALL_OUT )//���е�����ģʽ
	{
		for( i=0;i<4;i++ )
		{
			i16_speed_list[i] = (int16_t)(*(speed_list+i));
			TxMessage.Data[i*2+0] = (uint8_t)(i16_speed_list[i]>>8);
			TxMessage.Data[i*2+1] = (uint8_t)(i16_speed_list[i]);
		}
	}
	else if(out_mode==SIGNAL_OUT) 
	{
		TxMessage.Data[0]=0>>8;
		TxMessage.Data[1]=0;
		TxMessage.Data[2]=0>>8;
		TxMessage.Data[3]=0;
		TxMessage.Data[4]=0>>8;
		TxMessage.Data[5]=0;
		TxMessage.Data[6]=0;
		TxMessage.Data[7]=0;
	}
	else if(out_mode==NO_OUT)	
	{
		TxMessage.Data[0]=0>>8;
		TxMessage.Data[1]=0;
		TxMessage.Data[2]=0>>8;
		TxMessage.Data[3]=0;
		TxMessage.Data[4]=0>>8;
		TxMessage.Data[5]=0;
		TxMessage.Data[6]=0;
		TxMessage.Data[7]=0;
	}
	TransmitMailbox=CAN_Transmit(CAN2,&TxMessage);
	t=0;
	while((CAN_TransmitStatus(CAN2,TransmitMailbox)!=CANTXOK)&&(t!= 0xFF))
	{
		t++;
	}
}

