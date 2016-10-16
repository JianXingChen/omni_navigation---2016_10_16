
#include "PID.h"
#include "USART3.h"

/***************************************************************************************
  * @����������  �޷�.
  * @��ڲ�����  ��.
  * @����ֵ  :   ��.
****************************************************************************************/
float AmplitudeCheck(float Angle)
{
	if(Angle>4900)  
		Angle=4900;
	if(Angle<-4900)  
		Angle=-4900;
	return Angle;
}

/***************************************************************************************
  * @����������  PID������.
  * @��ڲ�����  FeedBack----����ֵ��Target----����ֵ��PID----������PID�ṹ�壬
                 Inner----1,�ڻ���0���⻷.
  * @����ֵ  :   ��.
****************************************************************************************/
void PID_PosControl(float FeedBack,float Target,struct PID_InitTypeDef *PID)
{
//	if(Inner==1)
//	{
//		PID->Kp = Pitch_para.core_P/1000; //   0.52;
//		PID->Ki = Pitch_para.core_I/1000;//  0;
//		PID->Kd = Pitch_para.core_D/1000;//  0;
//	}
//	else if(Inner==0)
//	{
//		PID->Kp=-Pitch_para.shell_P/10;//    120;
//		PID->Ki=-Pitch_para.shell_I/1000;//  0;
//		PID->Kd=-Pitch_para.shell_D/1000;//  0;
//	}
//	else ;
	
	PID->Bias = Target - FeedBack;
/*--------------------------  ����ʽPID  --------------------*/
//	PID->Out += PID->Kp * (PID->Bias - PID->Last_Bias) +	PID->Ki * PID->Bias + PID->Kd *(PID->Bias - 2*PID->Last_Bias + PID->Pre_Bias);
//	PID->Pre_Bias = PID->Last_Bias;
/*-------------------------  ����ʽPD  ---------------------*/
	PID->Out = PID->Kp * PID->Bias;
	
	PID->Out = AmplitudeCheck(PID->Out);
}

/***************************************************************************************
  * @����������  PID�ٶȴ�����.
  * @��ڲ�����  FeBack----����ֵ��SET----����ֵ��PID----������PID�ṹ�壬
  * @����ֵ  :   ��.
****************************************************************************************/
void  PID_SpeControl(int FeedBack,int Target,struct PID_WheelTypeDef *PID)
{
	
	PID->Bias = Target - FeedBack;
	PID->Out += PID->Kp * (PID->Bias - PID->Last_Bias) +	PID->Ki * PID->Bias;
	PID->Last_Bias = PID->Bias;
	
	AmplitudeCheck(PID->Out);
	
}




