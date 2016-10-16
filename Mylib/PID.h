#include "stm32f4xx.h" 
#include <stdio.h>

struct PID_InitTypeDef      
{
	float Kp;               //Kp����
	float Ki;               //Ki����
	float Kd;
	float Bias;             //�������
	float Last_Bias;  
	
	float Target;       //Ŀ��
	int16_t Out;          //���
};

struct PID_WheelTypeDef      
{
	float Kp;               //Kp����
	float Ki;               //Ki����
	float Bias;             //�������
	float Last_Bias;  
	int Enc_New;
	int Enc_Old;
	int Increment;
	
	int Target;
	int16_t Out;          //���
};

void  PID_PosControl(float FeedBack,float Target,struct PID_InitTypeDef *PID);
void  PID_SpeControl(int FeedBack,int Target,struct PID_WheelTypeDef *PID);

