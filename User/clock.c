#include "clock.h"
#include "USART3.h"
#include "sys.h" 
#define send2pc 1

int32_t clock_cnt = 0;
extern int Remote_flag;
int get_yaw_flag=0;
extern char odm[8];
float enable_getyaw;
extern  u8 rev_flag;

void TIM4_Int_Init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);  
	
  TIM_TimeBaseInitStructure.TIM_Period = 10-1; 	//1ms ���ж�һ��
	TIM_TimeBaseInitStructure.TIM_Prescaler=8400-1;  
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseInitStructure);
	
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE); 
	TIM_Cmd(TIM4,ENABLE); 
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0); 
	NVIC_InitStructure.NVIC_IRQChannel=TIM4_IRQn; 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0; 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=2; 
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void TIM4_IRQHandler(void)
{
 static int i=0,count=0,led_counter=0;

	if(TIM_GetITStatus(TIM4,TIM_IT_Update)==SET) 
	{
			clock_cnt++;	
			count++;
		  led_counter++;
			enable_getyaw=Get_Time_Micros();
			enable_getyaw=	enable_getyaw*1.0e-6;//�������
			
		  if( (led_counter >= 1000)&&(rev_flag==0))//û�н��յ�A1 A2 A3������ɫ
				{	
					led_counter=0;
					PCout(1)=1;
					PCout(2)=~PCout(2);
				}
	
			if( (clock_cnt >= 1000)&&(rev_flag==1))//���ܵ�	A1 A2 A3 ����ɫ
				{	
				clock_cnt=0;
				PCout(2)=1;
				PCout(1)=~PCout(1); 
				}
				
			if(enable_getyaw>6)
				{
				get_yaw_flag=1;//�ȴ�yaw�Ƕ��������ȶ�
			#if send2pc
				if((count >=5)&&(rev_flag==1)) //����λ������odm��Ƶ�ʣ�5*1��,���ڵ��Ե�ʱ��ر�
				{
					count=0;
				send_odm_msg();
				 }
		 #else
				if(count >=1) //����λ������odm��Ƶ��,���ڵ��Ե�ʱ��رգ�200ms����һ��	
				{
					count=0;
					Data_Exchange()	;//��λ�����ͳ���
				 }
			#endif						
			}
	
		if((get_yaw_flag)&&(Remote_flag))//���ܵ�ң�����Լ���ʼ��������õ����˶�
			{
				if((RC_Ctl.rc.s1==1)&&(RC_Ctl.rc.s2==1))		Usart_Chassis_Control(1,liner.x,liner.y,angular.z); //�ж���λ��/ң��������ģʽ
				else	Remote_Chassis_Control(1);
				Wheel_Speed_control(1);
			}
			
		if ((RC_Ctl.rc.s1==2)&&(RC_Ctl.rc.s2==2)){Wheel_Speed_control(0);}//ֹͣ���
}
	  TIM_ClearITPendingBit(TIM4,TIM_IT_Update);  
}



void TIM2_Configuration(void)
{
    TIM_TimeBaseInitTypeDef tim;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
    tim.TIM_Period = 0xFFFFFFFF;
    tim.TIM_Prescaler = 84 - 1;	 //1M ��ʱ��  
    tim.TIM_ClockDivision = TIM_CKD_DIV1;	
    tim.TIM_CounterMode = TIM_CounterMode_Up;  
    TIM_ARRPreloadConfig(TIM2, ENABLE);	
    TIM_TimeBaseInit(TIM2, &tim);

    TIM_Cmd(TIM2,ENABLE);	
}
   

int32_t ms_count = 0;
uint32_t Get_Time_Micros(void)
{
	return TIM2->CNT;
}