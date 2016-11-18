
#include "USART3.h"
#include "imu.h"
#include "Wheel_Speed.h"
#define normal 1//���������λ��V4.01ʹ��
#define r 0.076 //�����ķ�ְ뾶����λm
#define pi 3.1415926
extern char odm[8];
/*-----USART3_TX-----PB10-----*/
/*-----USART3_RX-----PB11-----*/
/*****************************************************************
*** ����cmd_vel���⣬������ת��Ϊ������ת��
*** ����odm���⣬�ṩ���Ե���̼���Ϣ
*** 2d�Ļ������ƶ�ƽֻ̨���õ�3��ֵlinear.x,liner.y��angular.z
    �ֱ����ˮƽ���ٶȣ���ֱ���ٶȣ�����ٶ�
	   |____|
		||    ||      ^
		 |    |       |                                          <
	  ||____||      | +x����       <��������  liner.y����            ��  angular.z
		
****	������λ������:��8λ����0λ�����ݴ�С����7λ�Ǽ��顣
****	��1λ��liner.x��8,��2λ�ǵ�8
****  ��3λ��liner.y��8����4λ�ǵ�8
****	��5λ��angular.z��8����6λ�ǵ�8		
*******************************************************************/


u8 com_data;
int head_flag=0,end_flag=0,n=0,put_flag,k=1;
static	int count=0;
u8 para_get[8];
u8 para_rev_tem[8];
extern char odm[8];
u8 rev_flag=0;
extern float mygetqval[9];
u8 data_to_send[0xff];//���ͻ���
u8	RxBuffer[0xff];//���ܻ���
u8 Send_PID1 = 0;//pid1���ͱ�־λ
u8 Send_PID3 = 0;//pid2���ͱ�־λ
geometry_msgs_twist liner;
geometry_msgs_twist  angular;
struct PID_PARA_Temp Pid_temp;
	int _temp;
u8 twist[16]={0};
void USART3_Configuration(int32_t baud)
{
    USART_InitTypeDef usart3;
		GPIO_InitTypeDef  gpio;
    NVIC_InitTypeDef  nvic;

		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_DMA1,ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
		
		GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART3);
		GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3); 
		
		gpio.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
		gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init(GPIOB,&gpio);
		
		usart3.USART_BaudRate = baud;
		usart3.USART_WordLength = USART_WordLength_8b;
		usart3.USART_StopBits = USART_StopBits_1;
		usart3.USART_Parity = USART_Parity_No;
		usart3.USART_Mode = USART_Mode_Tx|USART_Mode_Rx;
    usart3.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		USART_Init(USART3,&usart3);

		USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
		USART_Cmd(USART3,ENABLE);
    
    nvic.NVIC_IRQChannel = USART3_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority = 3;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
}

void USART3_Sends(char *buf1)		  //�ַ������ͺ���
{
		u8 i=0;
		for(i=0;i<16;i++)
		{	USART_SendData(USART3,buf1[i]);  //����һ���ֽ�
			while(USART_GetFlagStatus(USART3,USART_FLAG_TXE)==RESET){};//�ȴ����ͽ���
			}
	}
void USART3_Sendb(u8 k)		         //�ֽڷ��ͺ���
{		 
			USART_SendData(USART3,k);  //����һ���ֽ�
			while(USART_GetFlagStatus(USART3,USART_FLAG_TXE)==RESET){};//�ȴ����ͽ���	
} 


#if  normal
void USART3_IRQHandler(void)//�����ж�
{    static int a=0,b=0,c=0;
			if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)  //�����ж� (���ռĴ����ǿ�) 
			{
			USART_ClearITPendingBit(USART3,USART_IT_RXNE);//����жϱ�־λ
			 com_data =USART_ReceiveData(USART3);
				if(rev_flag==0){
					if(com_data==0xA1) a=1;
					if((com_data==0xA2)&&(a==1)) b=1;
					if((com_data==0xA3)&&(b==1)) 	rev_flag=1;	}//ֻ�н��յ� A1 A2 A3 ���ڲŹ���
			
				if(rev_flag==1)
				{
					if(com_data==0xA0)   head_flag=1;//�ж�֡ͷ
					if((com_data==0x06)&&(head_flag==1))
						{	head_flag=0;
							put_flag=1 ;}
				if(put_flag==1)
					{	para_rev_tem[count]=com_data;	 //��0x06��ʼ�棬����8��
						count++;
						if(count>=8)	
						{count=0;put_flag=0;}//���߸����ݴ���رմ����ݱ�־λ
					}	
				if((para_rev_tem[7]==0xA1)&&(count==0))  
			 { int i;
					 for(i=1;i<7;i++) para_get[i]=para_rev_tem[i]; }//У���8λ��			  
					 liner.tx   =	(para_get[1]*256+para_get[2]);//��λ������10000������
					 liner.ty   =	(para_get[3]*256+para_get[4]);
					 angular.tz =  (para_get[5]*256+para_get[6]);
					 liner.x   =	liner.tx/10000.0f;//��λ������10000������
					 liner.y   =	liner.ty/10000.0f;
					 angular.z =   angular.tz/10000.f;
					 liner.x   =  (liner.x/r)*30.0f/pi;   //���ٶ�ת����rpm     ��λ��v=w/r m/s ->rpm
					 liner.y   =  (liner.y/r)*30.0f/pi; 
					 angular.z =   angular.z *30.0f/pi;		//���ٶ�ת����rpm     ��λ��rad/s ->rpm					
  		} else 
			  {	
					liner.x   = 0;                      
					liner.y   = 0; 
					angular.z = 0;		
       	}
			 	}
		
	}

void send_odm_msg()//���͵�һ���͵��ĸ����ӵ��ٶ�,ע�ⷴ���ٶ�ȡֵ��Χ��8191���Ƕȷ�Χ��1800����λ���ֱ�Ӧ�á�19,��100;
{ 
int i ,j;
	u8 sum;
	for(i=0;i<8;i++) twist[i]=odm[i];

    _temp = (int)(yaw_angle*100);
		twist[8]= BYTE1(_temp);//��8λ
		twist[9]=	BYTE0(_temp);//��8λ
		twist[10]=	0;//��8λ
		twist[11]=	0;//��8λ
		twist[12]=	0;
		twist[13]=	0;
		twist[14]=	0;
		twist[15]=	0;
		twist[14]=	0;
		twist[15]=	0;
		for( j=0;j<16;j++) 	sum += twist[i];
			
		USART3_Sendb(0xb0);
		USART3_Sendb(0x10);
		USART3_Sends(twist);
		USART3_Sendb(0xb1);	
}





#else
/************************************************************
**  ˵��������Ϊ��λ��������λ���ĺ���
**  data_to_send�����ͻ�������
**  Uart3_Put_Buf(data_to_send,_cnt)�����ڷ��ͺ���
**  _cntΪ��Ҫ���͵��ֽ�����
**  BYTE3��BYTE2��BYTE1��BYTE0:��ʾȡĳ�����ĸߵ��ֽڣ�0��ͣ�3��ߡ�
*************************************************************/
void Uart3_Put_Buf(unsigned char *DataToSend , u8 data_num)
{     u8 i;
			for(i=0;i<data_num;i++)    
				{    
				 USART_SendData(USART3, DataToSend[i]);  
				 while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET); //�ȴ����ݷ������
				}
}
//���ͻ�����Ϣ����̬������״̬��
void Data_Send_Status(void)//������̬
{
				u8 _cnt=0;
				u8 sum = 0;
				u8 i;
				vs16 _temp;
				
				data_to_send[_cnt++]=0xAA;
				data_to_send[_cnt++]=0xAA;
				data_to_send[_cnt++]=0x01;
				data_to_send[_cnt++]=0;

				_temp = (int)(roll_angle*100);
				data_to_send[_cnt++]=BYTE1(_temp);
				data_to_send[_cnt++]=BYTE0(_temp);
				_temp = (int)(pitch_angle*100);
				data_to_send[_cnt++]=BYTE1(_temp);
				data_to_send[_cnt++]=BYTE0(_temp);
				_temp = (int)(yaw_angle*100);
				data_to_send[_cnt++]=BYTE1(_temp);
				data_to_send[_cnt++]=BYTE0(_temp);

				data_to_send[_cnt++]=0;
				data_to_send[_cnt++]=0;
				data_to_send[_cnt++]=0;
				data_to_send[_cnt++]=0;
					
				data_to_send[_cnt++]=0xA1;//����
				
				data_to_send[3] = _cnt-4;

				for(i=0;i<_cnt;i++)
				sum += data_to_send[i];
				data_to_send[_cnt++]=sum;

				Uart3_Put_Buf(data_to_send,_cnt);
}

//���ʹ���������
void Data_Send_Sensor(void)//������̬ԭʼ����
{
				u8 _cnt=0;
				u8 sum = 0;
				u8 i;
				
				data_to_send[_cnt++]=0xAA;
				data_to_send[_cnt++]=0xAA;
				data_to_send[_cnt++]=0x02;
				data_to_send[_cnt++]=0;
//				data_to_send[_cnt++]=BYTE1(mygetqval[0]);  //���ٶ�x��8
//				data_to_send[_cnt++]=BYTE0(mygetqval[0]);  //���ٶ�x��8
				data_to_send[_cnt++]=BYTE1(Four_Wheel_Info.speed_raw[0]);  //���ٶ�x��8
				data_to_send[_cnt++]=BYTE0(Four_Wheel_Info.speed_raw[0]);  //���ٶ�x��8	
				data_to_send[_cnt++]=50>>8;
				data_to_send[_cnt++]=50;
//		  data_to_send[_cnt++]=BYTE1(mygetqval[1]);
//			data_to_send[_cnt++]=BYTE0(mygetqval[1]);
//	
				data_to_send[_cnt++]=BYTE1(mygetqval[2]);
				data_to_send[_cnt++]=BYTE0(mygetqval[2]);
				data_to_send[_cnt++]=BYTE1(mygetqval[3]); //������x��8
				data_to_send[_cnt++]=BYTE0(mygetqval[3]); //������x��8
				data_to_send[_cnt++]=BYTE1(mygetqval[4]);
				data_to_send[_cnt++]=BYTE0(mygetqval[4]);
				data_to_send[_cnt++]=BYTE1(mygetqval[5]);
				data_to_send[_cnt++]=BYTE0(mygetqval[5]);
				data_to_send[_cnt++]=BYTE1(mygetqval[6]); //������x��8
				data_to_send[_cnt++]=BYTE0(mygetqval[6]);	//������x��8
				data_to_send[_cnt++]=BYTE1(mygetqval[7]);
				data_to_send[_cnt++]=BYTE0(mygetqval[7]);
				data_to_send[_cnt++]=BYTE1(mygetqval[8]);
				data_to_send[_cnt++]=BYTE0(mygetqval[8]);
				
				data_to_send[3] = _cnt-4;

				for(i=0;i<_cnt;i++)
				sum += data_to_send[i];
				data_to_send[_cnt++] = sum;

				Uart3_Put_Buf(data_to_send,_cnt);
}

void Data_Send_PID1(void)//8.����pid1
{
				u8 _cnt=0;
				u8 sum = 0;
				u8 i;
				vs16 _temp;
		
				data_to_send[_cnt++]=0xAA;
				data_to_send[_cnt++]=0xAA;
				data_to_send[_cnt++]=0x10;
				data_to_send[_cnt++]=0;
						
				_temp = Pid_temp.core_P ;//pid1
				data_to_send[_cnt++]=BYTE1(_temp);
				data_to_send[_cnt++]=BYTE0(_temp);
				_temp = Pid_temp.core_I ;
				data_to_send[_cnt++]=BYTE1(_temp);
				data_to_send[_cnt++]=BYTE0(_temp);
				_temp = Pid_temp.core_D;
				data_to_send[_cnt++]=BYTE1(_temp);
				data_to_send[_cnt++]=BYTE0(_temp);
				_temp = Pid_temp.shell_P ;//pid2
				data_to_send[_cnt++]=BYTE1(_temp);
				data_to_send[_cnt++]=BYTE0(_temp);
				_temp = Pid_temp.shell_I ;
				data_to_send[_cnt++]=BYTE1(_temp);
				data_to_send[_cnt++]=BYTE0(_temp);
				_temp = Pid_temp.shell_D ;
				data_to_send[_cnt++]=BYTE1(_temp);
				data_to_send[_cnt++]=BYTE0(_temp);
				_temp = 0 ;           //pid3
				data_to_send[_cnt++]=BYTE1(_temp);
				data_to_send[_cnt++]=BYTE0(_temp);
				_temp =0;
				data_to_send[_cnt++]=BYTE1(_temp);
				data_to_send[_cnt++]=BYTE0(_temp);
				_temp = 0 ;
				data_to_send[_cnt++]=BYTE1(_temp);
				data_to_send[_cnt++]=BYTE0(_temp);
				
				data_to_send[3] = _cnt-4;
				
				for(i=0;i<_cnt;i++)
					sum += data_to_send[i];
				
				data_to_send[_cnt++]=sum;
				
				Uart3_Put_Buf(data_to_send,_cnt);
		}


/************************************************************
**  ˵������λ��������λ���������ݺ���
*************************************************************/
void USART3_IRQHandler(void)
{     int RxState=0;
		 if (USART_GetFlagStatus(USART3, USART_FLAG_ORE) != RESET)//��������־λ
			{ 
				USART_ClearITPendingBit(USART3,USART_IT_RXNE);//����жϱ�־λ
				USART_ReceiveData(USART3);
			}
			if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)  //�����ж� (���ռĴ����ǿ�) 
			{
				u8 com_data =USART_ReceiveData(USART3);
				static u8 _data_len = 0,_data_cnt = 0;
				if(RxState==0&&com_data==0xAA)
				{
					RxState=1;
					RxBuffer[0]=com_data;
				}
				else if(RxState==1&&com_data==0xAF)
				{
					RxState=2;
					RxBuffer[1]=com_data;
				}
				else if(RxState==2&&com_data>0&&com_data<0XF1)
				{
					RxState=3;
					RxBuffer[2]=com_data;
				}
				else if(RxState==3&&com_data<50)
				{
					RxState = 4;
					RxBuffer[3]=com_data;
					_data_len = com_data;
					_data_cnt = 0;
				}
				else if(RxState==4&&_data_len>0)
				{
					_data_len--;
					RxBuffer[4+_data_cnt++]=com_data;
					if(_data_len==0)
					RxState = 5;
				}
				else if(RxState==5)
				{
					RxState = 0;
					RxBuffer[4+_data_cnt]=com_data;
					Data_Receive_Anl(RxBuffer,_data_cnt+5);
				}
				else
					RxState = 0;
			}
}

void Data_Receive_Anl(u8 *data_buf,u8 num)
{
			u8 sum = 0;
			u8 i;
			for(i=0;i<(num-1);i++)
				 sum += *(data_buf+i);
			if(!(sum==*(data_buf+num-1)))		return;		//�ж�sum
			if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;		//�ж�֡ͷ
			if(*(data_buf+2)==0X02)
			{
			if(*(data_buf+4)==0X01)
				{
					Send_PID1 = 1;
				}
			}
			if(*(data_buf+2)==0X10)//PID1
			{	
					Pid_temp.core_P   = (float)((vs16)(*(data_buf+4)<<8)|*(data_buf+5));
					Pid_temp.core_I   = (float)((vs16)(*(data_buf+6)<<8)|*(data_buf+7));
					Pid_temp.core_D   = (float)((vs16)(*(data_buf+8)<<8)|*(data_buf+9));
					Pid_temp.shell_P  = (float)((vs16)(*(data_buf+10)<<8)|*(data_buf+11));
					Pid_temp.shell_I  = (float)((vs16)(*(data_buf+12)<<8)|*(data_buf+13));
					Pid_temp.shell_D  = (float)((vs16)(*(data_buf+14)<<8)|*(data_buf+15));
//					                = (float)((vs16)(*(data_buf+16)<<8)|*(data_buf+17));
//				  							  = (float)((vs16)(*(data_buf+18)<<8)|*(data_buf+19));
//				  						 	  = (float)((vs16)(*(data_buf+20)<<8)|*(data_buf+21));
		    Data_Send_Check(*(data_buf+2),sum);
			}
}



/***У��**/
void Data_Send_Check(u8 head, u8 check_sum)
{	
				u8 sum = 0;
				u8 i;
				data_to_send[0]=0xAA;
				data_to_send[1]=0xAA;
				data_to_send[2]=0xEF;
				data_to_send[3]=2;
				data_to_send[4]=head;
				data_to_send[5]=check_sum;
				
				for( i=0;i<6;i++)
				sum += data_to_send[i];
				data_to_send[6]=sum;
				Uart3_Put_Buf(data_to_send,7);
}
/***ʵ��******/
void Data_Exchange(void)
{
			Data_Send_Status();
			Data_Send_Sensor();
	 if(Send_PID1)
			{
				Send_PID1 = 0;
				Data_Send_PID1();
			}
}
#endif



































