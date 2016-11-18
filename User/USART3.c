
#include "USART3.h"
#include "imu.h"
#include "Wheel_Speed.h"
#define normal 1//配合匿名上位机V4.01使用
#define r 0.076 //麦克纳姆轮半径，单位m
#define pi 3.1415926
extern char odm[8];
/*-----USART3_TX-----PB10-----*/
/*-----USART3_RX-----PB11-----*/
/*****************************************************************
*** 接收cmd_vel话题，并将其转化为左右轮转速
*** 发布odm话题，提供粗略的里程计信息
*** 2d的机器人移动平台只会用到3个值linear.x,liner.y与angular.z
    分别代表水平分速度，垂直分速度，与角速度
	   |____|
		||    ||      ^
		 |    |       |                                          <
	  ||____||      | +x方向       <————  liner.y方向            ）  angular.z
		
****	接收上位机程序:共8位，第0位是数据大小，第7位是检验。
****	第1位是liner.x高8,第2位是低8
****  第3位是liner.y高8，第4位是低8
****	第5位是angular.z高8，第6位是低8		
*******************************************************************/


u8 com_data;
int head_flag=0,end_flag=0,n=0,put_flag,k=1;
static	int count=0;
u8 para_get[8];
u8 para_rev_tem[8];
extern char odm[8];
u8 rev_flag=0;
extern float mygetqval[9];
u8 data_to_send[0xff];//发送缓存
u8	RxBuffer[0xff];//接受缓存
u8 Send_PID1 = 0;//pid1发送标志位
u8 Send_PID3 = 0;//pid2发送标志位
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

void USART3_Sends(char *buf1)		  //字符串发送函数
{
		u8 i=0;
		for(i=0;i<16;i++)
		{	USART_SendData(USART3,buf1[i]);  //发送一个字节
			while(USART_GetFlagStatus(USART3,USART_FLAG_TXE)==RESET){};//等待发送结束
			}
	}
void USART3_Sendb(u8 k)		         //字节发送函数
{		 
			USART_SendData(USART3,k);  //发送一个字节
			while(USART_GetFlagStatus(USART3,USART_FLAG_TXE)==RESET){};//等待发送结束	
} 


#if  normal
void USART3_IRQHandler(void)//接收中断
{    static int a=0,b=0,c=0;
			if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)  //接收中断 (接收寄存器非空) 
			{
			USART_ClearITPendingBit(USART3,USART_IT_RXNE);//清除中断标志位
			 com_data =USART_ReceiveData(USART3);
				if(rev_flag==0){
					if(com_data==0xA1) a=1;
					if((com_data==0xA2)&&(a==1)) b=1;
					if((com_data==0xA3)&&(b==1)) 	rev_flag=1;	}//只有接收到 A1 A2 A3 串口才工作
			
				if(rev_flag==1)
				{
					if(com_data==0xA0)   head_flag=1;//判断帧头
					if((com_data==0x06)&&(head_flag==1))
						{	head_flag=0;
							put_flag=1 ;}
				if(put_flag==1)
					{	para_rev_tem[count]=com_data;	 //从0x06开始存，共存8个
						count++;
						if(count>=8)	
						{count=0;put_flag=0;}//第七个数据存完关闭存数据标志位
					}	
				if((para_rev_tem[7]==0xA1)&&(count==0))  
			 { int i;
					 for(i=1;i<7;i++) para_get[i]=para_rev_tem[i]; }//校验第8位数			  
					 liner.tx   =	(para_get[1]*256+para_get[2]);//上位机扩大10000倍发送
					 liner.ty   =	(para_get[3]*256+para_get[4]);
					 angular.tz =  (para_get[5]*256+para_get[6]);
					 liner.x   =	liner.tx/10000.0f;//上位机扩大10000倍发送
					 liner.y   =	liner.ty/10000.0f;
					 angular.z =   angular.tz/10000.f;
					 liner.x   =  (liner.x/r)*30.0f/pi;   //线速度转换成rpm     单位：v=w/r m/s ->rpm
					 liner.y   =  (liner.y/r)*30.0f/pi; 
					 angular.z =   angular.z *30.0f/pi;		//角速度转换成rpm     单位：rad/s ->rpm					
  		} else 
			  {	
					liner.x   = 0;                      
					liner.y   = 0; 
					angular.z = 0;		
       	}
			 	}
		
	}

void send_odm_msg()//发送第一个和第四个轮子的速度,注意反馈速度取值范围±8191，角度范围±1800，上位机分别应该÷19,÷100;
{ 
int i ,j;
	u8 sum;
	for(i=0;i<8;i++) twist[i]=odm[i];

    _temp = (int)(yaw_angle*100);
		twist[8]= BYTE1(_temp);//高8位
		twist[9]=	BYTE0(_temp);//低8位
		twist[10]=	0;//高8位
		twist[11]=	0;//低8位
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
**  说明：以下为下位机发往上位机的函数
**  data_to_send：发送缓存区。
**  Uart3_Put_Buf(data_to_send,_cnt)：串口发送函数
**  _cnt为需要发送的字节数。
**  BYTE3，BYTE2，BYTE1，BYTE0:表示取某变量的高低字节，0最低，3最高。
*************************************************************/
void Uart3_Put_Buf(unsigned char *DataToSend , u8 data_num)
{     u8 i;
			for(i=0;i<data_num;i++)    
				{    
				 USART_SendData(USART3, DataToSend[i]);  
				 while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET); //等待数据发送完毕
				}
}
//发送基本信息（姿态，锁定状态）
void Data_Send_Status(void)//发送姿态
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
					
				data_to_send[_cnt++]=0xA1;//解锁
				
				data_to_send[3] = _cnt-4;

				for(i=0;i<_cnt;i++)
				sum += data_to_send[i];
				data_to_send[_cnt++]=sum;

				Uart3_Put_Buf(data_to_send,_cnt);
}

//发送传感器数据
void Data_Send_Sensor(void)//发送姿态原始数据
{
				u8 _cnt=0;
				u8 sum = 0;
				u8 i;
				
				data_to_send[_cnt++]=0xAA;
				data_to_send[_cnt++]=0xAA;
				data_to_send[_cnt++]=0x02;
				data_to_send[_cnt++]=0;
//				data_to_send[_cnt++]=BYTE1(mygetqval[0]);  //加速度x低8
//				data_to_send[_cnt++]=BYTE0(mygetqval[0]);  //加速度x高8
				data_to_send[_cnt++]=BYTE1(Four_Wheel_Info.speed_raw[0]);  //加速度x低8
				data_to_send[_cnt++]=BYTE0(Four_Wheel_Info.speed_raw[0]);  //加速度x高8	
				data_to_send[_cnt++]=50>>8;
				data_to_send[_cnt++]=50;
//		  data_to_send[_cnt++]=BYTE1(mygetqval[1]);
//			data_to_send[_cnt++]=BYTE0(mygetqval[1]);
//	
				data_to_send[_cnt++]=BYTE1(mygetqval[2]);
				data_to_send[_cnt++]=BYTE0(mygetqval[2]);
				data_to_send[_cnt++]=BYTE1(mygetqval[3]); //陀螺仪x低8
				data_to_send[_cnt++]=BYTE0(mygetqval[3]); //陀螺仪x高8
				data_to_send[_cnt++]=BYTE1(mygetqval[4]);
				data_to_send[_cnt++]=BYTE0(mygetqval[4]);
				data_to_send[_cnt++]=BYTE1(mygetqval[5]);
				data_to_send[_cnt++]=BYTE0(mygetqval[5]);
				data_to_send[_cnt++]=BYTE1(mygetqval[6]); //磁力计x低8
				data_to_send[_cnt++]=BYTE0(mygetqval[6]);	//磁力计x高8
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

void Data_Send_PID1(void)//8.发送pid1
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
**  说明：下位机解析上位机发送数据函数
*************************************************************/
void USART3_IRQHandler(void)
{     int RxState=0;
		 if (USART_GetFlagStatus(USART3, USART_FLAG_ORE) != RESET)//溢出错误标志位
			{ 
				USART_ClearITPendingBit(USART3,USART_IT_RXNE);//清除中断标志位
				USART_ReceiveData(USART3);
			}
			if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)  //接收中断 (接收寄存器非空) 
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
			if(!(sum==*(data_buf+num-1)))		return;		//判断sum
			if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;		//判断帧头
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



/***校验**/
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
/***实现******/
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



































