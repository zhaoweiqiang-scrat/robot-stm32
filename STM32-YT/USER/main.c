#include "stm32f10x.h"
#include "string.h"
#include "delay.h"
#include "usart.h"
#include "stdio.h"

#define IO1_Pin GPIO_Pin_15
#define IO2_Pin GPIO_Pin_14
#define IO3_Pin GPIO_Pin_15
#define IO4_Pin GPIO_Pin_1
#define IO5_Pin GPIO_Pin_0

#define IO145_GPIO_Port GPIOB
#define IO23_GPIO_Port  GPIOC

#define MAX_CNT 22500
#define MIN_CNT 3

#define LED_CMD 1
#define VANGLE_CMD 2
#define HANGLE_CMD 3

#define LIFTER_UP    GPIO_ResetBits(IO23_GPIO_Port,IO2_Pin) //向上
#define LIFTER_DOWN  GPIO_SetBits(IO23_GPIO_Port,IO2_Pin)   //向下
#define LIFTER_EN    GPIO_ResetBits(IO23_GPIO_Port,IO3_Pin)   //使能升降杆
#define LIFTER_DIS   GPIO_SetBits(IO23_GPIO_Port,IO3_Pin) //关闭升降杆

uint8_t PWM_FLAG_DOWM,PWM_FLAG_UP,LIFTER_ACTION,ANGLE_ACTION,HOME_CMD;
uint8_t Auto_Up_Cmd,Auto_Down_Cmd,Manual_Cmd,Auto_Flag;
uint8_t Yt_Auto_Up_Cmd,Yt_Auto_Down_Cmd,Yt_Auto_Left_Cmd,Yt_Auto_Right_Cmd,Yt_Manual_Cmd,Yt_Auto_Flag,Yt_Auto_Flag1,Yt_Auto_Flag2;
uint16_t PWM_FLAG_CNT,PREV_TARGET_CNT,CURRENT_TARGET_CNT;
uint8_t rebuf[11]={0};
uint8_t len = 0;
uint8_t YT_LED_FLAG,Reset_Flag;
volatile uint8_t RX_OK;//接收成功标志
uint8_t START_TRA,RECEIVE_TRA;//开始传输标志
uint8_t H_flag,V_flag;
typedef union {
        float real;
        uint8_t base[4];
      } u_yt;
//u_yt yt;
//uint8_t led_level[4];		

void Auto_Instruction_Angle(u_yt yt_Hangle,u_yt yt_Vangle,uint8_t action,uint8_t function)
{
	set_rx_tx(1,1);//发送模式
	Send_out(yt_Hangle.base,yt_Vangle.base,action,function);
	set_rx_tx(0,0);//换接收模式
}

void send_Instruction_Angle(u_yt yt_Hangle,u_yt yt_Vangle)
{
	int h_angle,v_angle;
	char v_ang[2];
	
	h_angle = (int)(yt_Hangle.real*100);
	v_angle = (int)(yt_Vangle.real*100);
	
	set_rx_tx(1,1);//发送模式
	//Send_out(yt_Hangle.base,yt_Vangle.base,DOU_CONTR,WRITE_DATA);
	HY_Send_out(H_POS_ANG, 0x3c, (h_angle >> 8) & 0xff, h_angle & 0xff);//发送水平角
	delay_ms(500);
	HY_Send_out(V_POS_ANG, 0x1e, (v_angle >> 8) & 0xff, v_angle & 0xff);//发送俯仰角
	//delay_ms(100);
	//HY_Send_out(SET_HV_TYPE, ANG_BACK_ON, 0x01, 0xf4);//0x1f4,500ms自动回传周期
	set_rx_tx(0,0);//换接收模式
	delay_ms(100);
}
void send_Instruction_Vangle(u_yt yt_Vangle)
{
	set_rx_tx(1,1);//发送模式
	send_out(yt_Vangle.base,PAN_CONTR,WRITE_DATA);
	//set_rx_tx(0,0);//换接收模式
	delay_ms(100);
}
void send_Instruction_Hangle(u_yt yt_Hangle)
{
	set_rx_tx(1,1);//发送模式
	send_out(yt_Hangle.base,TILT_CONTR,WRITE_DATA);
	//set_rx_tx(0,0);//换接收模式
	//delay_ms(100);
}
void send_Instruction_led(u_yt yt_Led)
{
	set_rx_tx(1,1);//发送模式
	send_out(yt_Led.base,LIGHT_CONTR,WRITE_DATA);
	set_rx_tx(0,0);//换接收模式
	//delay_ms(100);
}
void send_Reset_Angle()
{
	uint8_t reset_data[4];
	
	reset_data[0] = 0x00;
	reset_data[1] = 0x00;
	reset_data[2] = 0x00;
	reset_data[3] = 0x00;
	
	set_rx_tx(1,1);//发送模式
	send_out(reset_data,DOU_CONTR,INITIAL_SYS);
	//
	//set_rx_tx(0,0);//换接收模式
	delay_ms(100);
}
u8  usart1_len;
char usart1_ch;
u8 usart1_rx_buf[8];
u_yt yt,yt1,yt2,yt_led;
void USART1_IRQHandler(void)
{
	
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
		USART_ClearFlag(USART1,USART_FLAG_RXNE);//清中断标志
		usart1_ch=USART_ReceiveData(USART1);
		usart1_rx_buf[usart1_len++]=usart1_ch;
		if(usart1_len >= 8)
		{
			printf("数据1：0x%x",usart1_rx_buf[0]);
			printf("数据2：0x%x",usart1_rx_buf[1]);
			printf("数据3：0x%x",usart1_rx_buf[2]);
			printf("数据4：0x%x",usart1_rx_buf[3]);
			printf("数据5：0x%x",usart1_rx_buf[4]);
			printf("数据6：0x%x",usart1_rx_buf[5]);
			printf("数据7：0x%x",usart1_rx_buf[6]);
			printf("数据8：0x%x\r\n",usart1_rx_buf[7]);
			switch(usart1_rx_buf[2])
			{
				case LED_CMD:
										//send_Instruction_led(usart1_rx_buf[6]);
										break;
				case VANGLE_CMD:
										yt1.base[0] = usart1_rx_buf[3];
										yt1.base[1] = usart1_rx_buf[4];
										yt1.base[2] = usart1_rx_buf[5];
										yt1.base[3] = usart1_rx_buf[6];
										send_Instruction_Vangle(yt1);
										break;
				case HANGLE_CMD:
										yt2.base[0] = usart1_rx_buf[3];
										yt2.base[1] = usart1_rx_buf[4];
										yt2.base[2] = usart1_rx_buf[5];
										yt2.base[3] = usart1_rx_buf[6];
										send_Instruction_Hangle(yt2);
										break;
			}
			usart1_len = 0;
		}	 
	}	
}
void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC|RCC_APB2Periph_AFIO, ENABLE);
	
  /*Configure GPIO pins : PBPin PBPin */
  GPIO_InitStruct.GPIO_Pin = IO1_Pin;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(IO145_GPIO_Port, &GPIO_InitStruct); 
	
	GPIO_InitStruct.GPIO_Pin = IO4_Pin;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(IO145_GPIO_Port, &GPIO_InitStruct); 
	
	GPIO_InitStruct.GPIO_Pin = IO5_Pin;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(IO145_GPIO_Port, &GPIO_InitStruct); 
  
	 /*Configure GPIO pins : PBPin PBPin */
  GPIO_InitStruct.GPIO_Pin = IO2_Pin|IO3_Pin;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(IO23_GPIO_Port, &GPIO_InitStruct); 
	
	/*GPIOB.0 中断线配置*/
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource0);
    
  /*GPIOB.0 中断初始化配置*/
  EXTI_InitStructure.EXTI_Line=EXTI_Line0;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	/*根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存*/
  EXTI_Init(&EXTI_InitStructure);	 
	
	/*GPIOB.1 中断线配置*/
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource1);
	 /*GPIOB.1 中断初始化配置*/
  EXTI_InitStructure.EXTI_Line=EXTI_Line1;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	/*根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存*/
  EXTI_Init(&EXTI_InitStructure);	 
	
	
  NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
  /*设置抢占优先级，抢占优先级设为2*/	
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;	
	  /*设置子优先级，子优先级设为1*/
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;		
    /*使能外部中断通*/
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	
     /*根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器*/		
  NVIC_Init(&NVIC_InitStructure); 
}
void Interrupt_Enable()//脉冲边沿中断使能
{
	NVIC_InitTypeDef NVIC_InitStructure;
	
	 NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
  /*设置抢占优先级，抢占优先级设为2*/	
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;	
	  /*设置子优先级，子优先级设为1*/
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;		
    /*使能外部中断通*/
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	
     /*根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器*/		
  NVIC_Init(&NVIC_InitStructure); 
}
void Send_Height()
{
	uint8_t data_buf[4],i=0;
	
	memset(data_buf,0,4);//清零缓存
	data_buf[0] = 0x5A;
	data_buf[1] = 0xA5;
	data_buf[2] = PWM_FLAG_CNT>>8;//PWM_FLAG_CNT
	data_buf[3] = PWM_FLAG_CNT&0XFF;//CURRENT_TARGET_CNT
	//printf("升降杆高度mm：%f\n\r",PWM_FLAG_CNT*0.85953841);
  #if 0
	memcpy(SendBuff1,data_buf,4); 
	USART_DMACmd(USART2,USART_DMAReq_Tx,ENABLE);  //使能串口2的DMA发送 
	MYDMA_Enable(DMA1_Channel7);     //开始一次DMA传输！
	#else
	while(i<4)
	{
		//printf("发送数据%d:0x%x\r\n",i,data_buf[i]);
		USART_send_byte(USART2,data_buf[i++]);
	}
	#endif
}
void Send_Vangle()
{
	uint8_t data_buf[6],i=0;
	
	memset(data_buf,0,6);//清零缓存
	data_buf[0] = 0xCF;
	data_buf[1] = 0x01;
	data_buf[2] = RX_BUF[3];
	data_buf[3] = RX_BUF[4];
	data_buf[4] = RX_BUF[5];
	data_buf[5] = RX_BUF[6];
//	u_yt Angle_YT;
//	Angle_YT.base[0] = RX_BUF[6];
//	Angle_YT.base[1] = RX_BUF[5];
//	Angle_YT.base[2] = RX_BUF[4];
//	Angle_YT.base[3] = RX_BUF[3];
	//printf("返回的的俯仰角为：%f\r\n",Angle_YT.real);
  #if 0
	memcpy(SendBuff1,data_buf,6); 
	USART_DMACmd(USART2,USART_DMAReq_Tx,ENABLE);  //使能串口2的DMA发送 
	MYDMA_Enable(DMA1_Channel7);     //开始一次DMA传输！
	#else
	while(i<6)
	{
		//printf("发送数据%d:0x%x\r\n",i,data_buf[i]);
		USART_send_byte(USART2,data_buf[i++]);
	}
	#endif
}
void Send_Hangle()
{
	uint8_t data_buf[6],i=0;
//	u_yt Angle_YT;
	
	memset(data_buf,0,6);//清零缓存
	data_buf[0] = 0xCF;
	data_buf[1] = 0x02;
	data_buf[2] = RX_BUF1[3];
	data_buf[3] = RX_BUF1[4];
	data_buf[4] = RX_BUF1[5];
	data_buf[5] = RX_BUF1[6];
	
//	Angle_YT.base[0] = RX_BUF1[6];
//	Angle_YT.base[1] = RX_BUF1[5];
//	Angle_YT.base[2] = RX_BUF1[4];
//	Angle_YT.base[3] = RX_BUF1[3];
	//printf("返回的的水平角为：%f\r\n",Angle_YT.real);
	#if 0
	memcpy(SendBuff1,data_buf,6); 
	USART_DMACmd(USART2,USART_DMAReq_Tx,ENABLE);  //使能串口2的DMA发送 
	MYDMA_Enable(DMA1_Channel7);     //开始一次DMA传输！
	#else
	while(i<6)
	{
		delay_ms(1);
		USART_send_byte(USART2,data_buf[i++]);
	}
	#endif
}
void Send_Angle()
{
	uint8_t data_buf[10],i=0;
//	u_yt Angle_YT,Angle_YT1;
	
	memset(data_buf,0,6);//清零缓存
	data_buf[0] = 0xCF;
	data_buf[1] = 0x03;
	
	data_buf[2] = RX_BUF2[3];
	data_buf[3] = RX_BUF2[4];
	data_buf[4] = RX_BUF2[5];
	data_buf[5] = RX_BUF2[6];
	
	
	#if 0
	memcpy(SendBuff1,data_buf,6); 
	USART_DMACmd(USART2,USART_DMAReq_Tx,ENABLE);  //使能串口2的DMA发送 
	MYDMA_Enable(DMA1_Channel7);     //开始一次DMA传输！
	#else
	while(i<6)
	{
		//printf("发送数据%d:0x%x\r\n",i,data_buf[i]);
		//delay_ms(1);
		USART_send_byte(USART2,data_buf[i++]);
	}
	if(RX_BUF2[3] == 0x59){
		V_flag = 1;
		H_flag = 0;
	}
	if(RX_BUF2[3] == 0x5b){
		H_flag = 1;
		V_flag = 0;
	}
	#endif
	#if 0
	Angle_YT.base[0] = RX_BUF2[6];
	Angle_YT.base[1] = RX_BUF2[5];
	Angle_YT.base[2] = RX_BUF2[4];
	Angle_YT.base[3] = RX_BUF2[3];
	printf("返回的的水平角为：%f\r\n",Angle_YT.real);
	Angle_YT1.base[0] = RX_BUF2[10];
	Angle_YT1.base[1] = RX_BUF2[9];
	Angle_YT1.base[2] = RX_BUF2[8];
	Angle_YT1.base[3] = RX_BUF2[7];
	printf("返回的的俯仰角为：%f\r\n",Angle_YT1.real);
	#endif
}
void Send_Lifter_Ack()
{
	uint8_t data_buf[6],i=0;
	
	memset(data_buf,0,6);//清零缓存
	data_buf[0] = 0xCF;
	data_buf[1] = 0X5A;
	data_buf[2] = 0xFF;
	data_buf[3] = 0xFF;
	data_buf[4] = 0xFF;
	data_buf[5] = 0xFF;
	#if 0
	memcpy(SendBuff1,data_buf,6); 
	USART_DMACmd(USART2,USART_DMAReq_Tx,ENABLE);  //使能串口2的DMA发送 
	MYDMA_Enable(DMA1_Channel7);     //开始一次DMA传输！
	#else
	while(i<6)
	{
		//printf("发送数据%d:0x%x\r\n",i,data_buf[i]);
		USART_send_byte(USART2,data_buf[i++]);
	}
	#endif
}
void Send_Angle_Ack()
{
	uint8_t data_buf[6],i=0;
	
	memset(data_buf,0,6);//清零缓存
	data_buf[0] = 0xCF;
	data_buf[1] = 0X05;
	data_buf[2] = 0xFF;
	data_buf[3] = 0xFF;
	data_buf[4] = 0xFF;
	data_buf[5] = 0xFF;
	#if 0
	memcpy(SendBuff1,data_buf,6); 
	USART_DMACmd(USART2,USART_DMAReq_Tx,ENABLE);  //使能串口2的DMA发送 
	MYDMA_Enable(DMA1_Channel7);     //开始一次DMA传输！
	#else
	//USART_Send_bytes(USART2,data_buf,6);
	#if 1 
	while(i<6)
	{
		//printf("发送数据%d:0x%x\r\n",i,data_buf[i]);
		
		USART_send_byte(USART2,data_buf[i++]);
	}
	#endif
	#endif
}
void Send_Led_Ack()
{
	uint8_t data_buf[6],i=0;
	
	memset(data_buf,0,6);//清零缓存
	data_buf[0] = 0xCF;
	data_buf[1] = 0X06;
	data_buf[2] = 0xFF;
	data_buf[3] = 0xFF;
	data_buf[4] = 0xFF;
	data_buf[5] = 0xFF;
	
	#if 1 
	while(i<6)
	{
		//printf("发送数据%d:0x%x\r\n",i,data_buf[i]);
		
		USART_send_byte(USART2,data_buf[i++]);
	}
	#endif
}
void YT_Lifter_Contr(void);
void LIFTER_EXC()
{
	if(GPIO_ReadInputDataBit(IO23_GPIO_Port,IO2_Pin) == RESET)//向上
	{
		if(PWM_FLAG_CNT > MAX_CNT)
		{
			LIFTER_DIS;
		}
	}
}
void LIFTER_AUTO_MODE()
{
    if(Auto_Up_Cmd && !Auto_Down_Cmd && !Manual_Cmd && !Auto_Flag)//自动模式
		{
			Auto_Flag = 1;
			LIFTER_UP;
			LIFTER_EN;
		}
		if(Auto_Down_Cmd && !Auto_Up_Cmd && !Manual_Cmd && !Auto_Flag)//自动模式
		{
			Auto_Flag = 1;
			LIFTER_DOWN;
			LIFTER_EN;
		}
		if(!Auto_Down_Cmd && !Auto_Up_Cmd && Manual_Cmd && Auto_Flag)//手动模式
		{
			Auto_Flag = 0;
			LIFTER_DIS;
		}
}
void YT_AUTO_MODE()
{
		u_yt yt_Vangle_Up,yt_Vangle_Down,yt_Vangle;
		u_yt yt_Hangle_Left,yt_Hangle_Right,yt_Hangle;
	   set_rx_tx(1,1);//发送模式
    if(Yt_Auto_Up_Cmd && !Yt_Auto_Down_Cmd && !Yt_Auto_Left_Cmd && !Yt_Auto_Right_Cmd && !Yt_Manual_Cmd && !Yt_Auto_Flag)//自动模式
		{
			Yt_Auto_Flag = 1;
			Yt_Auto_Flag1 = 1;
			Yt_Auto_Flag2 = 0;
			yt_Hangle.real = 0;
			yt_Vangle.real = 0;
			//Auto_Instruction_Angle(yt_Hangle,yt_Vangle,PAN_CONTR,CW_AUTO);
			HY_Send_out(D_TYPE,0x08, 0x00, 0x19);//按2.5r/min的速度向上转
		}
		if(!Yt_Auto_Up_Cmd && Yt_Auto_Down_Cmd && !Yt_Auto_Left_Cmd && !Yt_Auto_Right_Cmd && !Yt_Manual_Cmd && !Yt_Auto_Flag)//自动模式
		{
			Yt_Auto_Flag = 1;
			Yt_Auto_Flag1 = 1;
			Yt_Auto_Flag2 = 0;
			yt_Hangle.real = 0;
			yt_Vangle.real = 0;
			//Auto_Instruction_Angle(yt_Hangle,yt_Vangle,PAN_CONTR,CCW_AUTO);
			HY_Send_out(D_TYPE, 0x10, 0x00, 0x19);//按2.5r/min的速度向下转
		}
		if(!Yt_Auto_Up_Cmd && !Yt_Auto_Down_Cmd && Yt_Auto_Left_Cmd && !Yt_Auto_Right_Cmd && !Yt_Manual_Cmd && !Yt_Auto_Flag)//自动模式--水平逆时针
		{
			Yt_Auto_Flag = 1;
			Yt_Auto_Flag1 = 0;
			Yt_Auto_Flag2 = 1;
			yt_Hangle.real = 0;
			yt_Vangle.real = 0;
			//Auto_Instruction_Angle(yt_Hangle,yt_Vangle,TILT_CONTR,CCW_AUTO);
			HY_Send_out(D_TYPE, 0x04, 0x32, 0x00);//按5r/min的速度向左转
		}
		if(!Yt_Auto_Up_Cmd && !Yt_Auto_Down_Cmd && !Yt_Auto_Left_Cmd && Yt_Auto_Right_Cmd && !Yt_Manual_Cmd && !Yt_Auto_Flag)//自动模式--水平顺时针
		{
			Yt_Auto_Flag = 1;
			Yt_Auto_Flag1 = 0;
			Yt_Auto_Flag2 = 1;
			yt_Hangle.real = 0;
			yt_Vangle.real = 0;
			//Auto_Instruction_Angle(yt_Hangle,yt_Vangle,TILT_CONTR,CW_AUTO);
			HY_Send_out(D_TYPE, 0x02, 0x32, 0x00);//按5r/min的速度向右转
		}
		if(!Yt_Auto_Up_Cmd && !Yt_Auto_Down_Cmd && !Yt_Auto_Left_Cmd && !Yt_Auto_Right_Cmd && Yt_Manual_Cmd && Yt_Auto_Flag)//手动模式
		{
			Yt_Auto_Flag = 0;
			yt_Hangle.real = 0;
			yt_Vangle.real = 0;
//			if(Yt_Auto_Flag1)
//			{
//				Yt_Auto_Flag1 = 0;
//				Auto_Instruction_Angle(yt_Hangle,yt_Vangle,PAN_CONTR,STOP_AUTO);
//			}
//			if(Yt_Auto_Flag2)
//			{
//				Yt_Auto_Flag2 = 0;
//				Auto_Instruction_Angle(yt_Hangle,yt_Vangle,TILT_CONTR,STOP_AUTO);
//			}
			HY_Send_out(D_TYPE, 0x00, 0x00, 0x00);	
		}
		set_rx_tx(0,0);//换接收模式
}
void SoftReset(void)
{
    __set_FAULTMASK(1); //关闭所有中断
    NVIC_SystemReset(); // 复位
}
int main(void)
{
	delay_init(72);//72M 
	/* Configure the NVIC Preemption Priority Bits */  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	MX_GPIO_Init();
	Usart1_Int(115200);
	Usart2_Int(115200);//与MCU-STM32F429通信
	Usart3_Int(19200);//115200波特率,与云台通信
  
	LIFTER_ACTION = 1;
	ANGLE_ACTION = 1;
	
	LIFTER_UP;
	LIFTER_EN;
	delay_ms(1000);
	LIFTER_DOWN;
	//LIFTER_DIS;
	PWM_FLAG_CNT = 4;
	CURRENT_TARGET_CNT = 1400 - 2*28;//5cm对应1400个脉冲，减掉过冲的2mm
	//Interrupt_Enable();//开机后升降杆回归原点，使能脉冲边沿中断，用于上升下降时的计数
	printf("巡检机器人--------中电科西北集团有限公司\n\r");
	H_flag = 1;
	V_flag = 0;
	//yt2.real = 0;
	//yt1.real = 0;
	//send_Instruction_Angle(yt2,yt1);
	while(1)
	{
		printf("巡检机器人\n\r");
		YT_Lifter_Contr();	
		if(YT_Flag_Angle1&&YT_Flag_Angle2)//接收到云台角度命令，同时云台可操作
		{	
			 //printf("接收到云台角度: 水平：%f,俯仰：%f\r\n",yt2.real,yt1.real);
			send_Instruction_Angle(yt2,yt1);//同时发送俯仰水平
			YT_Flag_Angle1 = 0;
			YT_Flag_Angle2 = 0;
		}
		if(H_flag)
		{
			set_rx_tx(1,1);//发送模式
			HY_Send_out(D_TYPE, H_ANG_BACK, 0x00, 0x00);//查询水平角度
			set_rx_tx(0,0);//换接收模式
		}
		if(V_flag)
		{
			set_rx_tx(1,1);//发送模式
			HY_Send_out(D_TYPE, V_ANG_BACK, 0x00, 0x00);
			set_rx_tx(0,0);//换接收模式
		}
		if(HOME_CMD&&(START_TRA == 1))
		{
			HOME_CMD = 0;
			//send_Home_Angle();
		}
		delay_ms(100);
		if(YT_LED_FLAG)//是否接收到LED控制命令
		{
			YT_LED_FLAG = 0;
			send_Instruction_led(yt_led);
		}
		
		if(YT_Flag_Angle)
		{
			YT_Flag_Angle = 0;
			Send_Angle();
		}
		//LIFTER_EXC();//升降杆上升高度超过阈值
		delay_ms(100);
		if(LIFTER_ACTION || Auto_Up_Cmd || Auto_Down_Cmd)
		{
			Send_Height();
		}
		LIFTER_AUTO_MODE();
		//YT_AUTO_MODE();
		if(Reset_Flag)
		{
			send_Reset_Angle();
			Reset_Flag = 0;
			SoftReset();
		}
	}
}

static uint8_t deal_irq_rx_end(uint8_t *buf)  
{ 
	uint8_t Length = 0;//数据长度
	if(USART_GetITStatus(USART2, USART_IT_IDLE) != RESET) 
  { 
        DMA_Cmd(DMA1_Channel6,DISABLE);
        USART2->SR; 
        USART2->DR; //清USART_IT_IDLE标志
        Length = UART_RX_LEN - DMA_GetCurrDataCounter(DMA1_Channel6); 
				RX_OK = 1;
		    memcpy(buf,Uart_Rx,Length);  
        //设置传输数据长度 
        DMA1_Channel6->CNDTR = UART_RX_LEN;//重新填装，并让地址偏址从0开始
        DMA_Cmd(DMA1_Channel6, ENABLE);//处理完，重新打开DMA  
				return Length;
	}
	return 0;
}
uint8_t CHeck(uint8_t *data,uint8_t length)
{
	uint8_t sum=0,i=0;

	if(length>11)//超过上传数据
		return 0;
	for(i=0;i<length-1;i++)
	 sum+=data[i];
	if(sum==data[length-1])
		return 1;
	else
    return 0;
}
void YT_Lifter_Contr(void)
{
	if(RX_OK == 1)
	{
		RX_OK = 0;
		if(CHeck(rebuf,len))
		{
			if((rebuf[0] == 0x5A)&&(rebuf[1] == 0xA5))
		  {  //接收到0x5A 0XA5代表使能升降杆，第三个字节代表脉冲数高字节，第四个字节代表脉冲数低字节
			   len = 0;
			   CURRENT_TARGET_CNT = (rebuf[2]<<8)|rebuf[3];
			   if(CURRENT_TARGET_CNT > 22500)CURRENT_TARGET_CNT=22500;
			   if(CURRENT_TARGET_CNT < 2)CURRENT_TARGET_CNT=4;
						     
			   LIFTER_ACTION = 0;
				
				 Send_Lifter_Ack();
				
				 if((CURRENT_TARGET_CNT != PREV_TARGET_CNT) || (abs(CURRENT_TARGET_CNT - PWM_FLAG_CNT) > 5))
				 {
					 PREV_TARGET_CNT = CURRENT_TARGET_CNT;
					 if(CURRENT_TARGET_CNT > PWM_FLAG_CNT)//给定值大于当前值，向上运行
					 {
						  CURRENT_TARGET_CNT = CURRENT_TARGET_CNT - 2*28 ;//向上运行时，存在过冲2mm的现象，每毫米28个脉冲
						  Auto_Up_Cmd = 0;
			        Auto_Down_Cmd = 0;
			        Manual_Cmd = 0;
							LIFTER_UP;
							LIFTER_EN;
			     }
					 else if(CURRENT_TARGET_CNT < PWM_FLAG_CNT)//给定值小于当前值，向下运行
					 {
						  Auto_Up_Cmd = 0;
			        Auto_Down_Cmd = 0;
			        Manual_Cmd = 0;
						  CURRENT_TARGET_CNT = CURRENT_TARGET_CNT + 6*28 ;//向下运行时，存在过冲6mm的现象，每毫米28个脉冲
							LIFTER_DOWN;
							LIFTER_EN;
			     }
					 
				 }
			   else
				 {
					 LIFTER_DIS;
				 }
				 
			  // printf("接收到的升降杆高度为：%fmm\r\n",CURRENT_TARGET_CNT*0.85953841);
				 LIFTER_ACTION = 1;
		 }
		 else if((rebuf[0] == 0xcf)&&(rebuf[1] == 0x01)&&(len >=7))//云台LED控制命令
		 {
			   len = 0;
			   yt_led.base[0] = rebuf[2];
			   yt_led.base[1] = rebuf[3];
			   yt_led.base[2] = rebuf[4];
			   yt_led.base[3] = rebuf[5];
			 
			   //printf("接收到的LED数据为：%f\r\n",yt_led.real);
			   Send_Led_Ack();
				 LIFTER_ACTION = 0;
			   YT_LED_FLAG = 1;
			  // led_level[3] = rebuf[5];
		 }
		 else if((rebuf[0] == 0xcf)&&(rebuf[1] == 0x05))//云台俯仰\水平角度数据
		 {
				 Yt_Auto_Up_Cmd = 0;
			   Yt_Auto_Down_Cmd = 0;
			   Yt_Auto_Left_Cmd = 0;
			   Yt_Auto_Right_Cmd = 0;
			   Yt_Manual_Cmd = 0;
			   len = 0;
				 //printf("到此\n\r");
			   LIFTER_ACTION = 0;
			   
			   yt1.base[0] = rebuf[2];
			   yt1.base[1] = rebuf[3];
			   yt1.base[2] = rebuf[4];
			   yt1.base[3] = rebuf[5];//俯仰角度
				 YT_Flag_Angle1 = 1;
			   //printf("recevie V：%f\r\n",yt1.real);
			   yt2.base[0] = rebuf[6];
			   yt2.base[1] = rebuf[7];
			   yt2.base[2] = rebuf[8];
			   yt2.base[3] = rebuf[9];//水平角度
				 YT_Flag_Angle2 = 1;
			   Send_Angle_Ack();//响应数据包
				 //printf("recevie H：%f\r\n",yt2.real);
		 }
		 else if((rebuf[0] == 0xFF)&&(rebuf[1] == 0xEE)&&(rebuf[2] == 0xDD))//接收到升降杆高度响应
		 {
			 LIFTER_ACTION = 0;//停止发送
		 }
		 else if((rebuf[0] == 0xFF)&&(rebuf[1] == 0xDD)&&(rebuf[2] == 0xCC))//接收到云台角度响应
		 {
			 YT_Flag_Angle1 = 0;
			 YT_Flag_Angle2 = 0;//停止给103云台发送
			 YT_Flag_Angle  = 0;
		 }
		 else if((rebuf[0] == 0xFF)&&(rebuf[1] == 0xCC)&&(rebuf[2] == 0xBB))//接收到升降杆高度响应
		 {
			 //HOME_CMD = 1;//发送云台归位命		 
		 }
		 else if((rebuf[0] == 0xFF)&&(rebuf[1] == 0xBB)&&(rebuf[2] == 0xAA))//自动模式上升
		 {
			 LIFTER_ACTION = 1;
			 Auto_Up_Cmd = 1;
			 Auto_Down_Cmd = 0;
			 Manual_Cmd = 0;
			 Auto_Flag = 0;
		 }
		 else if((rebuf[0] == 0xFF)&&(rebuf[1] == 0xAA)&&(rebuf[2] == 0x99))//自动模式下降
		 {
			 LIFTER_ACTION = 1;
			 Auto_Up_Cmd = 0;
			 Auto_Down_Cmd = 1;
			 Manual_Cmd = 0;
			 Auto_Flag = 0;
		 }
		 else if((rebuf[0] == 0xFF)&&(rebuf[1] == 0x99)&&(rebuf[2] == 0x88))//手动模式
		 {
			 Auto_Up_Cmd = 0;
			 Auto_Down_Cmd = 0;
			 Manual_Cmd = 1;
			 Auto_Flag = 1;
		 } 
		 else if((rebuf[0] == 0xFF)&&(rebuf[1] == 0x88)&&(rebuf[2] == 0x77))//俯视自动
		 {
			 Yt_Auto_Up_Cmd = 1;
			 Yt_Auto_Down_Cmd = 0;
			 Yt_Auto_Left_Cmd = 0;
			 Yt_Auto_Right_Cmd = 0;
			 Yt_Manual_Cmd = 0;
			 Yt_Auto_Flag = 0;
			 set_rx_tx(1,1);//发送模式
			 HY_Send_out(D_TYPE,0x08, 0x00, 0x19);//按2.5r/min的速度向上转
			 set_rx_tx(0,0);
		 }
		 else if((rebuf[0] == 0xFF)&&(rebuf[1] == 0x77)&&(rebuf[2] == 0x66))//仰视自动
		 {
			 Yt_Auto_Up_Cmd = 0;
			 Yt_Auto_Down_Cmd = 1;
			 Yt_Auto_Left_Cmd = 0;
			 Yt_Auto_Right_Cmd = 0;
			 Yt_Manual_Cmd = 0;
			 Yt_Auto_Flag = 0;
			 set_rx_tx(1,1);//发送模式
			 HY_Send_out(D_TYPE, 0x10, 0x00, 0x19);//按2.5r/min的速度向下转
			 set_rx_tx(0,0);
		 }
		 else if((rebuf[0] == 0xFF)&&(rebuf[1] == 0x66)&&(rebuf[2] == 0x55))//逆时针自动
		 {
			 LIFTER_ACTION = 0;
			 Yt_Auto_Up_Cmd = 0;
			 Yt_Auto_Down_Cmd = 0;
			 Yt_Auto_Left_Cmd = 1;
			 Yt_Auto_Right_Cmd = 0;
			 Yt_Manual_Cmd = 0;
			 Yt_Auto_Flag = 0;
			 /*自动模式，手动停止*/
			 YT_Flag_Angle1 = 0;
			 YT_Flag_Angle2 = 0;//停止给103云台发送
			 YT_Flag_Angle  = 0;
			 set_rx_tx(1,1);//发送模式
			 HY_Send_out(D_TYPE, 0x04, 0x32, 0x00);//按5r/min的速度向左转
			 set_rx_tx(0,0);
		 }
		 else if((rebuf[0] == 0xFF)&&(rebuf[1] == 0x55)&&(rebuf[2] == 0x44))//顺时针自动
		 {
			 LIFTER_ACTION = 0;
			 Yt_Auto_Up_Cmd = 0;
			 Yt_Auto_Down_Cmd = 0;
			 Yt_Auto_Left_Cmd = 0;
			 Yt_Auto_Right_Cmd = 1;
			 Yt_Manual_Cmd = 0;
			 Yt_Auto_Flag = 0;
			 /*自动模式，手动停止*/
			 YT_Flag_Angle1 = 0;
			 YT_Flag_Angle2 = 0;//停止给103云台发送
			 YT_Flag_Angle  = 0;
			 set_rx_tx(1,1);//发送模式
			 HY_Send_out(D_TYPE, 0x02, 0x32, 0x00);//按5r/min的速度向右转
			 set_rx_tx(0,0);
		 }
		 else if((rebuf[0] == 0xFF)&&(rebuf[1] == 0x44)&&(rebuf[2] == 0x33))//手动
		 {
			 LIFTER_ACTION = 0;
			 Yt_Auto_Up_Cmd = 0;
			 Yt_Auto_Down_Cmd = 0;
			 Yt_Auto_Left_Cmd = 0;
			 Yt_Auto_Right_Cmd = 0;
			 Yt_Manual_Cmd = 1;
			 Yt_Auto_Flag = 1;
			 /*自动模式，手动停止*/
			 //YT_Flag_Angle1 = 0;
			 //YT_Flag_Angle2 = 0;//停止给103云台发送
			 YT_Flag_Angle  = 0;
			 set_rx_tx(1,1);//发送模式
			 HY_Send_out(D_TYPE, 0x00, 0x00, 0x00);	
			 set_rx_tx(0,0);
		 }
		 else if((rebuf[0] == 0xFF)&&(rebuf[1] == 0x33)&&(rebuf[2] == 0x22))//归中
		 {
			 LIFTER_ACTION = 0;
			 Yt_Auto_Up_Cmd = 0;
			 Yt_Auto_Down_Cmd = 0;
			 Yt_Auto_Left_Cmd = 0;
			 Yt_Auto_Right_Cmd = 0;
			 Yt_Manual_Cmd = 1;
			 Yt_Auto_Flag = 1;
			 /*自动模式，手动停止*/
			 //YT_Flag_Angle1 = 0;
			 //YT_Flag_Angle2 = 0;//停止给103云台发送
			 YT_Flag_Angle  = 0;
			 yt1.base[0] = 0;
			 yt1.base[1] = 0;
			 yt1.base[2] = 0;
			 yt1.base[3] = 0;//俯仰角度
			 YT_Flag_Angle1 = 1;
			 //printf("recevie V：%f\r\n",yt1.real);
			 yt2.base[0] = 0;
			 yt2.base[1] = 0;
			 yt2.base[2] = 0;
			 yt2.base[3] = 0;//水平角度
			 YT_Flag_Angle2 = 1;
			 //send_Instruction_Angle(yt2,yt1);//同时发送俯仰水平
		 }
		 else if((rebuf[0] == 0xFF)&&(rebuf[1] == 0x33)&&(rebuf[2] == 0x22))//复位
		 {
			 Reset_Flag = 1;
		 }
	  }
	}
}
void USART2_IRQHandler(void)
{
	len = deal_irq_rx_end(rebuf);
}

void EXTI0_IRQHandler()
{
	if(GPIO_ReadInputDataBit(IO145_GPIO_Port,IO5_Pin) == RESET)
	{
		LIFTER_DIS;
		PWM_FLAG_CNT = 4;
		if(!Auto_Down_Cmd)
		{
		PWM_FLAG_CNT = 4;
		CURRENT_TARGET_CNT = 1400 - 2*28;//5cm对应1400个脉冲，减掉过冲的2mm
		delay_ms(1000);
		delay_ms(1000);
		delay_ms(1000);
		LIFTER_UP;
		LIFTER_EN;
		Interrupt_Enable();//开机后升降杆回归原点，使能脉冲边沿中断，用于上升下降时的计数
		}
	}  
	EXTI_ClearITPendingBit(EXTI_Line0);
}


void EXTI1_IRQHandler()
{
	if(GPIO_ReadInputDataBit(IO23_GPIO_Port,IO2_Pin) == RESET)//向上
	{
		if(PWM_FLAG_CNT > MAX_CNT)
		{
			LIFTER_DIS;
		}
		if(GPIO_ReadInputDataBit(IO145_GPIO_Port,IO4_Pin) == SET)//上升沿触发
	  {
       PWM_FLAG_UP = 1;//上升沿标志
       PWM_FLAG_DOWM = 2;//上升沿标志
    }
	  else//下降沿触发
    {
       if((PWM_FLAG_DOWM==2)&&(PWM_FLAG_UP == 1))//判断上一次是否为上升沿
       {
         PWM_FLAG_CNT += 1;		 
       }
       PWM_FLAG_DOWM=1;//下降沿标志
       PWM_FLAG_UP=2;//下降沿标志
    }
		if(!Auto_Up_Cmd && (PWM_FLAG_CNT >= CURRENT_TARGET_CNT))
		{
			 LIFTER_ACTION = 1;//升降杆数据发送
			 LIFTER_DIS;
		}	
		if(Auto_Up_Cmd && (PWM_FLAG_CNT >= 20000))
			LIFTER_DIS;
	}
	else
	{
		if(GPIO_ReadInputDataBit(IO145_GPIO_Port,IO4_Pin) == RESET)//上升沿触发
	  {
       PWM_FLAG_UP = 1;//上升沿标志
       PWM_FLAG_DOWM = 2;//上升沿标志
    }
	  else//下降沿触发
    {
       if((PWM_FLAG_DOWM==2)&&(PWM_FLAG_UP == 1))//判断上一次是否为上升沿
       {
         PWM_FLAG_CNT -= 1; 
				 
       }
       PWM_FLAG_DOWM=1;//下降沿标志
       PWM_FLAG_UP=2;//下降沿标志
    }
		if(!Auto_Down_Cmd && (PWM_FLAG_CNT <= CURRENT_TARGET_CNT))
		{
			 LIFTER_ACTION = 1;//升降杆数据发送
			 LIFTER_DIS;
		}
		//if(Auto_Down_Cmd && (PWM_FLAG_CNT <= (28 * 30)))
		//	LIFTER_DIS;
	}
	EXTI_ClearITPendingBit(EXTI_Line1);
}

