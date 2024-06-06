#ifdef __cplusplus
extern "C" {
#endif
//Sensors_ULtrasonic_driver ks103超声波传感器
#include "ks103.h"
#if 0
u8  AskCmd[3]={0xE8,0x02,0xBC};
u8 uart7_rx_buf[2];
u8  usartlen1;
char usart7_ch;
void KS103_Init(u32 bound)
{
	GPIO_InitTypeDef  GPIO_InitStructure;   /*定义相关结构体*/
	USART_InitTypeDef  USART_InitStructure;
	NVIC_InitTypeDef  NVIC_InitStructure;   
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);//① 使能相应的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6,ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOF,&GPIO_InitStructure);//②  初始化相应的IO口模式
	
	USART_InitStructure.USART_BaudRate=bound;//波特率
	USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;//无硬件流控
	USART_InitStructure.USART_Mode=USART_Mode_Tx|USART_Mode_Rx;//模式
	USART_InitStructure.USART_Parity=USART_Parity_No;//校验
	USART_InitStructure.USART_StopBits=USART_StopBits_1;//停止位
	USART_InitStructure.USART_WordLength=USART_WordLength_8b;//数据长度
	
	USART_Init(UART7,&USART_InitStructure);//③ 初始化串口相关参数
	
	USART_Cmd(UART7,ENABLE);//使能串口1
	
	USART_ITConfig(UART7,USART_IT_RXNE,DISABLE);//开启接收中断
	
	NVIC_InitStructure.NVIC_IRQChannel=UART7_IRQn;//通道
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;//开启中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;//设置中断优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=1;//子优先级
	NVIC_Init(&NVIC_InitStructure);
}

void	UART7_IRQHandler(void)
{
	if(USART_GetITStatus(USART1,USART_IT_RXNE))
	{
		
		usart7_ch=USART_ReceiveData(UART7);
		uart7_rx_buf[usartlen1++]=usart7_ch;
	}	
}

	
void KS103_Send(u8 *buf)
{	
		int i;
		for(i=0;i<3;i++)
		{	
			USART_SendData(UART7, buf[i]);
			while(USART_GetFlagStatus(UART7, USART_FLAG_TXE) == RESET){}; //等待发送完成
		}
	return;
}

void Sensors_Ultrasonic_task(void)
{ 
	  u16 Val,i;
	  float Cval,Dval,Mval;
		KS103_Send(AskCmd);	
	  //delay_ms(20);	
	  RS485_TX_EN=0;//脨脼赂脛脦陋陆脫脢脮脛拢脢陆
		//麓脣麓娄脩脫脢卤脳卯脡脵87ms		
		delay_ms(90);	
		while(usartlen1 != 2)
		{
			KS103_Send(AskCmd1[0],AskCmd1[1],AskCmd1[2]);	
			delay_ms(90);	
			i++;
			if((usartlen1 == 2)||(i>100))break;
		}
		i = 0;
		{
			RS485_TX_EN=1;				//路垄脣脥脛拢脢陆
			usartlen1 = 0;
			Val=usart2_rx_buf[0]*256+usart2_rx_buf[1];
			Cval = Val/10.0;
      Dval = Val/100.0;
			Mval = Val/1000.0;
			if(Val < 10)
				printf("Val = %dmm\n\r",Val);
			else if((Val >= 10)&&(Val < 100))
				printf("Val = %fcm\n\r",Cval);
			else if((Val >= 100)&&(Val < 1000))
				printf("Val = %fdm\n\r",Dval);
      else if((Val >= 1000))
				printf("Val = %fm\n\r",Mval);	
		}		
}
#endif
#ifdef __cplusplus
}
#endif