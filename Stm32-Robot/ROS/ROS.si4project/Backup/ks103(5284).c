#ifdef __cplusplus
extern "C" {
#endif
//Sensors_ULtrasonic_driver ks103超声波传感器
#include "ks103.h"
#include "sys.h"
#include "millisecondtimer.h"
	
#define RS485_RX_EN		PEout(9)	//485模式控制.0,使能接收
#define RS485_TX_EN		PEout(8)	//485模式控制.1,使能发送.

u8 uart7_rx_buf[2];
u8  usartlen;
char usart7_ch;
u16 Ultr_distance;	
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
	
	//PE8\PE9推挽输出，485模式控制  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9; //GPIOE8 GPIOE9
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOE,&GPIO_InitStructure); //初始化PE8\PE9
	
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
	
	RS485_TX_EN = 1;//默认发送
}

void	UART7_IRQHandler(void)
{
	if(USART_GetITStatus(USART1,USART_IT_RXNE))
	{
		usart7_ch=USART_ReceiveData(UART7);
		uart7_rx_buf[usartlen++]=usart7_ch;
	}	
}

	
void KS103_Send(void)
{	
	  u8	AskCmd[3]={0xE8,0x02,0xBC};
		int i;
		for(i=0;i<3;i++)
		{	
			USART_SendData(UART7, AskCmd[i]);
			while(USART_GetFlagStatus(UART7, USART_FLAG_TXE) == RESET){}; //等待发送完成
		}
	return;
}

void Sensors_Ultrasonic_task(void)
{ 
		if(usartlen == 2)
		{
			RS485_TX_EN=1;				//发送模式
			usartlen = 0;
			Ultr_distance = uart7_rx_buf[0]*256+uart7_rx_buf[1];
			KS103_Send();	//发布探测指令
			RS485_RX_EN=0;//接收模式
			//delay(90);	
		}
}
#ifdef __cplusplus
}
#endif