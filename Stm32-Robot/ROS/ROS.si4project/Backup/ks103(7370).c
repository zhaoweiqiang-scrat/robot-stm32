#ifdef __cplusplus
extern "C" {
#endif
//Sensors_ULtrasonic_driver ks103超声波传感器
#include "ks103.h"
#include "sys.h"
#include "millisecondtimer.h"
#if 0 
#define RS485_RX_EN		GPIO_ResetBits(GPIOF,GPIO_Pin_9)	//485模式控制.0,使能接收
#define RS485_RX_DIS	GPIO_ResetBits(GPIOF,GPIO_Pin_9)
#define RS485_TX_EN		GPIO_SetBits(GPIOF,GPIO_Pin_8)	//485模式控制.1,使能发送.
#define RS485_TX_DIS	GPIO_ResetBits(GPIOF,GPIO_Pin_8)
#endif
#define RS485_RX_EN PFout(9)
#define RS485_TX_EN PFout(8)
u8 uart7_rx_buf[2];
u8  usart7_len;
char usart7_ch;
u16 Ultr_distance_left,Ultr_distance_right;	

void KS103_Init(u32 bound)
{
	GPIO_InitTypeDef  GPIO_InitStructure;   /*定义相关结构体*/
	USART_InitTypeDef  USART_InitStructure;
	NVIC_InitTypeDef  NVIC_InitStructure;   
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF,ENABLE);//① 使能相应的时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART7,ENABLE);
	
	//串口7引脚复用映射
	GPIO_PinAFConfig(GPIOF,GPIO_PinSource6,GPIO_AF_UART7); //GPIOF6复用为UART7
	GPIO_PinAFConfig(GPIOF,GPIO_PinSource7,GPIO_AF_UART7); //GPIOF7复用为UART7
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;//GPIOF6 GPIOF7
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOF,&GPIO_InitStructure);//②  初始化相应的IO口模式
	
	//PF8\PF9推挽输出，485模式控制  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9; //GPIOF8 GPIOF9
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD; //推挽输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOF,&GPIO_InitStructure); //初始化PE8\PE9
	
	USART_InitStructure.USART_BaudRate=bound;//波特率
	USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;//无硬件流控
	USART_InitStructure.USART_Mode=USART_Mode_Tx|USART_Mode_Rx;//模式
	USART_InitStructure.USART_Parity=USART_Parity_No;//校验
	USART_InitStructure.USART_StopBits=USART_StopBits_1;//停止位
	USART_InitStructure.USART_WordLength=USART_WordLength_8b;//数据长度
	
	USART_Init(UART7,&USART_InitStructure);//③ 初始化串口相关参数
	
	USART_Cmd(UART7,ENABLE);//使能串口1
	
	USART_ITConfig(UART7,USART_IT_RXNE,ENABLE);//开启接收中断
	
	NVIC_InitStructure.NVIC_IRQChannel=UART7_IRQn;//通道
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;//开启中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;//设置中断优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=1;//子优先级
	NVIC_Init(&NVIC_InitStructure);
	
	RS485_RX_EN = 1;//默认发送
	RS485_TX_EN = 1;
}

void	UART7_IRQHandler(void)
{
	if(USART_GetITStatus(UART7,USART_IT_RXNE))
	{
		usart7_ch=USART_ReceiveData(UART7);
		uart7_rx_buf[usart7_len++]=usart7_ch;
	}	
}

	
void KS103_Send(u8 *buf)
{	
		int i;
	  RS485_TX_EN = 1;				//发送模式
		RS485_RX_EN = 1;
		for(i=0;i<3;i++)
		{	
			USART_SendData(UART7, buf[i]);
			while(USART_GetFlagStatus(UART7, USART_FLAG_TXE) == RESET){}; //等待发送完成
		}
		delay(1000);
		RS485_RX_EN = 0;//接收模式
		RS485_TX_EN = 0;
	return;
}

int Sensors_Ultrasonic_task1(void)
{ 
    u8	AskCmd2[3]={0xEA,0x02,0xBC};
	u8	AskCmd1[3]={0xE8,0x02,0xBC};
		if(usart7_len == 2)
		{
			RS485_TX_EN = 1;				//发送模式
			RS485_RX_EN = 1;
			usart7_len = 0;
			Ultr_distance_left = uart7_rx_buf[0]*256+uart7_rx_buf[1];
			KS103_Send(AskCmd1);	//发布探测指令
			//RS485_RX_EN=0;//接收模式
			//delay(90);	
			return 0;
		}
		return 1;
}
int Sensors_Ultrasonic_task2(void)
{ 
    u8	AskCmd1[3]={0xE8,0x02,0xBC};
	u8 Flag;
		if(usart7_len == 2)
		{
			RS485_TX_EN = 1;				//发送模式
			RS485_RX_EN = 1;
			usart7_len = 0;
			Ultr_distance_right = uart7_rx_buf[0]*256+uart7_rx_buf[1];
			KS103_Send(AskCmd1);	//发布探测指令
			//RS485_RX_EN=0;//接收模式
			//delay(90);	
			return 0;
		}
		return 1;
}
#ifdef __cplusplus
}
#endif