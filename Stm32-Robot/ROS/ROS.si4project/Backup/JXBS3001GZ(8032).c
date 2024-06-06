#ifdef __cplusplus
extern "C" {
#endif
//光强度检测传感器JXBS-3001-GZ
#include "JXBS3001GZ.h"
#include "sys.h"
#include "millisecondtimer.h"

#define RS485_RX_EN		PGout(7)	//485模式控制.0,使能接收
#define RS485_TX_EN		PGout(6)	//485模式控制.1,使能发送.
/************************************************
 
************************************************/

u8 crc_H,crc_L;
u8 usart6_rx_buf[10];
float Light;
u8  usartlen;
char usart6_ch;

union data
{
	unsigned char A[4];
	float Temp;
};

void JXBS3001GZ_Init(u32 bound)
{
	GPIO_InitTypeDef  GPIO_InitStructure;   /*定义相关结构体*/
	USART_InitTypeDef  USART_InitStructure;
	NVIC_InitTypeDef  NVIC_InitStructure;   
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);//① 使能相应的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6,ENABLE);
	
	//串口6引脚复用映射
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_USART6); //GPIOC6复用为USART6
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_USART6); //GPIOC7复用为USART6
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOC,&GPIO_InitStructure);//②  初始化相应的IO口模式
	
	USART_InitStructure.USART_BaudRate=bound;//波特率
	USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;//无硬件流控
	USART_InitStructure.USART_Mode=USART_Mode_Tx|USART_Mode_Rx;//模式
	USART_InitStructure.USART_Parity=USART_Parity_No;//校验
	USART_InitStructure.USART_StopBits=USART_StopBits_1;//停止位
	USART_InitStructure.USART_WordLength=USART_WordLength_8b;//数据长度
	
	USART_Init(USART6,&USART_InitStructure);//③ 初始化串口相关参数
	
	USART_Cmd(USART6,ENABLE);//使能串口6
	
	USART_ITConfig(USART6,USART_IT_RXNE,ENABLE);//开启接收中断
	
	NVIC_InitStructure.NVIC_IRQChannel=USART6_IRQn;//通道
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;//开启中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;//设置中断优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=1;//子优先级
	NVIC_Init(&NVIC_InitStructure);
	RS485_TX_EN = 1;
	numinit();
}

void	USART6_IRQHandler(void)
{
	if(USART_GetITStatus(USART6,USART_IT_RXNE))
	{
		
		usart6_ch=USART_ReceiveData(USART6);
		usart6_rx_buf[usartlen++]=usart6_ch;
	}	
}

	
void JXBS3001GZ_Send(u8 *buf)
{	
		int i;
		for(i=0;i<8;i++)
		{	
		
			USART_SendData(USART6, buf[i]);
			while(USART_GetFlagStatus(USART6, USART_FLAG_TXE) == RESET){}; //等待发送完成
		}
		RS485_RX_EN = 0;
	return;
}

void numinit()
{
	int h;
	for( h=0;h<10;h++)
	{
		usart6_rx_buf[h]=0;
	}
}

int calculate(void)
{	
	union data  a;
	u8  AskCmd[8]={0x01,0x03,0x00,0x07,0x00,0x02,0x75,0xca};
 	if(usartlen>=9)
	{
		JXBS3001GZ_Send(AskCmd);
		a.A[0]=usart6_rx_buf[3];
		a.A[1]=usart6_rx_buf[4];
		a.A[2]=usart6_rx_buf[5];
		a.A[3]=usart6_rx_buf[6];
		Light=a.Temp;
		usartlen=0;	
		return 0;
	}	
	return 1;
}	
#ifdef __cplusplus
}
#endif