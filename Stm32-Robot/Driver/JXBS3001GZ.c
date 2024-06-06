#ifdef __cplusplus
extern "C" {
#endif
//光强度检测传感器JXBS-3001-GZ
#include "JXBS3001GZ.h"
#include "sys.h"
#include "millisecondtimer.h"

#define RS485_RX_EN PGout(7) //485模式控制.0,使能接收
#define RS485_TX_EN PGout(6) //485模式控制.1,使能发送.

#define USART6_REC_LEN  			20 	//定义最大接收字节数 200
/************************************************
 
************************************************/

u8 crc_H,crc_L;
u8 usart6_buf[USART6_REC_LEN];
u8 usart6_rx_buf[USART6_REC_LEN];
u32 Light;
u8  usartlen;
char usart6_ch;
static u8 Reset;
union data
{
	unsigned char A[4];
	u32 Temp;
};
static void DMA_Use_USART6_Rx_Init(void)
{
    DMA_InitTypeDef DMA_InitStructure;
		/*使能DMA2时钟*/
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

    /* 配置使用DMA接收数据 */
    DMA_DeInit(DMA2_Stream2); 

    DMA_InitStructure.DMA_Channel             = DMA_Channel_5;               /* 配置DMA通道 */
    DMA_InitStructure.DMA_PeripheralBaseAddr  = (uint32_t)(&(USART6->DR));   /* 源*/
    DMA_InitStructure.DMA_Memory0BaseAddr     = (uint32_t)usart6_buf;      /* 目的 */
    DMA_InitStructure.DMA_DIR                 = DMA_DIR_PeripheralToMemory;    /* 方向 外设--》存储器 */
    DMA_InitStructure.DMA_BufferSize          = USART6_REC_LEN;                    /* 长度 */                  
    DMA_InitStructure.DMA_PeripheralInc       = DMA_PeripheralInc_Disable;    /* 外设地址是否自增 */
    DMA_InitStructure.DMA_MemoryInc           = DMA_MemoryInc_Enable;         /* 内存地址是否自增 */
    DMA_InitStructure.DMA_PeripheralDataSize  = DMA_MemoryDataSize_Byte;      
    DMA_InitStructure.DMA_MemoryDataSize      = DMA_MemoryDataSize_Byte;     
    DMA_InitStructure.DMA_Mode                = DMA_Mode_Normal;              
    DMA_InitStructure.DMA_Priority            = DMA_Priority_VeryHigh;        
    DMA_InitStructure.DMA_FIFOMode            = DMA_FIFOMode_Disable;          
    DMA_InitStructure.DMA_FIFOThreshold       = DMA_FIFOThreshold_HalfFull; 
    DMA_InitStructure.DMA_MemoryBurst         = DMA_MemoryBurst_Single;      
    DMA_InitStructure.DMA_PeripheralBurst     = DMA_PeripheralBurst_Single;

    DMA_Init(DMA2_Stream2, &DMA_InitStructure);

    USART_DMACmd(USART6,USART_DMAReq_Rx,ENABLE);

    /* 使能DMA */ 
    DMA_Cmd(DMA2_Stream2,ENABLE);
}
void numinit()
{
	int h;
	for( h=0;h<10;h++)
	{
		usart6_rx_buf[h]=0;
	}
}
void JXBS3001GZ_Init(u32 bound)
{
	GPIO_InitTypeDef  GPIO_InitStructure;   /*定义相关结构体*/
	USART_InitTypeDef  USART_InitStructure;
	NVIC_InitTypeDef  NVIC_InitStructure;   
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOG,ENABLE);//① 使能相应的时钟
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
	
	//PG6\PG7推挽输出，485模式控制  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; //GPIOG6 GPIOG7
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOG,&GPIO_InitStructure); //初始化PE8\PE9
	
	USART_InitStructure.USART_BaudRate=bound;//波特率
	USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;//无硬件流控
	USART_InitStructure.USART_Mode=USART_Mode_Tx|USART_Mode_Rx;//模式
	USART_InitStructure.USART_Parity=USART_Parity_No;//校验
	USART_InitStructure.USART_StopBits=USART_StopBits_1;//停止位
	USART_InitStructure.USART_WordLength=USART_WordLength_8b;//数据长度
	
	USART_Init(USART6,&USART_InitStructure);//③ 初始化串口相关参数
	//USART_ITConfig(USART6,USART_IT_RXNE,ENABLE);//开启接收中断
	USART_ITConfig(USART6, USART_IT_IDLE, ENABLE);  // 开启串口IDLE中断
	USART_GetFlagStatus(USART6, USART_FLAG_TC);
	
	USART_Cmd(USART6,ENABLE);//使能串口6
	
	NVIC_InitStructure.NVIC_IRQChannel=USART6_IRQn;//通道
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;//开启中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;//设置中断优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0;//子优先级
	NVIC_Init(&NVIC_InitStructure);
	DMA_Use_USART6_Rx_Init();
	RS485_TX_EN = 1;				//发送模式
	RS485_RX_EN = 1;
	numinit();
}
static uint8_t deal_irq_rx_end(uint8_t *buf)  
{     
    uint16_t len = 0;  
    /* 接收完成中断 */
    if(USART_GetITStatus(USART6, USART_IT_IDLE) != RESET)  
    {  
        USART6->SR;  
        USART6->DR; /*清除USART_IT_IDLE标志*/
        /* 关闭接收DMA  */
        DMA_Cmd(DMA2_Stream2,DISABLE);  
        /*清除标志位*/
        DMA_ClearFlag(DMA2_Stream2,DMA_FLAG_TCIF2);  

        /*获得接收帧帧长*/
        len = USART6_REC_LEN - DMA_GetCurrDataCounter(DMA2_Stream2);  
        memcpy(buf,usart6_buf,len);  
				Reset = 0;
        /*设置传输数据长度*/
        DMA_SetCurrDataCounter(DMA2_Stream2,USART6_REC_LEN);  
        /* 打开DMA */
        DMA_Cmd(DMA2_Stream2,ENABLE);  
        return len;  
    }   

    return 0;  
}
void	USART6_IRQHandler(void)
{
	usartlen = deal_irq_rx_end(usart6_rx_buf);
}

	
void JXBS3001GZ_Send(u8 *buf)
{	
		int i;
		RS485_TX_EN = 1;				//发送模式
		RS485_RX_EN = 1;
		for(i=0;i<8;i++)
		{	
			USART_SendData(USART6, buf[i]);
			while(USART_GetFlagStatus(USART6, USART_FLAG_TC) == RESET){}; //等待发送完成
		}
		RS485_RX_EN = 0;//接收模式
		RS485_TX_EN = 0;
	return;
}

int calculate(void)
{	
	union data  a;
	u8  AskCmd[8]={0x01,0x03,0x00,0x07,0x00,0x02,0x75,0xca};
	Reset++;
 	if(usartlen>=9)
	{
		JXBS3001GZ_Send(AskCmd);
		a.A[0]=usart6_rx_buf[6];
		a.A[1]=usart6_rx_buf[5];
		a.A[2]=usart6_rx_buf[4];
		a.A[3]=usart6_rx_buf[3];
		Light=a.Temp;
		usartlen=0;	
		return 0;
	}	
	if(Reset >=10)
	{
		Reset = 0;
		usartlen=0;	
		JXBS3001GZ_Send(AskCmd);
	}
	return 1;
}	

#ifdef __cplusplus
}
#endif