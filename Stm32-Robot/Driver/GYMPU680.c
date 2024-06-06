#ifdef __cplusplus
extern "C" {
#endif
#include "stm32f4xx.h"
#include "string.h"
#include "millisecondtimer.h"
#include "GYMPU680.h"
#define USART5_REC_LEN  			20 	//定义最大接收字节数 200
u8  uart5_len;
char uart5_ch;
u8 uart5_buf[USART5_REC_LEN];
u8 uart5_rx_buf[USART5_REC_LEN];

uint8_t RX_BUF[20]={0},stata=0;
//校验和检查
static void DMA_Use_UART5_Rx_Init(void)
{
    DMA_InitTypeDef DMA_InitStructure;
		/*使能DMA1时钟*/
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

    /* 配置使用DMA接收数据 */
    DMA_DeInit(DMA1_Stream0); 

    DMA_InitStructure.DMA_Channel             = DMA_Channel_4;               /* 配置DMA通道 */
    DMA_InitStructure.DMA_PeripheralBaseAddr  = (uint32_t)(&(UART5->DR));   /* 源*/
    DMA_InitStructure.DMA_Memory0BaseAddr     = (uint32_t)uart5_rx_buf;      /* 目的 */
    DMA_InitStructure.DMA_DIR                 = DMA_DIR_PeripheralToMemory;    /* 方向 外设--》存储器 */
    DMA_InitStructure.DMA_BufferSize          = USART5_REC_LEN;                    /* 长度 */                  
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

    DMA_Init(DMA1_Stream0, &DMA_InitStructure);

    USART_DMACmd(UART5,USART_DMAReq_Rx,ENABLE);

    /* 使能DMA */ 
    DMA_Cmd(DMA1_Stream0,ENABLE);
}

void GYMPU680_Init(uint32_t bound)
{
	GPIO_InitTypeDef GPIO_usartx;
	USART_InitTypeDef Usart_X;
  NVIC_InitTypeDef NVIC_X;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOD, ENABLE);
	
	//串口5引脚复用映射
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource12,GPIO_AF_UART5); //GPIOC12复用为UART5
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource2,GPIO_AF_UART5); //GPIOD2复用为UART5
	
	//UART5_TX   PC.12
  GPIO_usartx.GPIO_Pin = GPIO_Pin_12;
	GPIO_usartx.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_usartx.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_usartx.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_usartx.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOC,&GPIO_usartx);//②  初始化相应的IO口模式
	//UART5_RX	 PD.2
	GPIO_usartx.GPIO_Pin = GPIO_Pin_2;
	GPIO_usartx.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_usartx.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_usartx.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_usartx.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOD,&GPIO_usartx);//②  初始化相应的IO口模式
	
	Usart_X.USART_BaudRate=bound;//波特率
	Usart_X.USART_HardwareFlowControl=USART_HardwareFlowControl_None;//无硬件流控
	Usart_X.USART_Mode=USART_Mode_Tx|USART_Mode_Rx;//模式
	Usart_X.USART_Parity=USART_Parity_No;//校验
	Usart_X.USART_StopBits=USART_StopBits_1;//停止位
	Usart_X.USART_WordLength=USART_WordLength_8b;//数据长度
	
	USART_Init(UART5,&Usart_X);//③ 初始化串口相关参数
  //USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);//开启接收中断
	USART_ITConfig(UART5, USART_IT_IDLE, ENABLE);  // 开启串口IDLE中断
  USART_Cmd(UART5, ENABLE);
  
  /* 4个抢占优先级，4个响应优先级 */
 // NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  /*抢占优先级可打断中断级别低的中断*/
	/*响应优先级按等级执行*/
	NVIC_X.NVIC_IRQChannel = UART5_IRQn;//中断向量
  NVIC_X.NVIC_IRQChannelPreemptionPriority = 7;//抢占优先级
  NVIC_X.NVIC_IRQChannelSubPriority = 0;//响应优先级
  NVIC_X.NVIC_IRQChannelCmd = ENABLE;//使能中断响应
  NVIC_Init(&NVIC_X);
	DMA_Use_UART5_Rx_Init();
}


//input:byte,待发送的数据
static void USART_send_byte(uint8_t byte)
{
	USART_SendData(UART5, byte);	
	while(USART_GetFlagStatus(UART5,USART_FLAG_TC)==RESET);//等待发送完成
}
//发送多字节数据
static void USART_Send_bytes(uint8_t *Buffer, uint8_t Length)
{
	uint8_t i=0;
	while(i<Length)
	{
		USART_send_byte(Buffer[i++]);
	}
}

uint8_t CHeck(uint8_t *data)
{
	uint8_t sum=0,number=0,i=0;
	number=RX_BUF[3]+5;
	if(number>20)//超过上传数据
		return 0;
	for(i=0;i<number-1;i++)
	 sum+=RX_BUF[i];
	if(sum==RX_BUF[number-1])
	{
		memcpy(data,RX_BUF,number);
		return 1;
	}
	else
    return 0;
}

void send_Instruction(void)
{
	uint8_t Set_Cmd[4] = {0xa5,0x55,0x3F,0x39};
	uint8_t Auto_Cmd[4] = {0xa5,0x56,0x02,0xfd};
	uint8_t Query_Cmd[4] = {0xa5,0x56,0x01,0xfc};
	
	USART_Send_bytes(Set_Cmd,4);//发送
	delay(100);
	USART_Send_bytes(Auto_Cmd,4);//发送自动输出指令
	delay(100);
}
static uint8_t deal_irq_rx_end(uint8_t *buf)  
{     
    uint16_t len = 0;  
    /* 接收完成中断 */
    if(USART_GetITStatus(UART5, USART_IT_IDLE) != RESET)  
    {  
        UART5->SR;  
        UART5->DR; /*清除USART_IT_IDLE标志*/
			  stata=1;
        /* 关闭接收DMA  */
        DMA_Cmd(DMA1_Stream0,DISABLE);  
        /*清除标志位*/
        DMA_ClearFlag(DMA1_Stream0,DMA_FLAG_TCIF0);  

        /*获得接收帧帧长*/
        len = USART5_REC_LEN - DMA_GetCurrDataCounter(DMA1_Stream0);  
        memcpy(buf,uart5_rx_buf,len);  

        /*设置传输数据长度*/
        DMA_SetCurrDataCounter(DMA1_Stream0,USART5_REC_LEN);  
        /* 打开DMA */
        DMA_Cmd(DMA1_Stream0,ENABLE);  
        return len;  
    }   

    return 0;  
}
void UART5_IRQHandler(void)
{
	deal_irq_rx_end(RX_BUF);
}
float Temperature ,Humidity;
int calculate_TH()
{
	uint8_t data_buf[20]={0},count=0;
	uint16_t temp1=0;
  int16_t temp2=0;
	
	if(!stata)
		 return 1;
	stata=0;
	if(CHeck(data_buf))
	{
			 count=0;
		   if(data_buf[2]&0x01) //Temperature
			 {
			    temp2=((uint16_t)data_buf[4]<<8|data_buf[5]);   
          Temperature=(float)temp2/100;
          count=2;
			 }
			  if(data_buf[2]&0x02) //Humidity
			 {  
			    temp1=((uint16_t)data_buf[4+count]<<8)|data_buf[5+count];
				  Humidity=(float)temp1/100; 
          count+=2;
			 }
	}
	return 0;
}

#ifdef __cplusplus
}
#endif