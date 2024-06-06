#ifdef __cplusplus
extern "C" {
#endif
//Sensors_ULtrasonic_driver ks103超声波传感器
#include "mcu2ult.h"
#include "sys.h"
#include "millisecondtimer.h"

#define RS485_RX_EN PFout(9) //485模式控制.0,使能接收
#define RS485_TX_EN PFout(8) //485模式控制.1,使能发送.
#define UART7_REC_LEN  			20 	//定义最大接收字节数 20
u8 uart7_rx_buf[UART7_REC_LEN];
u8 Ultr_Data[15];
u8  usart7_len;
char usart7_ch;
static u8 Reset;
u16 Ultr_distance_F_left,Ultr_distance_F_right,Ultr_distance_A_left,Ultr_distance_A_right,Ultr_distance_left,Ultr_distance_right;	
u8 Rec_Ok;
	
static void DMA_Use_UART7_Rx_Init(void)
{
    DMA_InitTypeDef DMA_InitStructure;
		/*使能DMA2时钟*/
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

    /* 配置使用DMA接收数据 */
    DMA_DeInit(DMA1_Stream3); 

    DMA_InitStructure.DMA_Channel             = DMA_Channel_5;               /* 配置DMA通道 */
    DMA_InitStructure.DMA_PeripheralBaseAddr  = (uint32_t)(&(UART7->DR));   /* 源*/
    DMA_InitStructure.DMA_Memory0BaseAddr     = (uint32_t)uart7_rx_buf;      /* 目的 */
    DMA_InitStructure.DMA_DIR                 = DMA_DIR_PeripheralToMemory;    /* 方向 外设--》存储器 */
    DMA_InitStructure.DMA_BufferSize          = UART7_REC_LEN;                    /* 长度 */                  
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

    DMA_Init(DMA1_Stream3, &DMA_InitStructure);

    USART_DMACmd(UART7,USART_DMAReq_Rx,ENABLE);

    /* 使能DMA */ 
    DMA_Cmd(DMA1_Stream3,ENABLE);
}
void UART7_Init(u32 bound)
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
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOF,&GPIO_InitStructure); //初始化PE8\PE9
	
	USART_InitStructure.USART_BaudRate=bound;//波特率
	USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;//无硬件流控
	USART_InitStructure.USART_Mode=USART_Mode_Tx|USART_Mode_Rx;//模式
	USART_InitStructure.USART_Parity=USART_Parity_No;//校验
	USART_InitStructure.USART_StopBits=USART_StopBits_1;//停止位
	USART_InitStructure.USART_WordLength=USART_WordLength_8b;//数据长度
	
	USART_Init(UART7,&USART_InitStructure);//③ 初始化串口相关参数
	
	USART_Cmd(UART7,ENABLE);//使能串口7
	
	//USART_ITConfig(UART7,USART_IT_RXNE,ENABLE);//开启接收中断
	USART_ITConfig(UART7, USART_IT_IDLE, ENABLE);  // 开启串口IDLE中断
	USART_GetFlagStatus(UART7,USART_FLAG_TC);
	NVIC_InitStructure.NVIC_IRQChannel=UART7_IRQn;//通道
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;//开启中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=6;//设置中断优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0;//子优先级
	NVIC_Init(&NVIC_InitStructure);
	DMA_Use_UART7_Rx_Init();
	
	RS485_RX_EN = 0;//默认接收
	RS485_TX_EN = 0;
}
uint16_t len = 0;  
static uint8_t deal_irq_rx_end(uint8_t *buf)  
{     
    
    /* 接收完成中断 */
    if(USART_GetITStatus(UART7, USART_IT_IDLE) != RESET)  
    {  
        UART7->SR;  
        UART7->DR; /*清除USART_IT_IDLE标志*/
        /* 关闭接收DMA  */
        DMA_Cmd(DMA1_Stream3,DISABLE);  
        /*清除标志位*/
        DMA_ClearFlag(DMA1_Stream3,DMA_FLAG_TCIF3);  

        /*获得接收帧帧长*/
        len = UART7_REC_LEN - DMA_GetCurrDataCounter(DMA1_Stream3);  
				if((len == 15)&&((uart7_rx_buf[0] == 0xA5)&&(uart7_rx_buf[1] == 0x5A)))
					memcpy(buf,uart7_rx_buf,len);  
				Reset = 0;
			  Rec_Ok = 1;
        /*设置传输数据长度*/
        DMA_SetCurrDataCounter(DMA1_Stream3,UART7_REC_LEN);  
        /* 打开DMA */
        DMA_Cmd(DMA1_Stream3,ENABLE);  
        return len;  
    }   

    return 0;  
}
void	UART7_IRQHandler(void)
{
	#if 0
	if(USART_GetITStatus(UART7,USART_IT_RXNE))
	{
		usart7_ch=USART_ReceiveData(UART7);
		uart7_rx_buf[usart7_len++]=usart7_ch;
		if(usart7_len == 2)
		{
			Reset = 0;
			memcpy(Ultr_Data,uart7_rx_buf,usart7_len); 
			Rec_Ok = 1;
      usart7_len	= 0;		
		}
		  
	}
  #endif
  deal_irq_rx_end(Ultr_Data);	
}
uint8_t CHeck_Sum_Ult(uint8_t *data,uint8_t length)
{
	uint8_t sum=0,i=0;

	if(length>15)//超过上传数据
		return 0;
	for(i=0;i<length-1;i++)
	 sum+=data[i];
	if(sum==data[length-1])
		return 1;
	else
    return 0;
}
int Sensors_Ultrasonic_task(void)
{ 
    if(Rec_Ok)
		{
			Rec_Ok = 0;
			if(CHeck_Sum_Ult(Ultr_Data,len))
			{
				Ultr_distance_F_left = Ultr_Data[2]*256 + Ultr_Data[3];//前方左侧超声波
				Ultr_distance_F_right = Ultr_Data[4]*256 + Ultr_Data[5];//前方右侧超声波
				Ultr_distance_left =  Ultr_Data[6]*256 + Ultr_Data[7]; //左方探坑超声波
				Ultr_distance_right =  Ultr_Data[8]*256 + Ultr_Data[9];//右方探坑超声波
				Ultr_distance_A_left = Ultr_Data[10]*256 + Ultr_Data[11];//后方左侧超声波
				Ultr_distance_A_right = Ultr_Data[12]*256 + Ultr_Data[13];//后方右侧超声波
			}
		}
}