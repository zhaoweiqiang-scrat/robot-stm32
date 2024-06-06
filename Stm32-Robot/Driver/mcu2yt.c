#ifdef __cplusplus
extern "C" {
#endif
#include "mcu2yt.h"
#include "millisecondtimer.h"
#include "delay.h"
#define USART4_REC_LEN  			20 
typedef union {
        float real;
        uint8_t base[4];
      } angle;
angle Vangle,Hangle;
float Height,V_angle,H_angle;
u8  usart4_len;
u8 DMA_Rx_Angel_Flag,DMA_Rx_Lifter_Flag,DMA_Rx_Vangel_Flag,DMA_Rx_Hangel_Flag,Height_Flag1,Height_Flag2,Yt_Flag1,Yt_Flag2,Lifter_Ack,Yt_Ack,Led_Ack;
char usart4_ch;
u8 uart4_buf[USART4_REC_LEN];
u8 uart4_rx_buf[USART4_REC_LEN];
uint8_t uart4_tx_buf[20];

static void DMA_Use_UART4_Rx_Init(void)
{
    DMA_InitTypeDef DMA_InitStructure;
		/*使能DMA1时钟*/
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

    /* 配置使用DMA接收数据 */
    DMA_DeInit(DMA1_Stream2); 

    DMA_InitStructure.DMA_Channel             = DMA_Channel_4;               /* 配置DMA通道 */
    DMA_InitStructure.DMA_PeripheralBaseAddr  = (uint32_t)(&(UART4->DR));   /* 源*/
    DMA_InitStructure.DMA_Memory0BaseAddr     = (uint32_t)uart4_rx_buf;      /* 目的 */
    DMA_InitStructure.DMA_DIR                 = DMA_DIR_PeripheralToMemory;    /* 方向 外设--》存储器 */
    DMA_InitStructure.DMA_BufferSize          = USART4_REC_LEN;                    /* 长度 */                  
    DMA_InitStructure.DMA_PeripheralInc       = DMA_PeripheralInc_Disable;    /* 外设地址是否自增 */
    DMA_InitStructure.DMA_MemoryInc           = DMA_MemoryInc_Enable;         /* 内存地址是否自增 */
    DMA_InitStructure.DMA_PeripheralDataSize  = DMA_MemoryDataSize_Byte;      
    DMA_InitStructure.DMA_MemoryDataSize      = DMA_MemoryDataSize_Byte;     
    DMA_InitStructure.DMA_Mode                = DMA_Mode_Normal;              
    DMA_InitStructure.DMA_Priority            = DMA_Priority_Medium;        
    DMA_InitStructure.DMA_FIFOMode            = DMA_FIFOMode_Disable;          
    DMA_InitStructure.DMA_FIFOThreshold       = DMA_FIFOThreshold_HalfFull; 
    DMA_InitStructure.DMA_MemoryBurst         = DMA_MemoryBurst_Single;      
    DMA_InitStructure.DMA_PeripheralBurst     = DMA_PeripheralBurst_Single;

    DMA_Init(DMA1_Stream2, &DMA_InitStructure);

    USART_DMACmd(UART4,USART_DMAReq_Rx,ENABLE);

    /* 使能DMA */ 
    DMA_Cmd(DMA1_Stream2,ENABLE);
}
//DMAx的各通道配置
//这里的传输形式是固定的,这点要根据不同的情况来修改
//从存储器->外设模式/8位数据宽度/存储器增量模式
//DMA_Streamx:DMA数据流,DMA1_Stream0~7/DMA2_Stream0~7
//chx:DMA通道选择,@ref DMA_channel DMA_Channel_0~DMA_Channel_7
//par:外设地址
//mar:存储器地址
//ndtr:数据传输量  
//MYDMA_Config(DMA1_Stream6,DMA_Channel_4,(u32)&USART2->DR,(u32)USART_RX_BUF,USART_REC_LEN);//DMA1,STEAM6,CH4,外设为串口2,存储器为SendBuff,长度为:SEND_BUF_SIZE.
void MYDMA_UART4_Config()
{ 
 
	DMA_InitTypeDef  DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);//DMA1时钟使能 
	
  DMA_DeInit(DMA1_Stream4);
	
	while (DMA_GetCmdStatus(DMA1_Stream4) != DISABLE){}//等待DMA可配置 
	
  /* 配置 DMA Stream */
  DMA_InitStructure.DMA_Channel = DMA_Channel_4;  //通道选择
  DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&UART4->DR;//DMA外设地址
  DMA_InitStructure.DMA_Memory0BaseAddr = (u32)uart4_tx_buf;//DMA 存储器0地址
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;//存储器到外设模式
  DMA_InitStructure.DMA_BufferSize = USART4_REC_LEN;//数据传输量 
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设非增量模式
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//存储器增量模式
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//外设数据长度:8位
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//存储器数据长度:8位
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;// 使用普通模式 
  DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;//中等优先级
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;//存储器突发单次传输
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//外设突发单次传输
  DMA_Init(DMA1_Stream4, &DMA_InitStructure);//初始化DMA Stream
	
	DMA_ITConfig(DMA1_Stream4, DMA_IT_TC, ENABLE);            // 开启发送DMA通道中断
		
	/* 配置DMA中断优先级*/
  NVIC_InitStructure.NVIC_IRQChannel                   = DMA1_Stream4_IRQn;           
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;          
  NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 1; 
  NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}
void UART4_Init(u32 bound)
{
	GPIO_InitTypeDef  GPIO_InitStructure;   /*定义相关结构体*/
	USART_InitTypeDef  USART_InitStructure;
	NVIC_InitTypeDef  NVIC_InitStructure;   
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);//① 使能相应的时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,ENABLE);
	
	//串口4引脚复用映射
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource10,GPIO_AF_UART4); //GPIOC10复用为UART4
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource11,GPIO_AF_UART4); //GPIOC11复用为UART4

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;//GPIOC10 GPIOC11
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
	
	USART_Init(UART4,&USART_InitStructure);//③ 初始化串口相关参数
	
	
	USART_ClearFlag(UART4, USART_FLAG_TC);
	
	//USART_ITConfig(UART4,USART_IT_RXNE,ENABLE);//开启接收中断   ENABLE  DISABLE
	USART_ITConfig(UART4,USART_IT_IDLE,ENABLE);
	USART_GetFlagStatus(UART4,USART_FLAG_TC);
	USART_Cmd(UART4,ENABLE);//使能串口4
  
	NVIC_InitStructure.NVIC_IRQChannel=UART4_IRQn;//通道
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;//开启中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;//设置中断优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0;//子优先级
	NVIC_Init(&NVIC_InitStructure);
	
  //串口4接收DMA配置
	DMA_Use_UART4_Rx_Init();
	//串口4发送DMA配置
	MYDMA_UART4_Config();
}
static void delay_um(uint32_t delay)
{
	while(delay--);
}

static uint8_t deal_irq_rx_end(uint8_t *buf)  
{   
    uint16_t len = 0;	
    /* 接收完成中断 */
    if(USART_GetITStatus(UART4, USART_IT_IDLE) != RESET)  
    {  
				delay_um(500000);//不加此延时，接收得数据有异常
        UART4->SR;  
        UART4->DR; /*清除USART_IT_IDLE标志*/
			  /* 关闭接收DMA  */
        DMA_Cmd(DMA1_Stream2,DISABLE);  
        /*清除标志位*/
        DMA_ClearFlag(DMA1_Stream2,DMA_FLAG_TCIF2);  
				
        /*获得接收帧帧长*/
        len = USART4_REC_LEN - DMA_GetCurrDataCounter(DMA1_Stream2); 
				if((uart4_rx_buf[0]==0x5a)&&(uart4_rx_buf[1]==0xa5))
					DMA_Rx_Lifter_Flag = 1;
				if((uart4_rx_buf[0]==0xcf)&&(uart4_rx_buf[1] == 0X01))//角度协议 0xcf 0x01/0x02(0x01代表俯仰角度、0x02代表水平角度) 0x00 0x00 0x00 0x00(后四位代表浮点数)
					DMA_Rx_Vangel_Flag = 1;
				if((uart4_rx_buf[0]==0xcf)&&(uart4_rx_buf[1] == 0X02))
					DMA_Rx_Hangel_Flag = 1;
				if((uart4_rx_buf[0]==0xcf)&&(uart4_rx_buf[1] == 0X03))
					DMA_Rx_Angel_Flag = 1;
				if((uart4_rx_buf[0]==0xcf)&&(uart4_rx_buf[1] == 0X5a))
					Lifter_Ack = 1;
				if((uart4_rx_buf[0]==0xcf)&&(uart4_rx_buf[1] == 0X05))
					Yt_Ack = 1;
				if((uart4_rx_buf[0]==0xcf)&&(uart4_rx_buf[1] == 0X06))
					Led_Ack = 1;
        memcpy(buf,uart4_rx_buf,len);  
        /*设置传输数据长度*/
        DMA_SetCurrDataCounter(DMA1_Stream2,USART4_REC_LEN);  
        /* 打开DMA */
        DMA_Cmd(DMA1_Stream2,ENABLE);  
        return len;  
    }   
    return 0;  
}

void UART4_IRQHandler(void) 
{
		deal_irq_rx_end(uart4_buf);
}
void LIFTER_Contr(void)
{
	if(Lifter_Ack)
	{
		Lifter_Ack = 0;
		printf("RECEVIE LIFTER ACK!!!\r\n");
		Height_Flag1 = 0;//命令发送成功
		Height_Flag2 = 1;//接收到数据，即命令已发送成功
	}
	if((uart4_buf[0]==0X5A)&&(uart4_buf[1]==0XA5)&&DMA_Rx_Lifter_Flag)
	{
			DMA_Rx_Lifter_Flag = 0;
			Height = (((uart4_buf[2]<<8)|uart4_buf[3]))/28.0;
			//printf("升降杆脉冲数: 0x%x,升降杆高度：%fmm\r\n",(((uart4_buf[2]<<8)|uart4_buf[3])),Height);
	}
}
void YT_Contr(void)
{
	uint8_t i=0;
	if(Yt_Ack)
	{
		printf("RECEVIE ANGLE ACK!!!\r\n");
		Yt_Ack = 0;
		Yt_Flag1 = 0;//接收到数据，即命令已发送成功
		Yt_Flag2 = 1;
	}
	#if 0
	if((uart4_buf[0]==0XCF)&&(uart4_buf[1]==0X01)&&DMA_Rx_Vangel_Flag)
	{
			DMA_Rx_Vangel_Flag = 0;
			for(i = 0;i < 4;i++)
			{
				Vangle.base[i] = uart4_buf[5-i];
			}
			V_angle = Vangle.real; 
			
			printf("俯仰角度：%f\r\n",Vangle.real);
	}
	if((uart4_buf[0]==0XCF)&&(uart4_buf[1]==0X02)&&DMA_Rx_Hangel_Flag)
	{
			DMA_Rx_Hangel_Flag = 0;
			for(i = 0;i < 4;i++)
			{
				Hangle.base[i] = uart4_buf[5-i];
			}
			H_angle = Hangle.real + 33.0;
			printf("水平角度：%f\r\n",Hangle.real);
	}
	#endif
	if((uart4_buf[0]==0XCF)&&(uart4_buf[1]==0X03)&&DMA_Rx_Angel_Flag)
	{
			DMA_Rx_Angel_Flag = 0;
		  //for(i = 0;i < 4;i++)
			if(uart4_buf[2] == 0x59)
			{
				H_angle = ((uart4_buf[3]<<8) | uart4_buf[4])/100.0;
				printf("水平角度：%f\r\n",H_angle);
			}
		
			if(uart4_buf[2] == 0x5b)
			{
				if(uart4_buf[3] & 0x80)
				{
					uart4_buf[3] ^= 0xff;
					uart4_buf[4] ^= 0xff;
					uart4_buf[4] += 0x01;
					V_angle = ((uart4_buf[3]<<8) | uart4_buf[4])/100.0;
					V_angle = (-1) * V_angle;
				}
				else{
					V_angle = ((uart4_buf[3]<<8) | uart4_buf[4])/100.0;
				}
				printf("俯仰角度：%f\r\n",V_angle);
			}
	}
}



void DMA1_Stream4_IRQHandler(void)
{
    if(DMA_GetFlagStatus(DMA1_Stream4,DMA_FLAG_TCIF4)!=RESET)
    {
      DMA_ClearFlag(DMA1_Stream4,DMA_FLAG_TCIF4);//清除DMA1_Steam6传输完成标志
			DMA_Cmd(DMA1_Stream4, DISABLE); //关闭DMA传输 
    }
}

//开启一次DMA传输
//DMA_Streamx:DMA数据流,DMA1_Stream0~7/DMA2_Stream0~7 
//ndtr:数据传输量  
void MYDMA_UART4_Enable(uint8_t ndtr)
{
 
	DMA_Cmd(DMA1_Stream4, DISABLE);                      //关闭DMA传输 	
	while (DMA_GetCmdStatus(DMA1_Stream4) != DISABLE){}	//确保DMA可以被设置  
	DMA_SetCurrDataCounter(DMA1_Stream4,ndtr);          //数据传输量  
	DMA_Cmd(DMA1_Stream4, ENABLE);                      //开启DMA传输 
}	  
uint8_t CHeck_Sum(uint8_t *data,uint8_t len)
{
	uint8_t sum=0,i=0;

	for(i=0;i<len;i++)
	 sum+=data[i];
	
	return sum;
}
void MCU2YT(uint8_t *Buffer, uint8_t Length)
{
	uint8_t i=0;
	
	
	memcpy(uart4_tx_buf,Buffer,Length); 
	uart4_tx_buf[Length] = CHeck_Sum(Buffer,Length);
	#if 1
	USART_DMACmd(UART4,USART_DMAReq_Tx,ENABLE);  //使能串口2的DMA发送 
	MYDMA_UART4_Enable(Length+1);     //开始一次DMA传输！
	#else
	while(i<Length)
	{
		printf("数据%d：0x%x",i,Buffer[i]);
		UART4->SR;
		USART_SendData(UART4, Buffer[i++]);
	  while(USART_GetFlagStatus(UART4,USART_FLAG_TC)==RESET);//等待发送完成
	}
	#endif
}
#ifdef __cplusplus
}
#endif