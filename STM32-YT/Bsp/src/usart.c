#include "usart.h"
#include "string.h"
#include "stdio.h"

u8 SendBuff1[SEND_BUF_SIZE];	//串口2发送数据缓冲区
u8 SendBuff2[SEND_BUF_SIZE];	//串口3发送数据缓冲区

#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 

}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
    USART1->DR = (u8) ch;      
	return ch;
}
#endif 
//DMA1的各通道配置
//这里的传输形式是固定的,这点要根据不同的情况来修改
//从存储器->外设模式/8位数据宽度/存储器增量模式
//DMA_CHx:DMA通道CHx
//cpar:外设地址
//cmar:存储器地址
//cndtr:数据传输量 
//MYDMA_Config(DMA1_Channel4,(u32)&USART1->DR,(u32)SendBuff,SEND_BUF_SIZE);//DMA1通道4,外设为串口1,存储器为SendBuff,长度SEND_BUF_SIZE.
void MYDMA_Config(DMA_Channel_TypeDef* DMA_CHx,u32 cpar,u32 cmar,u16 cndtr)
{
	DMA_InitTypeDef DMA_InitStructure;
 	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);	//使能DMA传输
	
  DMA_DeInit(DMA_CHx);   //将DMA的通道1寄存器重设为缺省值

	DMA_InitStructure.DMA_PeripheralBaseAddr = cpar;  //DMA外设基地址
	DMA_InitStructure.DMA_MemoryBaseAddr = cmar;  //DMA内存基地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;  //数据传输方向，从内存读取发送到外设
	DMA_InitStructure.DMA_BufferSize = cndtr;  //DMA通道的DMA缓存的大小
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  //外设地址寄存器不变
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  //内存地址寄存器递增
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  //数据宽度为8位
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; //数据宽度为8位
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;  //工作在正常模式
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium; //DMA通道 x拥有中优先级 
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  //DMA通道x没有设置为内存到内存传输
	DMA_Init(DMA_CHx, &DMA_InitStructure);  //根据DMA_InitStruct中指定的参数初始化DMA的通道USART1_Tx_DMA_Channel所标识的寄存器
} 
//开启一次DMA传输
void MYDMA_Enable(DMA_Channel_TypeDef*DMA_CHx)
{ 
	DMA_Cmd(DMA_CHx, DISABLE );  //关闭USART TX DMA1 所指示的通道      
 	DMA_SetCurrDataCounter(DMA_CHx,SEND_BUF_SIZE);//DMA通道的DMA缓存的大小
 	DMA_Cmd(DMA_CHx, ENABLE);  //使能USART TX DMA1 所指示的通道 
}	  

uint8_t RX_BUF[15]={0},RX_BUF1[15]={0},RX_BUF2[15]={0},stata=0;
uint8_t Uart_Rx[UART_RX_LEN];
uint8_t Uart_Rx3[UART_RX_LEN];
void Usart3_Int(uint32_t BaudRatePrescaler)
{
	GPIO_InitTypeDef GPIO_usartx;
	GPIO_InitTypeDef GPIO_rtx;
	USART_InitTypeDef Usart_X;
	NVIC_InitTypeDef NVIC_InitStructure; 
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO|RCC_APB2Periph_GPIOA, ENABLE);
	  //USART3_TX   PB.10
  GPIO_usartx.GPIO_Pin = GPIO_Pin_10;
  GPIO_usartx.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_usartx.GPIO_Mode = GPIO_Mode_AF_PP;//复用推挽输出
  GPIO_Init(GPIOB, &GPIO_usartx); 
  //USART3_RX	  PB.11
  GPIO_usartx.GPIO_Pin = GPIO_Pin_11;
  GPIO_usartx.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
  GPIO_Init(GPIOB, &GPIO_usartx); 
	
	Usart_X.USART_BaudRate=BaudRatePrescaler;
	Usart_X.USART_WordLength=USART_WordLength_8b;//8位数据格式
	Usart_X.USART_StopBits=USART_StopBits_1;//1位停止位
	Usart_X.USART_Parity = USART_Parity_No;//无校验
	Usart_X.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
	Usart_X.USART_Mode= USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART3, &Usart_X);
  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//开启接收中断
	//USART_ITConfig(USART3,USART_IT_IDLE,ENABLE);//开启空闲中断
	
  USART_Cmd(USART3, ENABLE);
	
	//USART_DMACmd(USART3,USART_DMAReq_Rx,ENABLE);//采用DMA方式接收
	/* 使能串口3中断 */
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;	 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	/*IRQ通道使能*/
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	/*根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器USART3*/
	NVIC_Init(&NVIC_InitStructure);
	
	GPIO_rtx.GPIO_Pin = GPIO_Pin_11|GPIO_Pin_12;
  GPIO_rtx.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_rtx.GPIO_Mode = GPIO_Mode_Out_PP;//复用推挽输出
  GPIO_Init(GPIOA, &GPIO_rtx); 
	//MYDMA_Config(DMA1_Channel2,(u32)&USART3->DR,(u32)SendBuff2,SEND_BUF_SIZE);//配置USART3 DMA发送
	//DMA_ITConfig(DMA1_Channel2,DMA_IT_TC,ENABLE);
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel2_IRQn;	 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	/*IRQ通道使能*/
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	/*根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器USART3*/
//	NVIC_Init(&NVIC_InitStructure);
	
	set_rx_tx(0,0);//默认发送
}
void DMA1_Channel2_IRQHandler()
{
	if(DMA_GetFlagStatus(DMA1_FLAG_TC2)!=RESET)
  {
      DMA_ClearFlag(DMA1_FLAG_TC2);//清除DMA1_Steam6传输完成标志
			DMA_Cmd(DMA1_Channel2, DISABLE); //关闭DMA传输 
			set_rx_tx(0,0);//发送完成，改为接收模式
  }
	
}
uint8_t usart3_rebuf[7]={0},usart3_len=0,usart3_ch;
uint8_t YT_Flag_Angle,YT_Flag_Angle1,YT_Flag_Angle2;
void USART3_IRQHandler(void)
{
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
	{
		USART_ClearFlag(USART3,USART_FLAG_RXNE);//清中断标志
		usart3_ch=USART_ReceiveData(USART3);
		usart3_rebuf[usart3_len++]=usart3_ch;
		#if 1
		if(usart3_rebuf[0]!=0xff)//判断帧头
			usart3_len=0;
		if(usart3_len==7)
		{
			if((usart3_rebuf[0]==0xff)&&(usart3_rebuf[1]==0x01)&&(usart3_rebuf[2]==0x00)&&(usart3_rebuf[3]==0x59 || usart3_rebuf[3]==0x5b))
			{
				memcpy(RX_BUF2,usart3_rebuf,usart3_len);
				YT_Flag_Angle = 1;
			}
			stata=1;
			usart3_len=0;
		}
		#endif
	}	
}
void Usart1_Int(uint32_t BaudRatePrescaler)
{
	GPIO_InitTypeDef GPIO_usartx;
	USART_InitTypeDef Usart_X;
	NVIC_InitTypeDef NVIC_InitStructure; 
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO, ENABLE);
	  //USART1_TX   PA.9
  GPIO_usartx.GPIO_Pin = GPIO_Pin_9;
  GPIO_usartx.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_usartx.GPIO_Mode = GPIO_Mode_AF_PP;//复用推挽输出
  GPIO_Init(GPIOA, &GPIO_usartx); 
  //USART1_RX	  PA.10
  GPIO_usartx.GPIO_Pin = GPIO_Pin_10;
  GPIO_usartx.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
  GPIO_Init(GPIOA, &GPIO_usartx); 
	
	Usart_X.USART_BaudRate=BaudRatePrescaler;
	Usart_X.USART_WordLength=USART_WordLength_8b;//8位数据格式
	Usart_X.USART_StopBits=USART_StopBits_1;//1位停止位
	Usart_X.USART_Parity = USART_Parity_No;//无校验
	Usart_X.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
	Usart_X.USART_Mode= USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &Usart_X);
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启接收中断
  USART_Cmd(USART1, ENABLE);
	
	
	
	/* 使能串口1中断 */
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;	 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	/*IRQ通道使能*/
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	/*根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器USART1*/
	NVIC_Init(&NVIC_InitStructure);
}
//DMA_Configuration(DMA1_Channel6,(u32)(&USART2->DR),(u32)Uart_Rx,UART_RX_LEN)
void DMA_Configuration(DMA_Channel_TypeDef* DMA_CHx,u32 cpar,u32 cmar,u16 cndtr)
{
    DMA_InitTypeDef DMA_InitStructure;
	
    DMA_DeInit(DMA_CHx); 
    DMA_InitStructure.DMA_PeripheralBaseAddr = cpar; 
    DMA_InitStructure.DMA_MemoryBaseAddr = cmar; 
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC; 
    DMA_InitStructure.DMA_BufferSize = cndtr; 
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; 
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; 
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; 
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; 
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal; 
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh; 
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable; 
    DMA_Init(DMA_CHx,&DMA_InitStructure); 
 
    DMA_Cmd(DMA_CHx,ENABLE);
}
void Usart2_Int(uint32_t BaudRatePrescaler)
{
	GPIO_InitTypeDef GPIO_usartx;
	USART_InitTypeDef Usart_X;
	NVIC_InitTypeDef NVIC_InitStructure; 
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO, ENABLE);
	//启动DMA时钟
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE); 
	//USART2_TX   PA.2
  GPIO_usartx.GPIO_Pin = GPIO_Pin_2;
  GPIO_usartx.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_usartx.GPIO_Mode = GPIO_Mode_AF_PP;//复用推挽输出
  GPIO_Init(GPIOA, &GPIO_usartx); 
  //USART2_RX	  PA.3
  GPIO_usartx.GPIO_Pin = GPIO_Pin_3;
  GPIO_usartx.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
  GPIO_Init(GPIOA, &GPIO_usartx); 
	
	Usart_X.USART_BaudRate=BaudRatePrescaler;
	Usart_X.USART_WordLength=USART_WordLength_8b;//8位数据格式
	Usart_X.USART_StopBits=USART_StopBits_1;//1位停止位
	Usart_X.USART_Parity = USART_Parity_No;//无校验
	Usart_X.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
	Usart_X.USART_Mode= USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART2, &Usart_X);
  //USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//开启接收中断
	USART_ITConfig(USART2,USART_IT_IDLE,ENABLE);//开启空闲中断
  USART_Cmd(USART2, ENABLE);
	
	
	USART_DMACmd(USART2,USART_DMAReq_Rx,ENABLE);//采用DMA方式接收
	
	/* 使能串口2中断 */
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;	 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	/*IRQ通道使能*/
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	/*根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器USART1*/
	NVIC_Init(&NVIC_InitStructure);
	DMA_Configuration(DMA1_Channel6,(u32)(&USART2->DR),(u32)Uart_Rx,UART_RX_LEN);//DMA接收
	//MYDMA_Config(DMA1_Channel7,(u32)&USART2->DR,(u32)SendBuff1,SEND_BUF_SIZE);//配置USART2 DMA发送
}
//发送一个字节数据
//input:byte,待发送的数据
void USART_send_byte(USART_TypeDef* USARTx,uint8_t byte)
{
	USARTx->SR;
	USART_SendData(USARTx, byte);
	while(USART_GetFlagStatus(USARTx,USART_FLAG_TC)==RESET);//等待发送完成
}
//发送多字节数据
void USART_Send_bytes(USART_TypeDef* USARTx,uint8_t *Buffer, uint8_t Length)
{
	uint8_t i=0;
	while(i<Length)
	{
		USART_send_byte(USARTx,Buffer[i++]);
	}
}
//发送多字节数据+校验和
void USART_Send(USART_TypeDef* USARTx,uint8_t *Buffer, uint8_t Length)
{
	uint8_t i=0;
	while(i<Length)
	{
		if(i<(Length-1))
		Buffer[Length-1]+=Buffer[i];//累加Length-1前的数据
		USART_send_byte(USARTx,Buffer[i++]);
	}
}
//发送一帧数据
void send_out(uint8_t *data,uint8_t action,uint8_t function)
{
	uint8_t tx_DATA[14],i=0,k=0;
	memset(tx_DATA,0,14);//清零缓存TX_DATA
	tx_DATA[i++]=0XCF;//帧头
	tx_DATA[i++]=action;//动作字节---1表示控制灯，2，3表示控制云台俯仰，4表示控制系统进入待机状态
	tx_DATA[i++]=function;//功能字节-----1表示给设备写数据，2表示从设备读数据
	for(k=0;k<4;k++)//存入数据到缓存TX_DATA
	{
		tx_DATA[i++]=(uint16_t)data[k];
	}
	for(k=0;k<4;k++)//存入数据到缓存TX_DATA
	{
		tx_DATA[i++]=0;
	}
	tx_DATA[i++] = 0x00;
	tx_DATA[i++] = 0xAA;
	tx_DATA[i++] = 0xFF;
	
	USART_Send_bytes(USART3,tx_DATA,i);
	//printf("HOME命令发送完成\r\n");
	//memcpy(SendBuff2,TX_DATA,i); 
	//USART_DMACmd(USART3,USART_DMAReq_Tx,ENABLE);  //使能串口3的DMA发送 
	//MYDMA_Enable(DMA1_Channel2);     //开始一次DMA传输！
}
uint8_t TX_DATA[7] = {0};
void Send_out(uint8_t *data1,uint8_t *data2,uint8_t action,uint8_t function)
{
	uint8_t i=0,k=0;
	//memset(TX_DATA,0,14);//清零缓存TX_DATA
	TX_DATA[i++]=0XFF;//帧头
	TX_DATA[i++]=action;//动作字节---1表示控制灯，2，3表示控制云台俯仰，4表示控制系统进入待机状态
	TX_DATA[i++]=function;//功能字节-----1表示给设备写数据，2表示从设备读数据
	for(k=0;k<4;k++)//存入数据到缓存TX_DATA
	{
		TX_DATA[i++]=data1[k];
	}
	for(k=0;k<4;k++)//存入数据到缓存TX_DATA
	{
		TX_DATA[i++]=data2[k];
	}
	TX_DATA[i++] = 0x00;
	TX_DATA[i++] = 0xAA;
	TX_DATA[i++] = 0xFF;
	
	USART_Send_bytes(USART3,TX_DATA,i);
	printf("角度命令发送完成\r\n");
	//memcpy(SendBuff2,TX_DATA,i); 
	//USART_DMACmd(USART3,USART_DMAReq_Tx,ENABLE);  //使能串口3的DMA发送 
	//MYDMA_Enable(DMA1_Channel2);     //开始一次DMA传输！
}
void HY_Send_out(uint8_t type, uint8_t content1, uint8_t content2, uint8_t content3)
{
	uint8_t i=0,k=0;
	uint8_t tx_data[7] = {0};
	//memset(TX_DATA,0,14);//清零缓存TX_DATA
	tx_data[i++]=0XFF;//帧头
	tx_data[i++]=0X01;//地址 0~255
	tx_data[i++]=type;//
	
	tx_data[i++] = content1;
	tx_data[i++] = content2;
	tx_data[i++] = content3;
	
	for(k = 1; k < i; k++)
		tx_data[i] += tx_data[k];
	i++;
	USART_Send_bytes(USART3,tx_data,i);
	printf("角度命令发送完成\r\n");
	//memcpy(SendBuff2,TX_DATA,i); 
	//USART_DMACmd(USART3,USART_DMAReq_Tx,ENABLE);  //使能串口3的DMA发送 
	//MYDMA_Enable(DMA1_Channel2);     //开始一次DMA传输！
}
void set_rx_tx(uint8_t rx_state,uint8_t tx_state) 
{
  switch(rx_state)
	{
		case 0:
				GPIO_ResetBits(GPIOA, GPIO_Pin_12);
		    //printf("接收使能\n\r");
				break;
		case 1:
			  //printf("接收关闭\n\r");
			  GPIO_SetBits(GPIOA, GPIO_Pin_12);
		    break;
		default:
			  break;
			  //GPIO_SetBits(GPIOA, GPIO_Pin_12);
	}
	switch(tx_state)
	{
		case 0:
			  //printf("发送关闭\n\r");
				GPIO_ResetBits(GPIOA, GPIO_Pin_11);
				break;
		case 1:
			  //printf("发送使能\n\r");
			  GPIO_SetBits(GPIOA, GPIO_Pin_11);
		    break;
		default:
			  break;
			  //GPIO_ResetBits(GPIOA, GPIO_Pin_11);
	}	
}
