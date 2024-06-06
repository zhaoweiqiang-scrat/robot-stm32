#include "hardwareserial.h"
#include "interrupt.h"

USART_TypeDef*        SERIAL_USART[SERIALn] = {RIKI_SERIAL1, RIKI_SERIAL2, RIKI_SERIAL3};
GPIO_TypeDef*         SERIAL_PORT[SERIALn] = {RIKI_SERIAL1_GPIO_PORT, RIKI_SERIAL2_GPIO_PORT, RIKI_SERIAL3_GPIO_PORT}; 
const uint32_t        SERIAL_USART_CLK[SERIALn] = {RIKI_SERIAL1_CLK, RIKI_SERIAL2_CLK, RIKI_SERIAL3_CLK};
const uint32_t        SERIAL_PORT_CLK[SERIALn] = {RIKI_SERIAL1_GPIO_CLK, RIKI_SERIAL2_GPIO_CLK, RIKI_SERIAL3_GPIO_CLK};
const uint32_t        SERIAL_DMA_CLK[SERIALn] = {RIKI_SERIAL1_DMA_CLK,RIKI_SERIAL2_DMA_CLK,RIKI_SERIAL3_DMA_CLK};
const uint16_t        SERIAL_TX_PIN[SERIALn] = {RIKI_SERIAL1_TX_PIN, RIKI_SERIAL2_TX_PIN, RIKI_SERIAL3_TX_PIN}; 
const uint16_t        SERIAL_RX_PIN[SERIALn] = {RIKI_SERIAL1_RX_PIN, RIKI_SERIAL2_RX_PIN, RIKI_SERIAL2_RX_PIN}; 
const uint16_t        SERIAL_IRQn[SERIALn] = {RIKI_SERIAL1_IRQ, RIKI_SERIAL2_IRQ, RIKI_SERIAL3_IRQ};
const uint16_t        SERILA_NVIC[SERIALn] = {RIKI_SERIAL1_NVIC, RIKI_SERIAL2_NVIC, RIKI_SERIAL3_NVIC};
DMA_Stream_TypeDef *  SERILA_TX_DMA[SERIALn] = {RIKI_SERIAL1_DMA, RIKI_SERIAL2_DMA, RIKI_SERIAL3_DMA};
const uint32_t        SERILA_TX_DMA_CH[SERIALn] = {RIKI_SERIAL1_DMA_CH,RIKI_SERIAL2_DMA_CH,RIKI_SERIAL3_DMA_CH};      
const uint32_t        SERILA_TX_DMA_TCIF[SERIALn] = {RIKI_SERIAL1_DMA_TCIF,RIKI_SERIAL2_DMA_TCIF,RIKI_SERIAL3_DMA_TCIF};
const uint8_t         SERILA_GPIO_AF[SERIALn] = {RIKI_SERIAL1_GPIO_AF,RIKI_SERIAL2_GPIO_AF,RIKI_SERIAL3_GPIO_AF};
const uint8_t		  SERILA_TX_GPIO_SOURCE[SERIALn] = {RIKI_SERIAL1_TX_GPIO_Source,RIKI_SERIAL2_TX_GPIO_Source,RIKI_SERIAL3_TX_GPIO_Source};
const uint8_t		  SERILA_RX_GPIO_SOURCE[SERIALn] = {RIKI_SERIAL1_RX_GPIO_Source,RIKI_SERIAL2_RX_GPIO_Source,RIKI_SERIAL3_RX_GPIO_Source};

HardwareSerial::HardwareSerial(Serial_TypeDef _Serial)
{
		Serial = _Serial;

		if(this->Serial == SERIAL1)
			Serial1 = this;
		if(this->Serial == SERIAL2)
			Serial2 = this;
		if(this->Serial == SERIAL3)
			Serial3 = this;
}


void HardwareSerial::begin(uint32_t baud)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef  NVIC_InitStructure;
    DMA_InitTypeDef  DMA_InitStructure;
	
	if(this->Serial == SERIAL1) {
		RCC_APB2PeriphClockCmd(SERIAL_USART_CLK[this->Serial], ENABLE);	
	} else {
		RCC_APB1PeriphClockCmd(SERIAL_USART_CLK[this->Serial], ENABLE); 
	}
	RCC_AHB1PeriphClockCmd(SERIAL_PORT_CLK[this->Serial], ENABLE); 
	RCC_AHB1PeriphClockCmd(SERIAL_DMA_CLK[this->Serial],ENABLE);//DMAY时钟使能
	//串口对应引脚复用映射
	GPIO_PinAFConfig(SERIAL_PORT[this->Serial],SERILA_TX_GPIO_SOURCE[this->Serial],SERILA_GPIO_AF[this->Serial]); //GPIOx复用为USARTy
	GPIO_PinAFConfig(SERIAL_PORT[this->Serial],SERILA_RX_GPIO_SOURCE[this->Serial],SERILA_GPIO_AF[this->Serial]); //GPIOx复用为USARTy
    
	//USART端口配置
  GPIO_InitStructure.GPIO_Pin = SERIAL_TX_PIN[this->Serial] | SERIAL_RX_PIN[this->Serial]; //GPIOA2与GPIOA3
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(SERIAL_PORT[this->Serial],&GPIO_InitStructure); 
	
	USART_InitStructure.USART_BaudRate	 = baud;				
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;	
	USART_InitStructure.USART_StopBits 	 = USART_StopBits_1;	
	USART_InitStructure.USART_Parity	 = USART_Parity_No;		
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	

	USART_Init(SERIAL_USART[this->Serial], &USART_InitStructure); 		
	
   
	/* 配置 DMA Stream */
  DMA_InitStructure.DMA_Channel = SERILA_TX_DMA_CH[this->Serial]; //通道选择
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR;//DMA外设地址
  //DMA_InitStructure.DMA_Memory0BaseAddr = mar;//DMA 存储器0地址
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;//存储器到外设模式
  //DMA_InitStructure.DMA_BufferSize = ndtr;//数据传输量 
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
  DMA_Init(SERILA_TX_DMA[this->Serial], &DMA_InitStructure);//初始化DMA Stream
	
	NVIC_InitStructure.NVIC_IRQChannel = SERIAL_IRQn[this->Serial];       
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0; 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = SERILA_NVIC[this->Serial];		
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			
	NVIC_Init(&NVIC_InitStructure);			
  
	USART_DMACmd(SERIAL_USART[this->Serial],USART_DMAReq_Tx,ENABLE);  //使能串口1的DMA发送  	
	
	USART_ITConfig(SERIAL_USART[this->Serial], USART_IT_RXNE, ENABLE);

	USART_Cmd(SERIAL_USART[this->Serial], ENABLE);
}

uint32_t HardwareSerial::available(void)
{
	return (uint32_t)(SERIAL_BUFFER_SIZE + rx_buffer._iHead - rx_buffer._iTail) % SERIAL_BUFFER_SIZE ;
}


uint8_t HardwareSerial::read(void)
{
// if the head isn't ahead of the tail, we don't have any characters
  if ( rx_buffer._iHead == rx_buffer._iTail )
    return -1 ;

  uint8_t uc = rx_buffer._aucBuffer[rx_buffer._iTail] ;
  rx_buffer._iTail = (unsigned int)(rx_buffer._iTail + 1) % SERIAL_BUFFER_SIZE ;
  return uc ;

}

uint32_t HardwareSerial::write(uint8_t ch)
{
	#if 0
	DMA_Cmd(SERILA_TX_DMA[this->Serial], DISABLE);                      //关闭DMA传输 
	
	while (DMA_GetCmdStatus(SERILA_TX_DMA[this->Serial]) != DISABLE){}	//确保DMA可以被设置  
	/* Write to DMAy Streamx NDTR register */
    SERILA_TX_DMA[this->Serial]->NDTR = 1;
	/* Write to DMAy Streamx M0AR */
    SERILA_TX_DMA[this->Serial]->M0AR = (uint32_t)&ch;
    DMA_Cmd(SERILA_TX_DMA[this->Serial], ENABLE);
	while(1)
	{
		if(DMA_GetFlagStatus(SERILA_TX_DMA[this->Serial],SERILA_TX_DMA_TCIF[this->Serial])!=RESET)//等待DMAY_SteamX传输完成
		{ 
			DMA_ClearFlag(SERILA_TX_DMA[this->Serial],SERILA_TX_DMA_TCIF[this->Serial]);//清除DMAY_SteamX传输完成标志
			break; 
		}
	}
	#endif
	USART_SendData(SERIAL_USART[this->Serial], ch);
	while(USART_GetFlagStatus(SERIAL_USART[this->Serial], USART_FLAG_TXE) == RESET); 
	return 1; 
}

void HardwareSerial::flush()
{
	rx_buffer._iTail = rx_buffer._iHead;
}

void HardwareSerial::print(const char *format, ...)
{
	va_list args;
	char buf[256];
	va_start (args, format);
	vsprintf (buf, format, args);       
	va_end (args);    
	putstr(buf);

}

void HardwareSerial::putstr(const char *str)
{
	int i;
	for(i = 0; i < strlen(str); i++){	
		write(str[i]);
	}
}

void HardwareSerial::irq()
{
	uint8_t data;
	if (USART_GetITStatus(SERIAL_USART[Serial], USART_IT_RXNE) != RESET) {
		data = USART_ReceiveData(SERIAL_USART[Serial]);
		rx_buffer.store_char(data) ;  
		USART_ClearITPendingBit(SERIAL_USART[Serial], USART_IT_RXNE);    
	}
}

