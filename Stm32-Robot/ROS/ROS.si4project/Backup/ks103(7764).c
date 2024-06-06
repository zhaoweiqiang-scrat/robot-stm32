#ifdef __cplusplus
extern "C" {
#endif
//Sensors_ULtrasonic_driver ks103超声波传感器
#include "ks103.h"
#if 0
I2C_InitTypeDef iic2_init_struct; 


void delay_us(u32 n)
{
	SysTick->LOAD= n*9;       //时间加载  
	SysTick->VAL= 0;        //清空计数器      
 	SysTick->CTRL|= ENABLE;            //开始倒数    
 	while(SysTick->VAL||!(SysTick->CTRL&(1<<16)));//等待时间到达 
 	SysTick->CTRL=0X00000000;       //关闭计数器
 	SysTick->VAL= 0;        //清空计数器  
}
void delay_ms(u32 n)
{
	while(n--)
	{
		delay_us(1000);		
	}
}	
void i2c_init(void)
{
	GPIO_InitTypeDef iic2_io_init_struct;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2,ENABLE);		
	iic2_init_struct.I2C_OwnAddress1=0X00;
	iic2_init_struct.I2C_ClockSpeed=80000;
	iic2_init_struct.I2C_Mode=I2C_Mode_I2C;
	iic2_init_struct.I2C_Ack=I2C_Ack_Disable; 
	iic2_init_struct.I2C_AcknowledgedAddress=I2C_AcknowledgedAddress_7bit;
	I2C_Init(I2C2,&iic2_init_struct);
	I2C_Cmd(I2C1,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH,ENABLE);//PH4--I2C2_CLK PH5--I2C2_SDA	
	iic2_io_init_struct.GPIO_Pin=GPIO_Pin_4|GPIO_Pin_5;
	iic2_io_init_struct.GPIO_Speed=GPIO_Speed_50MHz;
	iic2_io_init_struct.GPIO_Mode=GPIO_Mode_AF;
	iic2_io_init_struct.GPIO_PuPd = GPIO_PuPd_NOPULL; 
	GPIO_Init(GPIOH,&iic2_io_init_struct); 
}

void ks103_send(u8 addr,u8 reg,u8 cmd)
{
	while(I2C_GetFlagStatus(I2C2,I2C_FLAG_BUSY));   
	/* Send START condition */
  I2C_GenerateSTART(I2C2, ENABLE);  
    /* Test on EV5 and clear it */
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));   
   /* Send MMA address for write */
  I2C_Send7bitAddress(I2C2, addr, I2C_Direction_Transmitter);    
	/* Test on EV6 and clear it */
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));   
    /* Send the MMA's Register address to write to */    
  I2C_SendData(I2C2, reg);   
  /* Test on EV8 and clear it */
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));   
    /* Send the byte to be written */
  I2C_SendData(I2C2, cmd);  
   /* Test on EV8 and clear it */
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));     
   /* Send STOP condition */
  I2C_GenerateSTOP(I2C2, ENABLE);
}				 
u8 ks103_recieve(u8 addr,u8 cmd)
{
	u8 RxData;
	I2C_GenerateSTART(I2C2, ENABLE);
	/* Test on EV5 and clear it */
	while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));
	/* Send ADXL address for write */
	I2C_Send7bitAddress(I2C2, addr, I2C_Direction_Transmitter);
	/* Test on EV6 and clear it */
	while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	/* Send the ADXL's Register address to write to */
	I2C_SendData(I2C2, cmd);  
	while(I2C_GetFlagStatus(I2C2, I2C_FLAG_BTF) == RESET);
	/* Send STRAT condition a second time */  
	I2C_GenerateSTART(I2C2, ENABLE);
	/* Test on EV5 and clear it */
	while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));
	/* Send ADXL address for read */
	I2C_Send7bitAddress(I2C2, addr+1, I2C_Direction_Receiver);
	while(I2C_GetFlagStatus(I2C2, I2C_FLAG_ADDR) == RESET);
	/* Disable Acknowledgement */
	I2C_AcknowledgeConfig(I2C2, DISABLE);
	(void)I2C2->SR2;
	/*!< Send STOP Condition */
	I2C_GenerateSTOP(I2C2, ENABLE);
	while(I2C_GetFlagStatus(I2C2, I2C_FLAG_RXNE) == RESET);
	/* Read a byte from the ADXL Register */
	RxData = I2C_ReceiveData(I2C2);
	while(I2C2->CR1 & I2C_CR1_STOP);
  I2C_AcknowledgeConfig(I2C2, ENABLE);
	return RxData;
}

u16 Sensors_Ultrasonic_task1(u8 add1,u8 add2 ,u16 a[])
{
	u8 HignE,LowE;	 
		ks103_send(add1,0x02,0xbc);	
	  ks103_send(add2,0x02,0xbc);
		//此处延时最少87ms		
		delay_ms(90);	
		I2C_DeInit(I2C2);		
		I2C_Init(I2C2,&iic2_init_struct);
		I2C_Cmd(I2C2,ENABLE);
		HignE=ks103_recieve(add1,0x02);
		LowE=ks103_recieve(add1,0x03);
	  a[0]=HignE*256+LowE;
	  HignE=ks103_recieve(add2,0x02);
		LowE=ks103_recieve(add2,0x03);
	  a[1]=HignE*256+LowE;		
	  return 1;		
}
#else
u8  AskCmd1[3]={0xE8,0x02,0xBC};
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
#endif


#ifdef __cplusplus
}
#endif