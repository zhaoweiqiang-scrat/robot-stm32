#include "main.h"
#include "string.h"
#include "stdio.h"

#define DMA_RX1 1
#define DMA_RX2 0
#define DMA_RX3 1

USART_RECEIVETYPE Usart1Type; //����1���ؽ���
USART_RECEIVETYPE Usart2Type; //����2����
USART_RECEIVETYPE Usart3Type; //����3����429����

u8 SendBuff3[SEND_BUF_SIZE];	//����3�������ݻ�����

#if 1
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 

}; 

FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//�ض���fputc���� 
int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40)==0);//ѭ������,ֱ���������   
    USART1->DR = (u8) ch;      
	return ch;
}
#endif 
//DMA1�ĸ�ͨ������
//����Ĵ�����ʽ�ǹ̶���,���Ҫ���ݲ�ͬ��������޸�
//�Ӵ洢��->����ģʽ/8λ���ݿ��/�洢������ģʽ
//DMA_CHx:DMAͨ��CHx
//cpar:�����ַ
//cmar:�洢����ַ
//cndtr:���ݴ����� 
//MYDMA_Config(DMA1_Channel4,(u32)&USART1->DR,(u32)SendBuff,SEND_BUF_SIZE);//DMA1ͨ��4,����Ϊ����1,�洢��ΪSendBuff,����SEND_BUF_SIZE.
void MYDMA_Config(DMA_Channel_TypeDef* DMA_CHx,u32 cpar,u32 cmar,u16 cndtr)
{
	DMA_InitTypeDef DMA_InitStructure;
 	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);	//ʹ��DMA����
	
  DMA_DeInit(DMA_CHx);   //��DMA��ͨ��1�Ĵ�������Ϊȱʡֵ

	DMA_InitStructure.DMA_PeripheralBaseAddr = cpar;  //DMA�������ַ
	DMA_InitStructure.DMA_MemoryBaseAddr = cmar;  //DMA�ڴ����ַ
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;  //���ݴ��䷽�򣬴��ڴ��ȡ���͵�����
	DMA_InitStructure.DMA_BufferSize = cndtr;  //DMAͨ����DMA����Ĵ�С
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  //�����ַ�Ĵ�������
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  //�ڴ��ַ�Ĵ�������
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  //���ݿ��Ϊ8λ
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; //���ݿ��Ϊ8λ
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;  //����������ģʽ
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium; //DMAͨ�� xӵ�������ȼ� 
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  //DMAͨ��xû������Ϊ�ڴ浽�ڴ洫��
	DMA_Init(DMA_CHx, &DMA_InitStructure);  //����DMA_InitStruct��ָ���Ĳ�����ʼ��DMA��ͨ��USART1_Tx_DMA_Channel����ʶ�ļĴ���
} 
//����һ��DMA����
void MYDMA_Enable(DMA_Channel_TypeDef*DMA_CHx)
{ 
	DMA_Cmd(DMA_CHx, DISABLE );  //�ر�USART TX DMA1 ��ָʾ��ͨ��      
 	DMA_SetCurrDataCounter(DMA_CHx,SEND_BUF_SIZE);//DMAͨ����DMA����Ĵ�С
 	DMA_Cmd(DMA_CHx, ENABLE);  //ʹ��USART TX DMA1 ��ָʾ��ͨ�� 
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
//����1����ͨ��
void Usart1_Int(uint32_t BaudRatePrescaler)
{
	GPIO_InitTypeDef GPIO_usartx;
	USART_InitTypeDef Usart_X;
	GPIO_InitTypeDef GPIO_rtx;
	NVIC_InitTypeDef NVIC_InitStructure; 
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO, ENABLE);
	
	//����DMAʱ��
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE); 
	
	  //USART1_TX   PA.9
  GPIO_usartx.GPIO_Pin = GPIO_Pin_9;
  GPIO_usartx.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_usartx.GPIO_Mode = GPIO_Mode_AF_PP;//�����������
  GPIO_Init(GPIOA, &GPIO_usartx); 
  //USART1_RX	  PA.10
  GPIO_usartx.GPIO_Pin = GPIO_Pin_10;
  GPIO_usartx.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
  GPIO_Init(GPIOA, &GPIO_usartx); 
	
	Usart_X.USART_BaudRate=BaudRatePrescaler;
	Usart_X.USART_WordLength=USART_WordLength_8b;//8λ���ݸ�ʽ
	Usart_X.USART_StopBits=USART_StopBits_1;//1λֹͣλ
	Usart_X.USART_Parity = USART_Parity_No;//��У��
	Usart_X.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
	Usart_X.USART_Mode= USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &Usart_X);
  
	#if DMA_RX1
	USART_ITConfig(USART1,USART_IT_IDLE,ENABLE);//���������ж�
	#else
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//���������ж�
	#endif
	
  USART_Cmd(USART1, ENABLE);
	
	#if DMA_RX1
	USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);//����DMA��ʽ����
	#endif
	
	GPIO_rtx.GPIO_Pin = GPIO_Pin_11|GPIO_Pin_12;
  GPIO_rtx.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_rtx.GPIO_Mode = GPIO_Mode_Out_PP;//�����������
  GPIO_Init(GPIOA, &GPIO_rtx); 
	
	/* ʹ�ܴ���1�ж� */
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;	 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	/*IRQͨ��ʹ��*/
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	/*����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���USART1*/
	NVIC_Init(&NVIC_InitStructure);
	#if DMA_RX1
	DMA_Configuration(DMA1_Channel5,(u32)(&USART1->DR),(u32)Usart1Type.RX_pData,RX_LEN);//DMA����
	#endif
	TX_EN;
	RX_DIS;//Ĭ�Ϸ���
}

void Usart2_Int(uint32_t BaudRatePrescaler)
{
	GPIO_InitTypeDef GPIO_usartx;
	USART_InitTypeDef Usart_X;
	NVIC_InitTypeDef NVIC_InitStructure; 
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO, ENABLE);
	//����DMAʱ��
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE); 
	//USART2_TX   PA.2
  GPIO_usartx.GPIO_Pin = GPIO_Pin_2;
  GPIO_usartx.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_usartx.GPIO_Mode = GPIO_Mode_AF_PP;//�����������
  GPIO_Init(GPIOA, &GPIO_usartx); 
  //USART2_RX	  PA.3
  GPIO_usartx.GPIO_Pin = GPIO_Pin_3;
  GPIO_usartx.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
  GPIO_Init(GPIOA, &GPIO_usartx); 
	
	Usart_X.USART_BaudRate=BaudRatePrescaler;
	Usart_X.USART_WordLength=USART_WordLength_8b;//8λ���ݸ�ʽ
	Usart_X.USART_StopBits=USART_StopBits_1;//1λֹͣλ
	Usart_X.USART_Parity = USART_Parity_No;//��У��
	Usart_X.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
	Usart_X.USART_Mode= USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART2, &Usart_X);
  //USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//���������ж�
	USART_ITConfig(USART2,USART_IT_IDLE,ENABLE);//���������ж�
  USART_Cmd(USART2, ENABLE);
	
	
	USART_DMACmd(USART2,USART_DMAReq_Rx,ENABLE);//����DMA��ʽ����
	
	/* ʹ�ܴ���2�ж� */
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;	 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	/*IRQͨ��ʹ��*/
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	/*����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���USART1*/
	NVIC_Init(&NVIC_InitStructure);
	//DMA_Configuration(DMA1_Channel6,(u32)(&USART2->DR),(u32)Uart_Rx,UART_RX_LEN);//DMA����
	//MYDMA_Config(DMA1_Channel7,(u32)&USART2->DR,(u32)SendBuff1,SEND_BUF_SIZE);//����USART2 DMA����
}

void Usart3_Int(uint32_t BaudRatePrescaler)
{
	GPIO_InitTypeDef GPIO_usartx;
	
	USART_InitTypeDef Usart_X;
	NVIC_InitTypeDef NVIC_InitStructure; 
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO|RCC_APB2Periph_GPIOA, ENABLE);
	  //USART3_TX   PB.10
  GPIO_usartx.GPIO_Pin = GPIO_Pin_10;
  GPIO_usartx.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_usartx.GPIO_Mode = GPIO_Mode_AF_PP;//�����������
  GPIO_Init(GPIOB, &GPIO_usartx); 
  //USART3_RX	  PB.11
  GPIO_usartx.GPIO_Pin = GPIO_Pin_11;
  GPIO_usartx.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
  GPIO_Init(GPIOB, &GPIO_usartx); 
	
	Usart_X.USART_BaudRate=BaudRatePrescaler;
	Usart_X.USART_WordLength=USART_WordLength_8b;//8λ���ݸ�ʽ
	Usart_X.USART_StopBits=USART_StopBits_1;//1λֹͣλ
	Usart_X.USART_Parity = USART_Parity_No;//��У��
	Usart_X.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
	Usart_X.USART_Mode= USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART3, &Usart_X);
	
	#if DMA_RX3
	USART_ITConfig(USART3,USART_IT_IDLE,ENABLE);//���������ж�
	#else
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//���������ж�
	#endif
	
  USART_Cmd(USART3, ENABLE);
	
	#if DMA_RX3
	USART_DMACmd(USART3,USART_DMAReq_Rx,ENABLE);//����DMA��ʽ����
	#endif
	
	#if DMA_RX3
	DMA_Configuration(DMA1_Channel3,(u32)(&USART3->DR),(u32)Usart3Type.RX_pData,RX_LEN);//DMA����
	#endif
	/* ʹ�ܴ���3�ж� */
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;	 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	/*IRQͨ��ʹ��*/
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	/*����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���USART3*/
	NVIC_Init(&NVIC_InitStructure);
	
	
	MYDMA_Config(DMA1_Channel2,(u32)&USART3->DR,(u32)SendBuff3,SEND_BUF_SIZE);//����USART3 DMA����
	//DMA_ITConfig(DMA1_Channel2,DMA_IT_TC,ENABLE);
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel2_IRQn;	 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	/*IRQͨ��ʹ��*/
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	/*����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���USART3*/
//	NVIC_Init(&NVIC_InitStructure);
}
uint8_t deal_irq_rx1_end(void)  
{ 
	uint8_t Length = 0;//���ݳ���
	if(USART_GetITStatus(USART1, USART_IT_IDLE) != RESET) 
  { 
        DMA_Cmd(DMA1_Channel5,DISABLE);
        USART1->SR; 
        USART1->DR; //��USART_IT_IDLE��־
        Length = RX_LEN - DMA_GetCurrDataCounter(DMA1_Channel5); 
				if((Usart1Type.RX_pData[0] == 0xdd) &&(Length == 0x22))
				{
					Usart1Type.RX_flag = 1;
					Usart1Type.RX_Size = 0x22;
				}
        //���ô������ݳ��� 
        DMA1_Channel5->CNDTR = RX_LEN;//������װ�����õ�ַƫַ��0��ʼ
        DMA_Cmd(DMA1_Channel5, ENABLE);//�����꣬���´�DMA  
				return Length;
	}
	return 0;
}

void USART1_IRQHandler(void)
{
	#if DMA_RX1	
	deal_irq_rx1_end();
	#else
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
		USART_ClearFlag(USART1,USART_FLAG_RXNE);//���жϱ�־
		Usart1Type.RX_pData[Usart1Type.RX_Size++]=USART_ReceiveData(USART1);;
		
		if(Usart1Type.RX_pData[0]!=0xdd)//�ж�֡ͷ
			Usart1Type.RX_Size=0;
		if(Usart1Type.RX_Size==0x22)
		{
			Usart1Type.RX_flag = 1;
			Usart1Type.RX_Size = 0;
		}
	}
	#endif
}

uint8_t deal_irq_rx3_end(void)  
{ 
	uint8_t Length = 0;//���ݳ���
	if(USART_GetITStatus(USART3, USART_IT_IDLE) != RESET) 
  { 
        DMA_Cmd(DMA1_Channel3,DISABLE);
        USART3->SR; 
        USART3->DR; //��USART_IT_IDLE��־
        Length = RX_LEN - DMA_GetCurrDataCounter(DMA1_Channel3); 
				if((Usart3Type.RX_pData[0] == 0xa5) && (Usart3Type.RX_pData[1] == 0x5a) && (Length == 0x04))
				{
					Usart3Type.RX_flag = 1;
					Usart3Type.RX_Size = 0x04;
					if(Usart3Type.RX_pData[2] == 0x02)
						CHARGE_EN;//�򿪳��̵���
					else if(Usart3Type.RX_pData[2] == 0x01)
						CHARGE_DIS;//�رճ��̵���
				}
        //���ô������ݳ��� 
        DMA1_Channel3->CNDTR = RX_LEN;//������װ�����õ�ַƫַ��0��ʼ
        DMA_Cmd(DMA1_Channel3, ENABLE);//�����꣬���´�DMA  
				return Length;
	}
	return 0;
}

void USART3_IRQHandler(void)
{
	#if DMA_RX3
	deal_irq_rx3_end();
	#else
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
	{
		USART_ClearFlag(USART3,USART_FLAG_RXNE);//���жϱ�־
		Usart3Type.RX_pData[Usart3Type.RX_Size++]=USART_ReceiveData(USART3);
	}	
	#endif
}

//����һ���ֽ�����
//input:byte,�����͵�����
void USART_send_byte(USART_TypeDef* USARTx,uint8_t byte)
{
	USART_SendData(USARTx, byte);
	while(USART_GetFlagStatus(USARTx,USART_FLAG_TC)==RESET);//�ȴ��������
}
//���Ͷ��ֽ�����
void USART_Send_bytes(USART_TypeDef* USARTx,uint8_t *Buffer, uint8_t Length)
{
	uint8_t i=0;
	while(i<Length)
	{
		USART_send_byte(USARTx,Buffer[i++]);
	}
}
//���Ͷ��ֽ�����+У���
void USART_Send(USART_TypeDef* USARTx,uint8_t *Buffer, uint8_t Length)
{
	uint8_t i=0;
	while(i<Length)
	{
		if(i<(Length-1))
		Buffer[Length-1]+=Buffer[i];//�ۼ�Length-1ǰ������
		USART_send_byte(USARTx,Buffer[i++]);
	}
}
//����һ֡����
void send_out(uint8_t *data,uint8_t action,uint8_t function)
{
	uint8_t TX_DATA[14],i=0,k=0;
	memset(TX_DATA,0,14);//���㻺��TX_DATA
	TX_DATA[i++]=0XCF;//֡ͷ
	TX_DATA[i++]=action;//�����ֽ�---1��ʾ���Ƶƣ�2��3��ʾ������̨������4��ʾ����ϵͳ�������״̬
	TX_DATA[i++]=function;//�����ֽ�-----1��ʾ���豸д���ݣ�2��ʾ���豸������
	for(k=0;k<4;k++)//�������ݵ�����TX_DATA
	{
		TX_DATA[i++]=(uint16_t)data[k];
	}
	for(k=0;k<4;k++)//�������ݵ�����TX_DATA
	{
		TX_DATA[i++]=0;
	}
	TX_DATA[i++] = 0x00;
	TX_DATA[i++] = 0xAA;
	TX_DATA[i++] = 0xFF;
	
	USART_Send_bytes(USART3,TX_DATA,i);
	//printf("HOME��������\r\n");
	//memcpy(SendBuff2,TX_DATA,i); 
	//USART_DMACmd(USART3,USART_DMAReq_Tx,ENABLE);  //ʹ�ܴ���3��DMA���� 
	//MYDMA_Enable(DMA1_Channel2);     //��ʼһ��DMA���䣡
}
void Send_out(uint8_t *data1,uint8_t *data2,uint8_t action,uint8_t function)
{
	uint8_t TX_DATA[14],i=0,k=0;
	memset(TX_DATA,0,14);//���㻺��TX_DATA
	TX_DATA[i++]=0XCF;//֡ͷ
	TX_DATA[i++]=action;//�����ֽ�---1��ʾ���Ƶƣ�2��3��ʾ������̨������4��ʾ����ϵͳ�������״̬
	TX_DATA[i++]=function;//�����ֽ�-----1��ʾ���豸д���ݣ�2��ʾ���豸������
	for(k=0;k<4;k++)//�������ݵ�����TX_DATA
	{
		TX_DATA[i++]=data1[k];
	}
	for(k=0;k<4;k++)//�������ݵ�����TX_DATA
	{
		TX_DATA[i++]=data2[k];
	}
	TX_DATA[i++] = 0x00;
	TX_DATA[i++] = 0xAA;
	TX_DATA[i++] = 0xFF;
	
	USART_Send_bytes(USART3,TX_DATA,i);
	printf("�Ƕ���������\r\n");
	//memcpy(SendBuff2,TX_DATA,i); 
	//USART_DMACmd(USART3,USART_DMAReq_Tx,ENABLE);  //ʹ�ܴ���3��DMA���� 
	//MYDMA_Enable(DMA1_Channel2);     //��ʼһ��DMA���䣡
}
