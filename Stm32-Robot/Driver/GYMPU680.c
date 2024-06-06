#ifdef __cplusplus
extern "C" {
#endif
#include "stm32f4xx.h"
#include "string.h"
#include "millisecondtimer.h"
#include "GYMPU680.h"
#define USART5_REC_LEN  			20 	//�����������ֽ��� 200
u8  uart5_len;
char uart5_ch;
u8 uart5_buf[USART5_REC_LEN];
u8 uart5_rx_buf[USART5_REC_LEN];

uint8_t RX_BUF[20]={0},stata=0;
//У��ͼ��
static void DMA_Use_UART5_Rx_Init(void)
{
    DMA_InitTypeDef DMA_InitStructure;
		/*ʹ��DMA1ʱ��*/
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

    /* ����ʹ��DMA�������� */
    DMA_DeInit(DMA1_Stream0); 

    DMA_InitStructure.DMA_Channel             = DMA_Channel_4;               /* ����DMAͨ�� */
    DMA_InitStructure.DMA_PeripheralBaseAddr  = (uint32_t)(&(UART5->DR));   /* Դ*/
    DMA_InitStructure.DMA_Memory0BaseAddr     = (uint32_t)uart5_rx_buf;      /* Ŀ�� */
    DMA_InitStructure.DMA_DIR                 = DMA_DIR_PeripheralToMemory;    /* ���� ����--���洢�� */
    DMA_InitStructure.DMA_BufferSize          = USART5_REC_LEN;                    /* ���� */                  
    DMA_InitStructure.DMA_PeripheralInc       = DMA_PeripheralInc_Disable;    /* �����ַ�Ƿ����� */
    DMA_InitStructure.DMA_MemoryInc           = DMA_MemoryInc_Enable;         /* �ڴ��ַ�Ƿ����� */
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

    /* ʹ��DMA */ 
    DMA_Cmd(DMA1_Stream0,ENABLE);
}

void GYMPU680_Init(uint32_t bound)
{
	GPIO_InitTypeDef GPIO_usartx;
	USART_InitTypeDef Usart_X;
  NVIC_InitTypeDef NVIC_X;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOD, ENABLE);
	
	//����5���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource12,GPIO_AF_UART5); //GPIOC12����ΪUART5
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource2,GPIO_AF_UART5); //GPIOD2����ΪUART5
	
	//UART5_TX   PC.12
  GPIO_usartx.GPIO_Pin = GPIO_Pin_12;
	GPIO_usartx.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_usartx.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_usartx.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_usartx.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOC,&GPIO_usartx);//��  ��ʼ����Ӧ��IO��ģʽ
	//UART5_RX	 PD.2
	GPIO_usartx.GPIO_Pin = GPIO_Pin_2;
	GPIO_usartx.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_usartx.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_usartx.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_usartx.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOD,&GPIO_usartx);//��  ��ʼ����Ӧ��IO��ģʽ
	
	Usart_X.USART_BaudRate=bound;//������
	Usart_X.USART_HardwareFlowControl=USART_HardwareFlowControl_None;//��Ӳ������
	Usart_X.USART_Mode=USART_Mode_Tx|USART_Mode_Rx;//ģʽ
	Usart_X.USART_Parity=USART_Parity_No;//У��
	Usart_X.USART_StopBits=USART_StopBits_1;//ֹͣλ
	Usart_X.USART_WordLength=USART_WordLength_8b;//���ݳ���
	
	USART_Init(UART5,&Usart_X);//�� ��ʼ��������ز���
  //USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);//���������ж�
	USART_ITConfig(UART5, USART_IT_IDLE, ENABLE);  // ��������IDLE�ж�
  USART_Cmd(UART5, ENABLE);
  
  /* 4����ռ���ȼ���4����Ӧ���ȼ� */
 // NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  /*��ռ���ȼ��ɴ���жϼ���͵��ж�*/
	/*��Ӧ���ȼ����ȼ�ִ��*/
	NVIC_X.NVIC_IRQChannel = UART5_IRQn;//�ж�����
  NVIC_X.NVIC_IRQChannelPreemptionPriority = 7;//��ռ���ȼ�
  NVIC_X.NVIC_IRQChannelSubPriority = 0;//��Ӧ���ȼ�
  NVIC_X.NVIC_IRQChannelCmd = ENABLE;//ʹ���ж���Ӧ
  NVIC_Init(&NVIC_X);
	DMA_Use_UART5_Rx_Init();
}


//input:byte,�����͵�����
static void USART_send_byte(uint8_t byte)
{
	USART_SendData(UART5, byte);	
	while(USART_GetFlagStatus(UART5,USART_FLAG_TC)==RESET);//�ȴ��������
}
//���Ͷ��ֽ�����
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
	if(number>20)//�����ϴ�����
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
	
	USART_Send_bytes(Set_Cmd,4);//����
	delay(100);
	USART_Send_bytes(Auto_Cmd,4);//�����Զ����ָ��
	delay(100);
}
static uint8_t deal_irq_rx_end(uint8_t *buf)  
{     
    uint16_t len = 0;  
    /* ��������ж� */
    if(USART_GetITStatus(UART5, USART_IT_IDLE) != RESET)  
    {  
        UART5->SR;  
        UART5->DR; /*���USART_IT_IDLE��־*/
			  stata=1;
        /* �رս���DMA  */
        DMA_Cmd(DMA1_Stream0,DISABLE);  
        /*�����־λ*/
        DMA_ClearFlag(DMA1_Stream0,DMA_FLAG_TCIF0);  

        /*��ý���֡֡��*/
        len = USART5_REC_LEN - DMA_GetCurrDataCounter(DMA1_Stream0);  
        memcpy(buf,uart5_rx_buf,len);  

        /*���ô������ݳ���*/
        DMA_SetCurrDataCounter(DMA1_Stream0,USART5_REC_LEN);  
        /* ��DMA */
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