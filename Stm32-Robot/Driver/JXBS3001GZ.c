#ifdef __cplusplus
extern "C" {
#endif
//��ǿ�ȼ�⴫����JXBS-3001-GZ
#include "JXBS3001GZ.h"
#include "sys.h"
#include "millisecondtimer.h"

#define RS485_RX_EN PGout(7) //485ģʽ����.0,ʹ�ܽ���
#define RS485_TX_EN PGout(6) //485ģʽ����.1,ʹ�ܷ���.

#define USART6_REC_LEN  			20 	//�����������ֽ��� 200
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
		/*ʹ��DMA2ʱ��*/
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

    /* ����ʹ��DMA�������� */
    DMA_DeInit(DMA2_Stream2); 

    DMA_InitStructure.DMA_Channel             = DMA_Channel_5;               /* ����DMAͨ�� */
    DMA_InitStructure.DMA_PeripheralBaseAddr  = (uint32_t)(&(USART6->DR));   /* Դ*/
    DMA_InitStructure.DMA_Memory0BaseAddr     = (uint32_t)usart6_buf;      /* Ŀ�� */
    DMA_InitStructure.DMA_DIR                 = DMA_DIR_PeripheralToMemory;    /* ���� ����--���洢�� */
    DMA_InitStructure.DMA_BufferSize          = USART6_REC_LEN;                    /* ���� */                  
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

    DMA_Init(DMA2_Stream2, &DMA_InitStructure);

    USART_DMACmd(USART6,USART_DMAReq_Rx,ENABLE);

    /* ʹ��DMA */ 
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
	GPIO_InitTypeDef  GPIO_InitStructure;   /*������ؽṹ��*/
	USART_InitTypeDef  USART_InitStructure;
	NVIC_InitTypeDef  NVIC_InitStructure;   
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOG,ENABLE);//�� ʹ����Ӧ��ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6,ENABLE);
	
	//����6���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_USART6); //GPIOC6����ΪUSART6
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_USART6); //GPIOC7����ΪUSART6
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOC,&GPIO_InitStructure);//��  ��ʼ����Ӧ��IO��ģʽ
	
	//PG6\PG7���������485ģʽ����  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; //GPIOG6 GPIOG7
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //�������
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOG,&GPIO_InitStructure); //��ʼ��PE8\PE9
	
	USART_InitStructure.USART_BaudRate=bound;//������
	USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;//��Ӳ������
	USART_InitStructure.USART_Mode=USART_Mode_Tx|USART_Mode_Rx;//ģʽ
	USART_InitStructure.USART_Parity=USART_Parity_No;//У��
	USART_InitStructure.USART_StopBits=USART_StopBits_1;//ֹͣλ
	USART_InitStructure.USART_WordLength=USART_WordLength_8b;//���ݳ���
	
	USART_Init(USART6,&USART_InitStructure);//�� ��ʼ��������ز���
	//USART_ITConfig(USART6,USART_IT_RXNE,ENABLE);//���������ж�
	USART_ITConfig(USART6, USART_IT_IDLE, ENABLE);  // ��������IDLE�ж�
	USART_GetFlagStatus(USART6, USART_FLAG_TC);
	
	USART_Cmd(USART6,ENABLE);//ʹ�ܴ���6
	
	NVIC_InitStructure.NVIC_IRQChannel=USART6_IRQn;//ͨ��
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;//�����ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;//�����ж����ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0;//�����ȼ�
	NVIC_Init(&NVIC_InitStructure);
	DMA_Use_USART6_Rx_Init();
	RS485_TX_EN = 1;				//����ģʽ
	RS485_RX_EN = 1;
	numinit();
}
static uint8_t deal_irq_rx_end(uint8_t *buf)  
{     
    uint16_t len = 0;  
    /* ��������ж� */
    if(USART_GetITStatus(USART6, USART_IT_IDLE) != RESET)  
    {  
        USART6->SR;  
        USART6->DR; /*���USART_IT_IDLE��־*/
        /* �رս���DMA  */
        DMA_Cmd(DMA2_Stream2,DISABLE);  
        /*�����־λ*/
        DMA_ClearFlag(DMA2_Stream2,DMA_FLAG_TCIF2);  

        /*��ý���֡֡��*/
        len = USART6_REC_LEN - DMA_GetCurrDataCounter(DMA2_Stream2);  
        memcpy(buf,usart6_buf,len);  
				Reset = 0;
        /*���ô������ݳ���*/
        DMA_SetCurrDataCounter(DMA2_Stream2,USART6_REC_LEN);  
        /* ��DMA */
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
		RS485_TX_EN = 1;				//����ģʽ
		RS485_RX_EN = 1;
		for(i=0;i<8;i++)
		{	
			USART_SendData(USART6, buf[i]);
			while(USART_GetFlagStatus(USART6, USART_FLAG_TC) == RESET){}; //�ȴ��������
		}
		RS485_RX_EN = 0;//����ģʽ
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