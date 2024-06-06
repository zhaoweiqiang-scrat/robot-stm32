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
		/*ʹ��DMA1ʱ��*/
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

    /* ����ʹ��DMA�������� */
    DMA_DeInit(DMA1_Stream2); 

    DMA_InitStructure.DMA_Channel             = DMA_Channel_4;               /* ����DMAͨ�� */
    DMA_InitStructure.DMA_PeripheralBaseAddr  = (uint32_t)(&(UART4->DR));   /* Դ*/
    DMA_InitStructure.DMA_Memory0BaseAddr     = (uint32_t)uart4_rx_buf;      /* Ŀ�� */
    DMA_InitStructure.DMA_DIR                 = DMA_DIR_PeripheralToMemory;    /* ���� ����--���洢�� */
    DMA_InitStructure.DMA_BufferSize          = USART4_REC_LEN;                    /* ���� */                  
    DMA_InitStructure.DMA_PeripheralInc       = DMA_PeripheralInc_Disable;    /* �����ַ�Ƿ����� */
    DMA_InitStructure.DMA_MemoryInc           = DMA_MemoryInc_Enable;         /* �ڴ��ַ�Ƿ����� */
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

    /* ʹ��DMA */ 
    DMA_Cmd(DMA1_Stream2,ENABLE);
}
//DMAx�ĸ�ͨ������
//����Ĵ�����ʽ�ǹ̶���,���Ҫ���ݲ�ͬ��������޸�
//�Ӵ洢��->����ģʽ/8λ���ݿ��/�洢������ģʽ
//DMA_Streamx:DMA������,DMA1_Stream0~7/DMA2_Stream0~7
//chx:DMAͨ��ѡ��,@ref DMA_channel DMA_Channel_0~DMA_Channel_7
//par:�����ַ
//mar:�洢����ַ
//ndtr:���ݴ�����  
//MYDMA_Config(DMA1_Stream6,DMA_Channel_4,(u32)&USART2->DR,(u32)USART_RX_BUF,USART_REC_LEN);//DMA1,STEAM6,CH4,����Ϊ����2,�洢��ΪSendBuff,����Ϊ:SEND_BUF_SIZE.
void MYDMA_UART4_Config()
{ 
 
	DMA_InitTypeDef  DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);//DMA1ʱ��ʹ�� 
	
  DMA_DeInit(DMA1_Stream4);
	
	while (DMA_GetCmdStatus(DMA1_Stream4) != DISABLE){}//�ȴ�DMA������ 
	
  /* ���� DMA Stream */
  DMA_InitStructure.DMA_Channel = DMA_Channel_4;  //ͨ��ѡ��
  DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&UART4->DR;//DMA�����ַ
  DMA_InitStructure.DMA_Memory0BaseAddr = (u32)uart4_tx_buf;//DMA �洢��0��ַ
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;//�洢��������ģʽ
  DMA_InitStructure.DMA_BufferSize = USART4_REC_LEN;//���ݴ����� 
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//���������ģʽ
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//�洢������ģʽ
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//�������ݳ���:8λ
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//�洢�����ݳ���:8λ
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;// ʹ����ͨģʽ 
  DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;//�е����ȼ�
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;//�洢��ͻ�����δ���
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//����ͻ�����δ���
  DMA_Init(DMA1_Stream4, &DMA_InitStructure);//��ʼ��DMA Stream
	
	DMA_ITConfig(DMA1_Stream4, DMA_IT_TC, ENABLE);            // ��������DMAͨ���ж�
		
	/* ����DMA�ж����ȼ�*/
  NVIC_InitStructure.NVIC_IRQChannel                   = DMA1_Stream4_IRQn;           
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;          
  NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 1; 
  NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}
void UART4_Init(u32 bound)
{
	GPIO_InitTypeDef  GPIO_InitStructure;   /*������ؽṹ��*/
	USART_InitTypeDef  USART_InitStructure;
	NVIC_InitTypeDef  NVIC_InitStructure;   
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);//�� ʹ����Ӧ��ʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,ENABLE);
	
	//����4���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource10,GPIO_AF_UART4); //GPIOC10����ΪUART4
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource11,GPIO_AF_UART4); //GPIOC11����ΪUART4

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;//GPIOC10 GPIOC11
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOC,&GPIO_InitStructure);//��  ��ʼ����Ӧ��IO��ģʽ
	
	USART_InitStructure.USART_BaudRate=bound;//������
	USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;//��Ӳ������
	USART_InitStructure.USART_Mode=USART_Mode_Tx|USART_Mode_Rx;//ģʽ
	USART_InitStructure.USART_Parity=USART_Parity_No;//У��
	USART_InitStructure.USART_StopBits=USART_StopBits_1;//ֹͣλ
	USART_InitStructure.USART_WordLength=USART_WordLength_8b;//���ݳ���
	
	USART_Init(UART4,&USART_InitStructure);//�� ��ʼ��������ز���
	
	
	USART_ClearFlag(UART4, USART_FLAG_TC);
	
	//USART_ITConfig(UART4,USART_IT_RXNE,ENABLE);//���������ж�   ENABLE  DISABLE
	USART_ITConfig(UART4,USART_IT_IDLE,ENABLE);
	USART_GetFlagStatus(UART4,USART_FLAG_TC);
	USART_Cmd(UART4,ENABLE);//ʹ�ܴ���4
  
	NVIC_InitStructure.NVIC_IRQChannel=UART4_IRQn;//ͨ��
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;//�����ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;//�����ж����ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0;//�����ȼ�
	NVIC_Init(&NVIC_InitStructure);
	
  //����4����DMA����
	DMA_Use_UART4_Rx_Init();
	//����4����DMA����
	MYDMA_UART4_Config();
}
static void delay_um(uint32_t delay)
{
	while(delay--);
}

static uint8_t deal_irq_rx_end(uint8_t *buf)  
{   
    uint16_t len = 0;	
    /* ��������ж� */
    if(USART_GetITStatus(UART4, USART_IT_IDLE) != RESET)  
    {  
				delay_um(500000);//���Ӵ���ʱ�����յ��������쳣
        UART4->SR;  
        UART4->DR; /*���USART_IT_IDLE��־*/
			  /* �رս���DMA  */
        DMA_Cmd(DMA1_Stream2,DISABLE);  
        /*�����־λ*/
        DMA_ClearFlag(DMA1_Stream2,DMA_FLAG_TCIF2);  
				
        /*��ý���֡֡��*/
        len = USART4_REC_LEN - DMA_GetCurrDataCounter(DMA1_Stream2); 
				if((uart4_rx_buf[0]==0x5a)&&(uart4_rx_buf[1]==0xa5))
					DMA_Rx_Lifter_Flag = 1;
				if((uart4_rx_buf[0]==0xcf)&&(uart4_rx_buf[1] == 0X01))//�Ƕ�Э�� 0xcf 0x01/0x02(0x01�������Ƕȡ�0x02����ˮƽ�Ƕ�) 0x00 0x00 0x00 0x00(����λ��������)
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
        /*���ô������ݳ���*/
        DMA_SetCurrDataCounter(DMA1_Stream2,USART4_REC_LEN);  
        /* ��DMA */
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
		Height_Flag1 = 0;//����ͳɹ�
		Height_Flag2 = 1;//���յ����ݣ��������ѷ��ͳɹ�
	}
	if((uart4_buf[0]==0X5A)&&(uart4_buf[1]==0XA5)&&DMA_Rx_Lifter_Flag)
	{
			DMA_Rx_Lifter_Flag = 0;
			Height = (((uart4_buf[2]<<8)|uart4_buf[3]))/28.0;
			//printf("������������: 0x%x,�����˸߶ȣ�%fmm\r\n",(((uart4_buf[2]<<8)|uart4_buf[3])),Height);
	}
}
void YT_Contr(void)
{
	uint8_t i=0;
	if(Yt_Ack)
	{
		printf("RECEVIE ANGLE ACK!!!\r\n");
		Yt_Ack = 0;
		Yt_Flag1 = 0;//���յ����ݣ��������ѷ��ͳɹ�
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
			
			printf("�����Ƕȣ�%f\r\n",Vangle.real);
	}
	if((uart4_buf[0]==0XCF)&&(uart4_buf[1]==0X02)&&DMA_Rx_Hangel_Flag)
	{
			DMA_Rx_Hangel_Flag = 0;
			for(i = 0;i < 4;i++)
			{
				Hangle.base[i] = uart4_buf[5-i];
			}
			H_angle = Hangle.real + 33.0;
			printf("ˮƽ�Ƕȣ�%f\r\n",Hangle.real);
	}
	#endif
	if((uart4_buf[0]==0XCF)&&(uart4_buf[1]==0X03)&&DMA_Rx_Angel_Flag)
	{
			DMA_Rx_Angel_Flag = 0;
		  //for(i = 0;i < 4;i++)
			if(uart4_buf[2] == 0x59)
			{
				H_angle = ((uart4_buf[3]<<8) | uart4_buf[4])/100.0;
				printf("ˮƽ�Ƕȣ�%f\r\n",H_angle);
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
				printf("�����Ƕȣ�%f\r\n",V_angle);
			}
	}
}



void DMA1_Stream4_IRQHandler(void)
{
    if(DMA_GetFlagStatus(DMA1_Stream4,DMA_FLAG_TCIF4)!=RESET)
    {
      DMA_ClearFlag(DMA1_Stream4,DMA_FLAG_TCIF4);//���DMA1_Steam6������ɱ�־
			DMA_Cmd(DMA1_Stream4, DISABLE); //�ر�DMA���� 
    }
}

//����һ��DMA����
//DMA_Streamx:DMA������,DMA1_Stream0~7/DMA2_Stream0~7 
//ndtr:���ݴ�����  
void MYDMA_UART4_Enable(uint8_t ndtr)
{
 
	DMA_Cmd(DMA1_Stream4, DISABLE);                      //�ر�DMA���� 	
	while (DMA_GetCmdStatus(DMA1_Stream4) != DISABLE){}	//ȷ��DMA���Ա�����  
	DMA_SetCurrDataCounter(DMA1_Stream4,ndtr);          //���ݴ�����  
	DMA_Cmd(DMA1_Stream4, ENABLE);                      //����DMA���� 
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
	USART_DMACmd(UART4,USART_DMAReq_Tx,ENABLE);  //ʹ�ܴ���2��DMA���� 
	MYDMA_UART4_Enable(Length+1);     //��ʼһ��DMA���䣡
	#else
	while(i<Length)
	{
		printf("����%d��0x%x",i,Buffer[i]);
		UART4->SR;
		USART_SendData(UART4, Buffer[i++]);
	  while(USART_GetFlagStatus(UART4,USART_FLAG_TC)==RESET);//�ȴ��������
	}
	#endif
}
#ifdef __cplusplus
}
#endif