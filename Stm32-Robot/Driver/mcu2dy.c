#ifdef __cplusplus
extern "C" {
#endif
#include "mcu2dy.h"

#define USART3_REC_LEN  			20 	
u8  usart3_len;
char usart3_ch;
u8 usart3_rx_buf[USART3_REC_LEN];
u8 usart3_buf[USART3_REC_LEN];
static u8 Col_Rx_Flag,Bat_Rx_Flag;
u8 Collision_Front_L,Collision_Front_R,Collision_Later_L,SD_Rx_Flag,Collision_Later_R;

static void DMA_Use_USART3_Rx_Init(void)
{
    DMA_InitTypeDef DMA_InitStructure;
		/*ʹ��DMA1ʱ��*/
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

    /* ����ʹ��DMA�������� */
    DMA_DeInit(DMA1_Stream1); 

    DMA_InitStructure.DMA_Channel             = DMA_Channel_4;               /* ����DMAͨ�� */
    DMA_InitStructure.DMA_PeripheralBaseAddr  = (uint32_t)(&(USART3->DR));   /* Դ*/
    DMA_InitStructure.DMA_Memory0BaseAddr     = (uint32_t)usart3_rx_buf;      /* Ŀ�� */
    DMA_InitStructure.DMA_DIR                 = DMA_DIR_PeripheralToMemory;    /* ���� ����--���洢�� */
    DMA_InitStructure.DMA_BufferSize          = USART3_REC_LEN;                    /* ���� */                  
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

    DMA_Init(DMA1_Stream1, &DMA_InitStructure);

    USART_DMACmd(USART3,USART_DMAReq_Rx,ENABLE);

    /* ʹ��DMA */ 
    DMA_Cmd(DMA1_Stream1,ENABLE);
}

void USART3_Init(u32 bound)
{
	GPIO_InitTypeDef  GPIO_InitStructure;   /*������ؽṹ��*/
	USART_InitTypeDef  USART_InitStructure;
	NVIC_InitTypeDef  NVIC_InitStructure;   
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);//�� ʹ����Ӧ��ʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
	
	//����7���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART3); //GPIOB10����ΪUSART3
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3); //GPIOB11����ΪUSART3
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;//GPIOF10 GPIOF11
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOB,&GPIO_InitStructure);//��  ��ʼ����Ӧ��IO��ģʽ
	
	USART_InitStructure.USART_BaudRate=bound;//������
	USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;//��Ӳ������
	USART_InitStructure.USART_Mode=USART_Mode_Tx|USART_Mode_Rx;//ģʽ
	USART_InitStructure.USART_Parity=USART_Parity_No;//У��
	USART_InitStructure.USART_StopBits=USART_StopBits_1;//ֹͣλ
	USART_InitStructure.USART_WordLength=USART_WordLength_8b;//���ݳ���
	
	USART_Init(USART3,&USART_InitStructure);//�� ��ʼ��������ز���
	
	USART_Cmd(USART3,ENABLE);//ʹ�ܴ���3
	
//	USART_ITConfig(USART3,USART_IT_RXNE,ENABLE);//���������ж�  ENABLE  DISABLE
	USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);  // ��������IDLE�ж�
	USART_GetFlagStatus(USART3,USART_FLAG_TC);
	NVIC_InitStructure.NVIC_IRQChannel=USART3_IRQn;//ͨ��
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;//�����ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=5;//�����ж����ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0;//�����ȼ�
	NVIC_Init(&NVIC_InitStructure);
	
	DMA_Use_USART3_Rx_Init();
}

static uint8_t deal_irq_rx_end(uint8_t *buf)  
{     
    uint16_t len = 0;  
    /* ��������ж� */
    if(USART_GetITStatus(USART3, USART_IT_IDLE) != RESET)  
    {  
        USART3->SR;  
        USART3->DR; /*���USART_IT_IDLE��־*/
        /* �رս���DMA  */
        DMA_Cmd(DMA1_Stream1,DISABLE);  
        /*�����־λ*/
        DMA_ClearFlag(DMA1_Stream1,DMA_FLAG_TCIF1);  

        /*��ý���֡֡��*/
        len = USART3_REC_LEN - DMA_GetCurrDataCounter(DMA1_Stream1);  
        memcpy(buf,usart3_rx_buf,len); 
				if(usart3_rx_buf[0] == 0xff)
				{
					Col_Rx_Flag = 1;
					COL2MCU();
				}
				if((usart3_rx_buf[0] == 0xee)&&(usart3_rx_buf[1] == 0x05))
				{
					SD_Rx_Flag = 1;
					//BAT2MCU();
				}   
				if(usart3_rx_buf[0] == 0xdd)
				{
					Bat_Rx_Flag = 1;
					BAT2MCU();
				}
				    
        /*���ô������ݳ���*/
        DMA_SetCurrDataCounter(DMA1_Stream1,USART3_REC_LEN);  
        /* ��DMA */
        DMA_Cmd(DMA1_Stream1,ENABLE);  
        return len;  
    }   

    return 0;  
}

void USART3_IRQHandler(void) 
{
		deal_irq_rx_end(usart3_buf);
}
void COL2MCU(void)
{
	if((Col_Rx_Flag)&&(usart3_buf[0]==0Xff))
	{
		Col_Rx_Flag = 0;
	
		if((usart3_buf[1]==0x01)&&(usart3_buf[2]==0x01))//��ǰ����ײ
	  {
		  Collision_Front_L = 1;
			Collision_Front_R = 0;
		  Collision_Later_L = 0;
			Collision_Later_R = 0;
	  }
		else if((usart3_buf[1]==0x02)&&(usart3_buf[2]==0x01))//��ǰ����ײ
	  {
		  Collision_Front_L = 0;
			Collision_Front_R = 1;
		  Collision_Later_L = 0;
			Collision_Later_R = 0;
	  }
		else if((usart3_buf[1]==0x03)&&(usart3_buf[2]==0x01))//�������ײ
	  {
		  Collision_Front_L = 0;
			Collision_Front_R = 0;
		  Collision_Later_L = 1;
			Collision_Later_R = 0;
	  }
	  else if((usart3_buf[1]==0x04)&&(usart3_buf[2]==0x01))	//�Һ�����ײ
	  {
		  Collision_Front_L = 0;
			Collision_Front_R = 0;
		  Collision_Later_L = 0;
			Collision_Later_R = 1;
	  }
	  else //����ײ
	  {
		  Collision_Front_L = 0;
			Collision_Front_R = 0;
		  Collision_Later_L = 0;
			Collision_Later_R = 0;
	  }
	}	
}
float Voltage,Current;
float Surplus_capacity,Total_capacity;//���ʣ��������������
float Surplus_capacity_percent;//ʣ�������ٷֱ�
u16 State_protection;//����״̬
void BAT2MCU(void)
{
	if((Bat_Rx_Flag)&&(usart3_buf[0]==0XDD))
	{
		Bat_Rx_Flag = 0;
		Voltage = (usart3_buf[1]<<8|usart3_buf[2])/100.0;//��ص�ѹ����λ10mv������100����λ��ΪV
		//��ص�������λ10mA,���λΪ1����Ϊ�ŵ磬����100������λ��ΪA
		Current = ((usart3_buf[3]<<8|usart3_buf[4])&0x8000) ? ((0xffff - (usart3_buf[3]<<8|usart3_buf[4]))/(-100.0)):((usart3_buf[3]<<8|usart3_buf[4])/100.0);
		Surplus_capacity = (usart3_buf[5]<<8|usart3_buf[6])/100.0;//��λ10mAH,��ΪAH
		Total_capacity = (usart3_buf[7]<<8|usart3_buf[8])/100.0;//��λ10mAH,��ΪAH
		State_protection = usart3_buf[9]<<8|usart3_buf[10];
		Surplus_capacity_percent = usart3_buf[11]/100.0;//ʣ�������ٷֱ�
	}
}
//input:byte,�����͵�����
static void USART_send_byte(uint8_t byte)
{
	USART3->SR;
	USART_SendData(USART3, byte);	
	while(USART_GetFlagStatus(USART3,USART_FLAG_TC)==RESET);//�ȴ��������
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

uint8_t CMD_Info_Sta[4] = {0XA5,0X5A,0X03,0XFF};//��03 ��ȡ������Ϣ��״̬
uint8_t CMD_Bat_Vol[4] = {0XA5,0X5A,0X04,0XFF}; //��04 ��ȡ��ص����ѹ
uint8_t CMD_Hw_Ver[4] = {0XA5,0X5A,0X05,0XFF};  //��05 ��ȡ������Ӳ���汾��
//STM32F429��STM32F103
void MCU2DY(u8 state)
{
	uint8_t CMD_Close_Charge[4] = {0XA5,0X5A,0X01,0XFF};//�رճ�翪��
  uint8_t CMD_Open_Charge[4] = {0XA5,0X5A,0X02,0XFF};//�򿪳�翪��
	if(state)
	  USART_Send_bytes(CMD_Open_Charge,4);
	else
		USART_Send_bytes(CMD_Close_Charge,4);
}
#ifdef __cplusplus
}
#endif