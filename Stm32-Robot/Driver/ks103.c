#ifdef __cplusplus
extern "C" {
#endif
#if 0
//Sensors_ULtrasonic_driver ks103������������
#include "ks103.h"
#include "sys.h"
#include "millisecondtimer.h"

#define RS485_RX_EN PFout(9) //485ģʽ����.0,ʹ�ܽ���
#define RS485_TX_EN PFout(8) //485ģʽ����.1,ʹ�ܷ���.
#define UART7_REC_LEN  			20 	//�����������ֽ��� 200	
u8 uart7_rx_buf[UART7_REC_LEN];
u8 Ultr_Data[UART7_REC_LEN];
u8  usart7_len;
char usart7_ch;
static u8 Reset;
u16 Ultr_distance_F_left,Ultr_distance_F_middle,Ultr_distance_F_right,Ultr_distance_left,Ultr_distance_right,Ultr_distance_A_left,Ultr_distance_A_right;	
u8 Flag_F_left,Flag_F_middle,Flag_F_right,Flag_left,Flag_right,Flag_A_left,Flag_A_right,Rec_Ok;
u8 Flag_F_left_send,Flag_F_middle_send,Flag_F_right_send,Flag_left_send,Flag_right_send,Flag_A_left_send,Flag_A_right_send;
	
static void DMA_Use_UART7_Rx_Init(void)
{
    DMA_InitTypeDef DMA_InitStructure;
		/*ʹ��DMA2ʱ��*/
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

    /* ����ʹ��DMA�������� */
    DMA_DeInit(DMA1_Stream3); 

    DMA_InitStructure.DMA_Channel             = DMA_Channel_5;               /* ����DMAͨ�� */
    DMA_InitStructure.DMA_PeripheralBaseAddr  = (uint32_t)(&(UART7->DR));   /* Դ*/
    DMA_InitStructure.DMA_Memory0BaseAddr     = (uint32_t)uart7_rx_buf;      /* Ŀ�� */
    DMA_InitStructure.DMA_DIR                 = DMA_DIR_PeripheralToMemory;    /* ���� ����--���洢�� */
    DMA_InitStructure.DMA_BufferSize          = UART7_REC_LEN;                    /* ���� */                  
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

    DMA_Init(DMA1_Stream3, &DMA_InitStructure);

    USART_DMACmd(UART7,USART_DMAReq_Rx,ENABLE);

    /* ʹ��DMA */ 
    DMA_Cmd(DMA1_Stream3,ENABLE);
}
void KS103_Init(u32 bound)
{
	GPIO_InitTypeDef  GPIO_InitStructure;   /*������ؽṹ��*/
	USART_InitTypeDef  USART_InitStructure;
	NVIC_InitTypeDef  NVIC_InitStructure;   
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF,ENABLE);//�� ʹ����Ӧ��ʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART7,ENABLE);
	
	//����7���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOF,GPIO_PinSource6,GPIO_AF_UART7); //GPIOF6����ΪUART7
	GPIO_PinAFConfig(GPIOF,GPIO_PinSource7,GPIO_AF_UART7); //GPIOF7����ΪUART7
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;//GPIOF6 GPIOF7
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOF,&GPIO_InitStructure);//��  ��ʼ����Ӧ��IO��ģʽ
	
	//PF8\PF9���������485ģʽ����  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9; //GPIOF8 GPIOF9
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //�������
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOF,&GPIO_InitStructure); //��ʼ��PE8\PE9
	
	USART_InitStructure.USART_BaudRate=bound;//������
	USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;//��Ӳ������
	USART_InitStructure.USART_Mode=USART_Mode_Tx|USART_Mode_Rx;//ģʽ
	USART_InitStructure.USART_Parity=USART_Parity_No;//У��
	USART_InitStructure.USART_StopBits=USART_StopBits_1;//ֹͣλ
	USART_InitStructure.USART_WordLength=USART_WordLength_8b;//���ݳ���
	
	USART_Init(UART7,&USART_InitStructure);//�� ��ʼ��������ز���
	
	USART_Cmd(UART7,ENABLE);//ʹ�ܴ���7
	
	//USART_ITConfig(UART7,USART_IT_RXNE,ENABLE);//���������ж�
	USART_ITConfig(UART7, USART_IT_IDLE, ENABLE);  // ��������IDLE�ж�
	USART_GetFlagStatus(UART7,USART_FLAG_TC);
	NVIC_InitStructure.NVIC_IRQChannel=UART7_IRQn;//ͨ��
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;//�����ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=6;//�����ж����ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0;//�����ȼ�
	NVIC_Init(&NVIC_InitStructure);
	DMA_Use_UART7_Rx_Init();
	
	RS485_RX_EN = 1;//Ĭ�Ϸ���
	RS485_TX_EN = 1;
	Flag_left = 1;
	Flag_right = 0;
}
static uint8_t deal_irq_rx_end(uint8_t *buf)  
{     
    uint16_t len = 0;  
    /* ��������ж� */
    if(USART_GetITStatus(UART7, USART_IT_IDLE) != RESET)  
    {  
        UART7->SR;  
        UART7->DR; /*���USART_IT_IDLE��־*/
        /* �رս���DMA  */
        DMA_Cmd(DMA1_Stream3,DISABLE);  
        /*�����־λ*/
        DMA_ClearFlag(DMA1_Stream3,DMA_FLAG_TCIF3);  

        /*��ý���֡֡��*/
        len = UART7_REC_LEN - DMA_GetCurrDataCounter(DMA1_Stream3);  
				if((len == 3)&&((uart7_rx_buf[0] == 0xd0)||(uart7_rx_buf[0] == 0xd2)||(uart7_rx_buf[0] == 0xd4)||(uart7_rx_buf[0] == 0xd6)||(uart7_rx_buf[0] == 0xd8)||(uart7_rx_buf[0] == 0xda)||(uart7_rx_buf[0] == 0xdc)))
					memcpy(buf,uart7_rx_buf,len);  
				Reset = 0;
			  Rec_Ok = 1;
        /*���ô������ݳ���*/
        DMA_SetCurrDataCounter(DMA1_Stream3,UART7_REC_LEN);  
        /* ��DMA */
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

	
void KS103_Send(u8 *buf)
{	
		int i;
	
	  RS485_TX_EN = 1;				//����ģʽ
		RS485_RX_EN = 1;

		for(i=0;i<3;i++)
		{	
			USART_SendData(UART7, buf[i]);
			while(USART_GetFlagStatus(UART7, USART_FLAG_TC) == RESET){}; //�ȴ��������
		}
		if(buf[0] == 0Xda)
		{
			Flag_F_left_send = 1;//������߳�����������
			Flag_F_middle_send = 0;
			Flag_F_right_send = 0;
			
			Flag_left_send = 0;
			Flag_right_send = 0;
			
			Flag_A_left_send = 0;
			Flag_A_right_send = 0;
		}
		else if(buf[0] == 0Xd0)
		{
			Flag_F_left_send = 0;
			Flag_F_middle_send = 1;
			Flag_F_right_send = 0;
			
			Flag_left_send = 0;
			Flag_right_send = 0;
			
			Flag_A_left_send = 0;
			Flag_A_right_send = 0;
		}
		else if(buf[0] == 0Xd2)
		{
			Flag_F_left_send = 0;
			Flag_F_middle_send = 0;
			Flag_F_right_send = 1;
			
			Flag_left_send = 0;
			Flag_right_send = 0;
			
			Flag_A_left_send = 0;
			Flag_A_right_send = 0;
		}
		else if(buf[0] == 0Xd4)
		{
			Flag_F_left_send = 0;
			Flag_F_middle_send = 0;
			Flag_F_right_send = 0;
			
			Flag_left_send = 1;
			Flag_right_send = 0;
			
			Flag_A_left_send = 0;
			Flag_A_right_send = 0;
		}
		else if(buf[0] == 0Xdc)
		{
			Flag_F_left_send = 0;
			Flag_F_middle_send = 0;
			Flag_F_right_send = 0;
			
			Flag_left_send = 0;
			Flag_right_send = 1;
			
			Flag_A_left_send = 0;
			Flag_A_right_send = 0;
		}
		else if(buf[0] == 0Xd8)
		{
			Flag_F_left_send = 0;
			Flag_F_middle_send = 0;
			Flag_F_right_send = 0;
			
			Flag_left_send = 0;
			Flag_right_send = 0;
			
			Flag_A_left_send = 1;
			Flag_A_right_send = 0;
		}
		else if(buf[0] == 0Xd6)
		{
			Flag_F_left_send = 0;
			Flag_F_middle_send = 0;
			Flag_F_right_send = 0;
			
			Flag_left_send = 0;
			Flag_right_send = 0;
			
			Flag_A_left_send = 0;
			Flag_A_right_send = 1;
		}
		RS485_RX_EN = 0;//����ģʽ
		RS485_TX_EN = 0;
		
	return;
}

int Sensors_Ultrasonic_task1(void)
{ 
    u8	AskCmd1[3]={0xda,0x02,0xBC};
		u8	AskCmd2[3]={0xd2,0x02,0xBC};
		u8	AskCmd3[3]={0xd2,0x02,0xBC};
		
		Reset++;
		if(Flag_F_left_send&&Rec_Ok)
		{
			Rec_Ok = 0;
			Flag_F_left = 1;
			Ultr_distance_F_left = Ultr_Data[1]*256+Ultr_Data[2];
			KS103_Send(AskCmd3);	//����̽��ָ��
			return 0;
		}	
		
		if(Reset >= 10)
		{
			KS103_Send(AskCmd1);	//����̽��ָ��
			return 1;
		}
			
		return 1;
}
int Sensors_Ultrasonic_task2(void)
{ 
		u8	AskCmd3[3]={0xd4,0x02,0xBC};
		
		if(Flag_F_middle_send&&Rec_Ok)
		{
			Rec_Ok = 0;
			Flag_F_middle = 1;
			Ultr_distance_F_middle = uart7_rx_buf[1]*256+uart7_rx_buf[2];
			KS103_Send(AskCmd3);	//����̽��ָ��
			return 0;
		}
		return 1;
}
int Sensors_Ultrasonic_task3(void)
{ 
		u8	AskCmd1[3]={0xda,0x02,0xBC};
		u8	AskCmd4[3]={0xd6,0x02,0xBC};
		u8	AskCmd6[3]={0xd8,0x02,0xBC};
		
	  //Reset++;
		if(Flag_F_right_send&&Rec_Ok)
		{
			Rec_Ok = 0;
			Flag_F_right = 1;
			Ultr_distance_F_right = uart7_rx_buf[1]*256+uart7_rx_buf[2];
			KS103_Send(AskCmd6);	//����̽��ָ��
			return 0;
		}
		return 1;
}
int Sensors_Ultrasonic_task4(void)
{ 
    u8	AskCmd1[3]={0xd0,0x02,0xBC};
		u8	AskCmd2[3]={0xd2,0x02,0xBC};
		u8	AskCmd3[3]={0xd4,0x02,0xBC};
		u8	AskCmd4[3]={0xd6,0x02,0xBC};
		u8	AskCmd5[3]={0xd8,0x02,0xBC};
		u8	AskCmd6[3]={0xda,0x02,0xBC};
		u8	AskCmd7[3]={0xdc,0x02,0xBC};
		
	  //Reset++;
		if(Flag_left_send&&Rec_Ok)
		{
			Rec_Ok = 0;
			Flag_left = 1;
			Ultr_distance_left = uart7_rx_buf[1]*256+uart7_rx_buf[2];
			KS103_Send(AskCmd5);	//����̽��ָ��
			return 0;
		}
		return 1;
}
int Sensors_Ultrasonic_task5(void)
{ 
    u8	AskCmd1[3]={0xd0,0x02,0xBC};
		u8	AskCmd2[3]={0xd2,0x02,0xBC};
		u8	AskCmd3[3]={0xd4,0x02,0xBC};
		u8	AskCmd4[3]={0xd6,0x02,0xBC};
		u8	AskCmd5[3]={0xd8,0x02,0xBC};
		u8	AskCmd6[3]={0xda,0x02,0xBC};
		u8	AskCmd7[3]={0xdc,0x02,0xBC};
		
	  //Reset++;
		if(Flag_right_send&&Rec_Ok)
		{
			Rec_Ok = 0;
			Flag_right = 1;
			Ultr_distance_right = uart7_rx_buf[1]*256+uart7_rx_buf[2];
			KS103_Send(AskCmd6);	//����̽��ָ��
			return 0;
		}
		return 1;
}
int Sensors_Ultrasonic_task6(void)
{ 
		u8	AskCmd7[3]={0xd6,0x02,0xBC};
		
	  //Reset++;
		if(Flag_A_left_send&&Rec_Ok)
		{
			Rec_Ok = 0;
			Flag_A_left = 1;
			Ultr_distance_A_left = uart7_rx_buf[1]*256+uart7_rx_buf[2];
			KS103_Send(AskCmd7);	//����̽��ָ��
			return 0;
		}
		return 1;
}
int Sensors_Ultrasonic_task7(void)
{ 
    u8	AskCmd1[3]={0xda,0x02,0xBC};
		
	  //Reset++;
		if(Flag_A_right_send&&Rec_Ok)
		{
			Rec_Ok = 0;
			Flag_A_right = 1;
			Ultr_distance_A_right = uart7_rx_buf[1]*256+uart7_rx_buf[2];
			KS103_Send(AskCmd1);	//����̽��ָ��
			return 0;
		}
		return 1;
}
void Change_Address(u8 OldAddress,u8 NewAddress)
{
	USART_SendData(UART7, OldAddress);
	while(USART_GetFlagStatus(UART7,USART_FLAG_TC)!=1); //�ȴ��������
	USART_SendData(UART7, 0x02);
	while(USART_GetFlagStatus(UART7,USART_FLAG_TC)!=1); //�ȴ��������
	USART_SendData(UART7, 0x9a);
	while(USART_GetFlagStatus(UART7,USART_FLAG_TC)!=1); //�ȴ��������
	delay(20);
	
	USART_SendData(UART7, OldAddress);
	while(USART_GetFlagStatus(UART7,USART_FLAG_TC)!=1); //�ȴ��������
	USART_SendData(UART7, 0x02);
	while(USART_GetFlagStatus(UART7,USART_FLAG_TC)!=1); //�ȴ��������
	USART_SendData(UART7, 0x92);
	while(USART_GetFlagStatus(UART7,USART_FLAG_TC)!=1); //�ȴ��������
	delay(20);
	
	USART_SendData(UART7, OldAddress);
	while(USART_GetFlagStatus(UART7,USART_FLAG_TC)!=1); //�ȴ��������
	USART_SendData(UART7, 0x02);
	while(USART_GetFlagStatus(UART7,USART_FLAG_TC)!=1); //�ȴ��������
	USART_SendData(UART7, 0x9e);
	while(USART_GetFlagStatus(UART7,USART_FLAG_TC)!=1); //�ȴ��������
	delay(20);
	
	USART_SendData(UART7, OldAddress);
	while(USART_GetFlagStatus(UART7,USART_FLAG_TC)!=1); //�ȴ��������
	USART_SendData(UART7, 0x02);
	while(USART_GetFlagStatus(UART7,USART_FLAG_TC)!=1); //�ȴ��������
	USART_SendData(UART7, NewAddress);
	while(USART_GetFlagStatus(UART7,USART_FLAG_TC)!=1); //�ȴ��������
	delay(2000);
}
#endif
#ifdef __cplusplus
}
#endif