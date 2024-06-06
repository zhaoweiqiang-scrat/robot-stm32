#ifdef __cplusplus
extern "C" {
#endif
//Sensors_ULtrasonic_driver ks103������������
#include "ks103.h"
#include "sys.h"
#include "millisecondtimer.h"
	
#define RS485_RX_EN		PEout(9)	//485ģʽ����.0,ʹ�ܽ���
#define RS485_TX_EN		PEout(8)	//485ģʽ����.1,ʹ�ܷ���.

u8 uart7_rx_buf[2];
u8  usartlen;
char usart7_ch;
u16 Ultr_distance;	
void KS103_Init(u32 bound)
{
	GPIO_InitTypeDef  GPIO_InitStructure;   /*������ؽṹ��*/
	USART_InitTypeDef  USART_InitStructure;
	NVIC_InitTypeDef  NVIC_InitStructure;   
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);//�� ʹ����Ӧ��ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6,ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOF,&GPIO_InitStructure);//��  ��ʼ����Ӧ��IO��ģʽ
	
	//PE8\PE9���������485ģʽ����  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9; //GPIOE8 GPIOE9
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //�������
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOE,&GPIO_InitStructure); //��ʼ��PE8\PE9
	
	USART_InitStructure.USART_BaudRate=bound;//������
	USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;//��Ӳ������
	USART_InitStructure.USART_Mode=USART_Mode_Tx|USART_Mode_Rx;//ģʽ
	USART_InitStructure.USART_Parity=USART_Parity_No;//У��
	USART_InitStructure.USART_StopBits=USART_StopBits_1;//ֹͣλ
	USART_InitStructure.USART_WordLength=USART_WordLength_8b;//���ݳ���
	
	USART_Init(UART7,&USART_InitStructure);//�� ��ʼ��������ز���
	
	USART_Cmd(UART7,ENABLE);//ʹ�ܴ���1
	
	USART_ITConfig(UART7,USART_IT_RXNE,DISABLE);//���������ж�
	
	NVIC_InitStructure.NVIC_IRQChannel=UART7_IRQn;//ͨ��
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;//�����ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;//�����ж����ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=1;//�����ȼ�
	NVIC_Init(&NVIC_InitStructure);
	
	RS485_TX_EN = 1;//Ĭ�Ϸ���
}

void	UART7_IRQHandler(void)
{
	if(USART_GetITStatus(USART1,USART_IT_RXNE))
	{
		usart7_ch=USART_ReceiveData(UART7);
		uart7_rx_buf[usartlen++]=usart7_ch;
	}	
}

	
void KS103_Send(void)
{	
	  u8	AskCmd[3]={0xE8,0x02,0xBC};
		int i;
		for(i=0;i<3;i++)
		{	
			USART_SendData(UART7, AskCmd[i]);
			while(USART_GetFlagStatus(UART7, USART_FLAG_TXE) == RESET){}; //�ȴ��������
		}
	return;
}

void Sensors_Ultrasonic_task(void)
{ 
		if(usartlen == 2)
		{
			RS485_TX_EN=1;				//����ģʽ
			usartlen = 0;
			Ultr_distance = uart7_rx_buf[0]*256+uart7_rx_buf[1];
			KS103_Send();	//����̽��ָ��
			RS485_RX_EN=0;//����ģʽ
			//delay(90);	
		}
}
#ifdef __cplusplus
}
#endif