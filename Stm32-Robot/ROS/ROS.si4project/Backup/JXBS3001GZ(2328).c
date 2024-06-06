#ifdef __cplusplus
extern "C" {
#endif
//��ǿ�ȼ�⴫����JXBS-3001-GZ
#include "JXBS3001GZ.h"
#include "millisecondtimer.h"

/************************************************
 
************************************************/
u8  AskCmd[8]={0x01,0x03,0x00,0x07,0x00,0x02,0x75,0xca};
u8 crc_H,crc_L;
u8 usart6_rx_buf[10];
float fdata;
u8  usartlen;
char usart6_ch;

union data
{
	unsigned char A[4];
	float Temp;
};

void JXBS3001GZ_Init(u32 bound)
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
	GPIO_Init(GPIOC,&GPIO_InitStructure);//��  ��ʼ����Ӧ��IO��ģʽ
	
	USART_InitStructure.USART_BaudRate=bound;//������
	USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;//��Ӳ������
	USART_InitStructure.USART_Mode=USART_Mode_Tx|USART_Mode_Rx;//ģʽ
	USART_InitStructure.USART_Parity=USART_Parity_No;//У��
	USART_InitStructure.USART_StopBits=USART_StopBits_1;//ֹͣλ
	USART_InitStructure.USART_WordLength=USART_WordLength_8b;//���ݳ���
	
	USART_Init(USART6,&USART_InitStructure);//�� ��ʼ��������ز���
	
	USART_Cmd(USART6,ENABLE);//ʹ�ܴ���1
	
	USART_ITConfig(USART6,USART_IT_RXNE,DISABLE);//���������ж�
	
	NVIC_InitStructure.NVIC_IRQChannel=USART6_IRQn;//ͨ��
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;//�����ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;//�����ж����ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=1;//�����ȼ�
	NVIC_Init(&NVIC_InitStructure);
}

void	USART6_IRQHandler(void)
{
	if(USART_GetITStatus(USART1,USART_IT_RXNE))
	{
		
		usart6_ch=USART_ReceiveData(USART6);
		usart6_rx_buf[usartlen++]=usart6_ch;
	}	
}

	
void JXBS3001GZ_Send(u8 *buf)
{	
		int i;
		for(i=0;i<8;i++)
		{	
		
			USART_SendData(USART6, buf[i]);
			while(USART_GetFlagStatus(USART6, USART_FLAG_TXE) == RESET){}; //�ȴ��������
		}
	return;
}

void numinit()
{
	int h;
	for( h=0;h<12;h++)
	{
		usart6_rx_buf[h]=0;
	}
}

u16 crc_check(u8 *ptr , int len)
{
	u16 crc16=0xffff;
	int i,j,tmp;
	for(i=0;i<len;i++)
	{
		crc16=*ptr^crc16;
		for(j=0;j<8;j++)
		{
			tmp=crc16&0x0001;
			crc16=crc16>>1;
			if(tmp)
				crc16=crc16^0xa001;
		}
		*ptr++;
	}
	return crc16;
	
}
void calculate(void)
{	

	union data  a;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//�����ж����ȼ�����Ϊ��2��2λ��ռ���ȼ���2λ��Ӧ���ȼ�

	JXBS3001GZ_Init(9600);
	JXBS3001GZ_Send(AskCmd);
	delay(1000);
	numinit();
 	while(1)
	{
			USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);
			if(usartlen>=9)
			{
					a.A[0]=usart6_rx_buf[3];
					a.A[1]=usart6_rx_buf[4];
					a.A[2]=usart6_rx_buf[5];
					a.A[3]=usart6_rx_buf[6];
					fdata=a.Temp;
					usartlen=0;		
			}
			
			delay(100);
	}
}	
#ifdef __cplusplus
}
#endif