#ifdef __cplusplus
extern "C" {
#endif
#include "stm32f4xx.h"
#include "string.h"
#include "millisecondtimer.h"
void GYMPU680_Init(uint32_t bound)
{
	GPIO_InitTypeDef GPIO_usartx;
	USART_InitTypeDef Usart_X;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOD, ENABLE);
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
  USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);//���������ж�
  USART_Cmd(UART5, ENABLE);
}
//input:byte,�����͵�����
void USART_send_byte(uint8_t byte)
{
	while(USART_GetFlagStatus(USART2,USART_FLAG_TC)==RESET);//�ȴ��������
	USART2->DR=byte;	
}
//���Ͷ��ֽ�����
void USART_Send_bytes(uint8_t *Buffer, uint8_t Length)
{
	uint8_t i=0;
	while(i<Length)
	{
		USART_send_byte(Buffer[i++]);
	}
}
//���Ͷ��ֽ�����+У���
void USART_Send(uint8_t *Buffer, uint8_t Length)
{
	uint8_t i=0;
	while(i<Length)
	{
		if(i<(Length-1))
		Buffer[Length-1]+=Buffer[i];//�ۼ�Length-1ǰ������
		USART_send_byte(Buffer[i++]);
	}
}
//����һ֡����
void send_out(int16_t *data,uint8_t length,uint8_t send)
{
	uint8_t TX_DATA[30],i=0,k=0;
	memset(TX_DATA,0,(2*length+5));//���㻺��TX_DATA
	TX_DATA[i++]=0X5A;//֡ͷ
	TX_DATA[i++]=0X5A;//֡ͷ
	TX_DATA[i++]=send;//�����ֽ�
	TX_DATA[i++]=2*length;//���ݳ���
	for(k=0;k<length;k++)//�������ݵ�����TX_DATA
	{
		TX_DATA[i++]=(uint16_t)data[k]>>8;
		TX_DATA[i++]=(uint16_t)data[k];
	}
	USART_Send(TX_DATA,i);	
}
void send_8bit_out(uint8_t *data,uint8_t length,uint8_t send)
{
	uint8_t TX_DATA[50],i=0,k=0;
	memset(TX_DATA,0,(2*length+5));//���㻺��TX_DATA
	TX_DATA[i++]=0X5A;//֡ͷ
	TX_DATA[i++]=0X5A;//֡ͷ
	TX_DATA[i++]=send;//�����ֽ�
	TX_DATA[i++]=length;//���ݳ���
	for(k=0;k<length;k++)//�������ݵ�����TX_DATA
	{
		TX_DATA[i++]=(uint16_t)data[k];
	}
	USART_Send(TX_DATA,i);	
}
uint8_t RX_BUF[50]={0},stata=0;
//У��ͼ��
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
static void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_X;
  
  /* 4����ռ���ȼ���4����Ӧ���ȼ� */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  /*��ռ���ȼ��ɴ���жϼ���͵��ж�*/
	/*��Ӧ���ȼ����ȼ�ִ��*/
	NVIC_X.NVIC_IRQChannel = USART2_IRQn;//�ж�����
  NVIC_X.NVIC_IRQChannelPreemptionPriority = 0;//��ռ���ȼ�
  NVIC_X.NVIC_IRQChannelSubPriority = 0;//��Ӧ���ȼ�
  NVIC_X.NVIC_IRQChannelCmd = ENABLE;//ʹ���ж���Ӧ
  NVIC_Init(&NVIC_X);
}
void send_Instruction(void)
{
	uint8_t send_data[4]={0};
	send_data[0]=0xa5;
	send_data[1]=0x55;
	send_data[2]=0x3F;
	send_data[3]=0x39;
	USART_Send_bytes(send_data,4);//����
	
	delay(100);
	
	send_data[0]=0xa5;
	send_data[1]=0x56;
	send_data[2]=0x02;
	send_data[3]=0xfd;
	USART_Send_bytes(send_data,4);//�����Զ����ָ��
	delay(100);
}
void UART5_IRQHandler(void)
{
	static uint8_t rebuf[13]={0},i=0;
	if(USART_GetITStatus(UART5, USART_IT_RXNE) != RESET)
	{
		rebuf[i++]=USART_ReceiveData(UART5);
		if(rebuf[0]!=0x5a)//�ж�֡ͷ
			i=0;
	  if((i==2)&&(rebuf[1]!=0x5a))//�ж�֡ͷ
			i=0;
		if(i>4)//��i����ֵ=5ʱ�������ֽڽ�����ϣ����ݳ����ֽڽ������
		{
			if(i==rebuf[3]+5)
			{
	       memcpy(RX_BUF,rebuf,i);
				stata=1;
				i=0;
			}
		}
		USART_ClearFlag(UART5,USART_FLAG_RXNE);//���жϱ�־
	}	
}
int calculate_TH()
{
	uint8_t data_buf[50]={0},count=0;
	float Temperature ,Humidity;
  uint32_t Gas;
  uint32_t Pressure;
  uint16_t IAQ;
	int16_t Altitude=0;
  uint8_t IAQ_accuracy;
	uint16_t temp1=0;
  int16_t temp2=0;
	
	send_Instruction();//��ģ�鷢��ָ��
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
				  printf("Temperature: %.2f 'C", Temperature);
          count=2;
			 }
			  if(data_buf[2]&0x02) //Humidity
			 {  
			    temp1=((uint16_t)data_buf[4+count]<<8)|data_buf[5+count];
				  Humidity=(float)temp1/100; 
				  printf(" ,Humidity: %.2f %% ", Humidity);
          count+=2;
			 }
			  if(data_buf[2]&0x04) //Pressure
			 {
			    Pressure=((uint32_t)data_buf[4+count]<<16)|((uint16_t)data_buf[5+count]<<8)|data_buf[6+count];
				  printf(" ,Pressure: %d Pa", Pressure);
          count+=3;
			 }
			 
			  if(data_buf[2]&0x08) //IAQ_accuracy��IAQ
			 {
			   IAQ_accuracy=(data_buf[4+count]&0xf0)>>4;
				 IAQ=(((uint16_t)data_buf[4+count]&0x000f)<<8)|data_buf[5+count];
		
				 printf(" ,IAQ: %d ,IAQ_accuracy: %d",IAQ,IAQ_accuracy);
          count+=2;
			 }

			   if(data_buf[2]&0x10) //Gas
			 {
			    Gas =((uint32_t)data_buf[4+count]<<24)|((uint32_t)data_buf[5+count]<<16)|((uint16_t)data_buf[6+count]<<8)|data_buf[7+count]; 
				  printf(" ,Gas: %d ohm ", Gas);
          count+=4;
			 }
			   if(data_buf[2]&0x10)//����
				 {
				    Altitude=((int16_t)data_buf[4+count]<<8)|data_buf[5+count];
				    printf(" ,Altitude: %d m \r\n", Altitude);
				 }
			 
	}
	return 0;
}

#ifdef __cplusplus
}
#endif