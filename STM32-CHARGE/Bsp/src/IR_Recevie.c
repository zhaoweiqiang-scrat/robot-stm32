#include "main.h"
#include "stm32f10x_tim.h"

#define IR_MARGIN		((WORD)130UL)
#define time_9000_us	((WORD)9000UL)
#define time_4500_us	((WORD)4500UL)
#define time_2250_us	((WORD)2250UL)
#define time_1125_us	((WORD)1125UL)

// State of the remote key decoder
#define IDLE		0
#define LEADER_ON	1
#define LEADER_OFF	2
#define ADDRESS		3
#define DATA1		4
#define DATA2		5
#define IR_ADDR  0x7F80

#define RISING_EDGE   1
#define FALLING_EDGE  0

uint8_t  Receive_Flag_start,Receive_Flag_stop,Receive_Num1,Receive_Num2;

//�ⲿ�ж�0�������
void EXTIX_Init(void)
{
 	EXTI_InitTypeDef EXTI_InitStructure;
 	NVIC_InitTypeDef NVIC_InitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);	//ʹ�ܸ��ù���ʱ��

    //GPIOE.2 �ж����Լ��жϳ�ʼ������   �½��ش���
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOC,GPIO_PinSource13);

  EXTI_InitStructure.EXTI_Line=EXTI_Line13;	//IR_RXD
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);	 	//����EXTI_InitStruct��ָ���Ĳ�����ʼ������EXTI�Ĵ���

  NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;			//ʹ�ܰ���WK_UP���ڵ��ⲿ�ж�ͨ��
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;	//��ռ���ȼ�2�� 
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x03;					//�����ȼ�3
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//ʹ���ⲿ�ж�ͨ��
  NVIC_Init(&NVIC_InitStructure); 
}
//�趨���ش���-----������or�½���  1�������أ�0���½���
void IR_EXTI_Edge_Trigger(uint8_t ring_failing)
{
	EXTI_InitTypeDef EXTI_InitStructure;
	  
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC,GPIO_PinSource13);

  EXTI_InitStructure.EXTI_Line=EXTI_Line13;	//IR_RXD
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  
	if(ring_failing)
	{
	 	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	}
	else
	{
		EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	}
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);	 	//����EXTI_InitStruct��ָ���Ĳ�����ʼ������EXTI�Ĵ���
}
void deal_irq_end(void)
{
	static BYTE	state = 0;	// State holder
	static BYTE		count;							// bits counter
	unsigned long time_count, t0;
	static BYTE		data1;					// Temporary for holding the decoded data
	/*NECң��ָ������ݸ�ʽΪ��ͬ����ͷ����ַ�롢��ַ���롢�����롢���Ʒ���
	  ͬ������һ��9ms�ĵ͵�ƽ+һ��4.5ms�ĸߵ�ƽ��ɣ���ַ�롢��ַ���롢����
	  �롢���Ʒ������8λ���ݸ�ʽ*/
	switch (state) 
	{
	  case IDLE:		//ͬ�����½��أ��͵�ƽ����9ms
							TIM_SetCounter(TIM2, 0);	
		          TIM_Cmd(TIM2, ENABLE);	
	            time_count = TIM_GetCounter(TIM2);	
		          IR_EXTI_Edge_Trigger(RISING_EDGE);//�޸ı��ش����ж�Ϊ�����ش���
		          state = LEADER_ON;//��һ��״̬Ϊͬ����������
		          break;
    case LEADER_ON:        //����ͬ����
	            time_count = TIM_GetCounter(TIM2);
		          TIM_SetCounter(TIM2, 0);	
		          TIM_Cmd(TIM2, ENABLE);	
		          t0 = time_count;
				      IR_EXTI_Edge_Trigger(FALLING_EDGE);//�޸ı��ش����ж�Ϊ�½��ش���
              state =  ((t0>(time_9000_us-8*IR_MARGIN)) && (t0<(time_9000_us+8*IR_MARGIN))) ? LEADER_OFF:IDLE;//�͵�ƽ����ʱ���Ƿ�Ϊ9ms  
		          break;
    case LEADER_OFF:    //����ͬ����
	            time_count = TIM_GetCounter(TIM2);
		          TIM_SetCounter(TIM2, 0);	
		          TIM_Cmd(TIM2, ENABLE);	
		          t0 = time_count;
		          if ((t0 > time_4500_us - (4*IR_MARGIN)) && (t0 < time_4500_us + (4*IR_MARGIN)) )//�ߵ�ƽ����ʱ���Ƿ�Ϊ4.5ms
							{
			          //state = ADDRESS;//׼����ʼ���յ�ַ�롢��ַ���롢�����롢���Ʒ��룬�½��ش����ж�
			          //address = 0;
			          //count = 0;
								state = DATA1;
			          count = 0;
			          data1 = 0;
		          } 
		          else 
							{
			          IR_EXTI_Edge_Trigger(FALLING_EDGE);
			          state = IDLE;//���ճ������¿�ʼ���½��ش���
		          }	
		          break;
    case DATA1:
	           time_count = TIM_GetCounter(TIM2);
		         TIM_SetCounter(TIM2, 0);	
		         TIM_Cmd(TIM2, ENABLE);	
		         t0 = time_count ;
		         count++;
		         if ( (t0 > time_1125_us - 3*IR_MARGIN) && (t0 < time_1125_us + 3*IR_MARGIN)) 
						 {
			         data1 <<= 1;	/* a zero bit */
		         } 
		         else 
						 { 
			         if ((t0 > time_2250_us - 3*IR_MARGIN) && (t0 < time_2250_us + 4*IR_MARGIN)) 
							 {
				         data1 = (data1 << 1) | 1;	/* a one bit */
			         } 
			         else 
						   {
			          IR_EXTI_Edge_Trigger(FALLING_EDGE);
				        state = IDLE;
				        break;
               }
			       }
		         if (count == 8) 
						 {      	
			         IR_EXTI_Edge_Trigger(FALLING_EDGE);
			         state = IDLE;
								if(data1 == 0x39)
								{
									Receive_Flag_start = 1;//�����˿�ʼѲ��
									Receive_Flag_stop = 0;//ֹͣ���
									Receive_Num1++;
									Receive_Num2 = 0;
									if(Receive_Num1 == 1)
									{
										CHARGE_DIS;
										AC220_DIS;
										CHARGE_STATE_DIS;
										electromagnetism_OFF;
									}	
								}
								else if(data1 == 0x3A)
								{
									Receive_Flag_start = 0;//ֹͣѲ��
									Receive_Flag_stop = 1;//���յ�stm32f429�ĺ��ⷢ���ź�,�����˿�ʼ���
									Receive_Num2++;
									Receive_Num1 = 0;
									if(Receive_Num2 == 1)
									{
										electromagnetism_ON;
									}
								}
		         }
		         break;
  }	
}
