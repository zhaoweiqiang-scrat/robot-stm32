#ifdef __cplusplus
extern "C" {
#endif
#include "IR_Receive.h"
#include "IR.h"
#include "mcu2dy.h"
	
u32 Receive_Code;	  //?�??????32???y?Y????????n???????
u8  Receive_Flag,Receive_Flag1,Receive_Flag2,Receive_Flag3;  //?�??????8???y?Y????????????????????????
u8  Receive_Flag_Backup,Receive_Flag1_Backup,Receive_Flag2_Backup,Receive_Flag3_Backup;
u8  Receive_Flag1_Q,Receive_Flag2_Q,Receive_Flag3_Q;
u8  Tim_Cnt;
u8  Receive_Data[3];
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
//#define IR_CODE1 0x7F8035CA
//#define IR_CODE2 0x7F8036C9
//#define IR_CODE3 0x7F8037C8
#define IR_CODE1 0x36 //????????
#define IR_CODE2 0x37 //?D??????
#define IR_CODE3 0x35 //????????
#define IR_OK    0x38
#define RISING_EDGE   1
#define FALLING_EDGE  0

static uint8_t deal_irq_end(void);
u8 Charge_Pos,Recevie_Num=1,Recevie_Ok;
//?�???�???�3?D??3????�
//arr?o???�???~????
//psc?o???????????y
//?�???�??3????????????�:Tout=((arr+1)*(psc+1))/Ft us.
//Ft=?�???�1??�????,????:Mhz
//?a???1???????�???�3!
void TIM2_Int_Init(u16 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);  ///?1??TIM2????
	
  TIM_TimeBaseInitStructure.TIM_Period = arr; 	//???�???~????
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc;  //?�???�????
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //???????y????
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStructure);//3????�TIM2

	TIM_Cmd(TIM2,ENABLE); //?1???�???�3
}
//?�???�???�10?D??3????�
//arr?o???�???~????
//psc?o???????????y
//?�???�??3????????????�:Tout=((arr+1)*(psc+1))/Ft us.
//Ft=?�???�1??�????,????:Mhz
//?a???1???????�???�10!
void TIM10_Int_Init(u16 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10,ENABLE);  ///?1??TIM10????
	
  TIM_TimeBaseInitStructure.TIM_Period = arr; 	//???�???~????
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc;  //?�???�????
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //???????y????
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM10,&TIM_TimeBaseInitStructure);//3????�TIM10
	
	TIM_ITConfig(TIM10,TIM_IT_Update,ENABLE); //??D??�???�10??D??D??
	TIM_Cmd(TIM10,ENABLE); //?1???�???�10
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM1_UP_TIM10_IRQn; //?�???�10?D??
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01; //??????????1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x03; //????????3
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

/*******************************************************************************
* o� ?y ??         : IR_Receive_Init
* o�?y1|??		   : o??a????3????�o�?y	  ?????????~?a2??D??3????� 
* ??    ??         : ?T
* ??    3?         : ?T
*******************************************************************************/
void IR_Receive_Init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	/* ?a??GPIO?????~1????n?????? */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE); //?n??????n??a
	
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOI,EXTI_PinSource11);
	
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN; 
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_11;//1???????
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;//???-
	GPIO_Init(GPIOI,&GPIO_InitStructure); //3????�??11??
	
	EXTI_ClearITPendingBit(EXTI_Line11);
	
	/* ?????a2??D???????? */ 
	EXTI_InitStructure.EXTI_Line=EXTI_Line11;
	EXTI_InitStructure.EXTI_Mode=EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger=EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd=ENABLE;
	EXTI_Init(&EXTI_InitStructure); 

	/* ????NVIC2??y */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;   //n??a?????D??
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; //???????????a0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1; 	 //???|???????a1
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;   //?1??
	NVIC_Init(&NVIC_InitStructure);

	TIM2_Int_Init(10000-1,84-1);	//?�???�????84M?????????y8400??????84M/840=1Mhz?????y?????????y10000n??a10ms 
	TIM10_Int_Init(4000-1,84-1);  //?�???�????84M?????????y84??????84M/84=1Mhz?????y?????????y4000n??a4ms ???n4ms????n???3??D??
}
//???�????n???-----???y??or??????  1?o???y????0?o??????
void IR_EXTI_Edge_Trigger(u8 ring_failing)
{
	EXTI_InitTypeDef EXTI_InitStructure;
	  
	/* EXTI line(PA1) mode config */
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOI,EXTI_PinSource11);
	EXTI_InitStructure.EXTI_Line = EXTI_Line11;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	if(ring_failing)
	{
	 	 EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising; 
	}
	else
	{
		EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; 
	}
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure); 
}

void EXTI15_10_IRQHandler(void)	  //o??a?????a2??D??
{
	static BYTE	state = 0;	// State holder
	static WORD		address;		// Hold the custom (remote ID) code
	static BYTE		count;							// bits counter
	unsigned long time_count, t0;
	static BYTE		data1, data2;					// Temporary for holding the decoded data
	int i = 255;
	/*NEC???????????y?Y?????a?o??2????????????????????n?????????????????n??
	  ??2?????????9ms????????+????4.5ms??????????3????????????????n????????
	  ?????????n??????8???y?Y????*/
	if(EXTI_GetITStatus(EXTI_Line11)!=RESET)//?D??????????
	{
		switch (state) 
	{
	  case IDLE:		//??2?????????????????3?D?9ms
        	    TIM_SetCounter(TIM2, 0);	
		          TIM_Cmd(TIM2, ENABLE);	
	            time_count = TIM_GetCounter(TIM2);	
		          IR_EXTI_Edge_Trigger(RISING_EDGE);//DT??????n????D???a???y??n???
		          state = LEADER_ON;//???????n???a??2??????y??
		          break;
    case LEADER_ON:        //n|????2???
	            time_count = TIM_GetCounter(TIM2);
		          TIM_SetCounter(TIM2, 0);	
		          TIM_Cmd(TIM2, ENABLE);
		          t0 = time_count;
				      IR_EXTI_Edge_Trigger(FALLING_EDGE);//DT??????n????D???a??????n???
              state =  ((t0>(time_9000_us-8*IR_MARGIN)) && (t0<(time_9000_us+8*IR_MARGIN))) ? LEADER_OFF:IDLE;//??????3?D??????????a9ms  
		          break;
    case LEADER_OFF:    //n|????2???
	            time_count = TIM_GetCounter(TIM2);
		          TIM_SetCounter(TIM2, 0);	
		          TIM_Cmd(TIM2, ENABLE);
		          t0 = time_count;
		          if ((t0 > time_4500_us - (4*IR_MARGIN)) && (t0 < time_4500_us + (4*IR_MARGIN)) )//??????3?D??????????a4.5ms
							{
			          //state = ADDRESS;//?????a???????????????????n?????????????????n??????????n????D??
			          //address = 0;
			          //count = 0;
								state = DATA1;
			          count = 0;
			          data1 = 0;
		          } 
		          else 
							{
			          IR_EXTI_Edge_Trigger(FALLING_EDGE);
			          state = IDLE;//????3?n?????D??a??????????n???
		          }	
		          break;
		#if 0
    case ADDRESS://n|???????????????n??
		          time_count = TIM_GetCounter(TIM2);
		          TIM_SetCounter(TIM2, 0);	
		          t0 = time_count ;
							/*???????o???????-1n???D??a2.25ms?�560us??3?+1680us??????),???????-0n???D??a1.125ms(560us??3?+560us??????),
		            o??a??????????????3????a???????????D??3????a??????
								???????o???-1?|????560us??+1680us???????-0?|????560us??+560us??*/
		          if ((t0 > time_1125_us - 3*IR_MARGIN) && (t0 < time_1125_us + 4*IR_MARGIN)) 
							{
			          address <<= 1;	/* a zero bit */
		          } 
		          else 
							{
			          if ( (t0 > time_2250_us - 2*IR_MARGIN) && (t0 < time_2250_us + 3*IR_MARGIN)) 
								{
				          address = (address << 1) | 1;	/* a one bit */
			          } 
			          else 
							  {
				          IR_EXTI_Edge_Trigger(FALLING_EDGE);
				          state = IDLE;
				          break;
                }
			        }
		         /* count 16 'custom' bits */
		         if (++count == 16) 
						 {
			        if (address != IR_ADDR) 
							{
				        state = IDLE;
				        break;
			        }
			        state = DATA1;
			        count = 0;
			        data1 = 0;
		         }
		         break;
    case DATA1://n|????????
		         time_count = TIM_GetCounter(TIM2);
		         TIM_SetCounter(TIM2, 0);	
		         t0 = time_count;
		         count++;
		         if ( (t0 > time_1125_us - 3*IR_MARGIN) && (t0 <time_1125_us + 3*IR_MARGIN)) 
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
			        state = DATA2;
			        count = 0;
			        data2 = 0;
		         }
		         break;
		#endif
    case DATA1:
	           time_count = TIM_GetCounter(TIM2);
		         TIM_SetCounter(TIM2, 0);	
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
								if((data1 == IR_CODE1)||(data1 == IR_CODE2)||(data1 == IR_CODE3)||(data1 == IR_OK))
								{
									Recevie_Num++;
									printf("0x%x\r\n",data1);
									if(data1 == IR_CODE1)
									{
										Receive_Flag1 = 1;//????????????o??a????D?o?
										Receive_Flag1_Q++;
									}
									else if(data1 == IR_CODE2)
									{
										Receive_Flag2 = 1;//???????D????o??a????D?o?
										Receive_Flag2_Q++;
									}
									else if(data1 == IR_CODE3)
									{
										Receive_Flag3 = 1;//????????????o??a????D?o?
										Receive_Flag3_Q++;
									}
									else if(data1 == IR_OK)
									{
										Receive_Flag = 1;
									}
								}
								else
								{
										Receive_Flag = 0;
										Receive_Flag1 = 0;
										Receive_Flag2 = 0;
										Receive_Flag3 = 0;
								}
		         }
		         break;
  }
	Recevie_Ok = 1;
	if((Recevie_Num%4) == 0)
	{
		Recevie_Num = 1;
		Receive_Flag1_Backup = Receive_Flag1;
		Receive_Flag2_Backup = Receive_Flag2;
		Receive_Flag3_Backup = Receive_Flag3;
		Receive_Flag_Backup = Receive_Flag;
		
		Receive_Flag = 0;
	  Receive_Flag1 = 0;
	  Receive_Flag2 = 0;
	  Receive_Flag3 = 0;
		deal_irq_end();
		if(!Charge_Pos)
		{
			//printf("????D?o?????:%d\r\n",Receive_Flag1_Q);
	    //printf("?D??D?o?????:%d\r\n",Receive_Flag2_Q);
	    //printf("????D?o?????:%d\r\n",Receive_Flag3_Q);
		}
		//printf("3?????????????:%d\r\n",Charge_Pos);
		
	}
	EXTI_ClearITPendingBit(EXTI_Line11);   
	}
	
	
	 //?????ao??a???2???|??n|??n???
	
	//while(i--);//????
	if(EXTI_GetITStatus(EXTI_Line10)!=RESET)//?D??????????
	{
		if(GPIO_ReadInputDataBit(GPIOG,GPIO_Pin_10))
		{
			IR_RIGHT = 1;//D???13cm
		}
		else
		{
			IR_RIGHT = 0;//n???13cm
		}
    EXTI_ClearITPendingBit(EXTI_Line10);//??3yLINE10??????????
	}
}
static uint8_t deal_irq_end(void) 
{
	if(!Receive_Flag_Backup&&!Receive_Flag1_Backup&&!Receive_Flag2_Backup&&!Receive_Flag3_Backup)
	{
		Charge_Pos = 0;//?????y???????n?????D????
	}
	else if(Receive_Flag1_Backup&&Receive_Flag2_Backup&&Receive_Flag3_Backup)
	{
		Charge_Pos = 1;//?????y???????n??????????
	}
	else if(!Receive_Flag1_Backup&&Receive_Flag2_Backup&&!Receive_Flag3_Backup)
	{
		Charge_Pos = 2;//?????????????D??????D?o?
	}
	else if(Receive_Flag1_Backup&&!Receive_Flag2_Backup&&!Receive_Flag3_Backup)
	{
		Charge_Pos = 3;//????????????????????D?o?
	}
	else if(!Receive_Flag1_Backup&&!Receive_Flag2_Backup&&Receive_Flag3_Backup)
	{
		Charge_Pos = 4;//????????????????????D?o?
	}
	else if(Receive_Flag1_Backup&&Receive_Flag2_Backup&&!Receive_Flag3_Backup)
	{
		Charge_Pos = 5;//???????????????????D??????D?o?
	}
	else if(!Receive_Flag1_Backup&&Receive_Flag2_Backup&&Receive_Flag3_Backup)
	{
		Charge_Pos = 6;//???????????????????D??????D?o?
	}
	else if(Receive_Flag1_Backup&&!Receive_Flag2_Backup&&Receive_Flag3_Backup)
	{
		Charge_Pos = 7;//???????????????????D??????D?o?
	}
	else if(Receive_Flag_Backup)
	{
		Charge_Pos = 8;
	}
}
u8 compare_sucess;
u8 Compare_Data(u8 Std_Data)
{
	u8 i;
	compare_sucess = 0;
	for(i = 0; i < 3; i++)
	{
		if(Receive_Data[i] == Std_Data)compare_sucess++;
	}
	if(compare_sucess)return compare_sucess;
	else return 0;
}
u8 Com_IR1,Com_IR2,Com_IR3,Com_IR,Zero_Num;
void Charge_Contr()
{
	Com_IR = Compare_Data(IR_OK);
	Com_IR1 = Compare_Data(IR_CODE1);
	Com_IR2 = Compare_Data(IR_CODE2);
	Com_IR3 = Compare_Data(IR_CODE3);
	
	if(!Com_IR&&!Com_IR1&&!Com_IR2&&!Com_IR3)
	{
		Zero_Num++;
		if(Zero_Num == 5)
		{
			Zero_Num = 0;
			Charge_Pos = 0;//?????y???????n?????D????
		}
		
	}
	else if(Com_IR1&&Com_IR2&&Com_IR3)
	{
		Zero_Num = 0;
		Charge_Pos = 1;//?????y???????n??????????
	}
	else if(!Com_IR1&&Com_IR2&&!Com_IR3)
	{
		Zero_Num = 0;
		Charge_Pos = 2;//?????????????D??????D?o?
	}
	else if(Com_IR1&&!Com_IR2&&!Com_IR3)
	{
		Zero_Num = 0;
		Charge_Pos = 3;//????????????????????D?o?
	}
	else if(!Com_IR1&&!Com_IR2&&Com_IR3)
	{
		Zero_Num = 0;
		Charge_Pos = 4;//????????????????????D?o?
	}
	else if(Com_IR1&&Com_IR2&&!Com_IR3)
	{
		Zero_Num = 0;
		Charge_Pos = 5;//???????????????????D??????D?o?
	}
	else if(!Com_IR1&&Com_IR2&&Com_IR3)
	{
		Zero_Num = 0;
		Charge_Pos = 6;//???????????????????D??????D?o?
	}
	else if(Com_IR1&&!Com_IR2&&Com_IR3)
	{
		Zero_Num = 0;
		Charge_Pos = 7;//???????????????????D??????D?o?
	}
	else if(Com_IR)
	{
		Zero_Num = 0;
		Charge_Pos = 8;
	}
	//printf("���λ��:%d\r\n",Charge_Pos);
}


u8 i;
void Move_Data(u8 data)
{
	Receive_Data[i++] = data;
	if(i == 3)i=0;
}
//?�???�10?D???t??o�?y
void TIM1_UP_TIM10_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM10,TIM_IT_Update)==SET) //??3??D??
	{
		Tim_Cnt++;
		if(Receive_Flag)
		{
			Receive_Flag = 0;
			Tim_Cnt = 0;
			Move_Data(IR_OK);
		}
		if(Receive_Flag1)
		{
			Receive_Flag1 = 0;
			Tim_Cnt = 0;
			Move_Data(IR_CODE1);
		}
		if(Receive_Flag2)
		{
			Receive_Flag2 = 0;
			Tim_Cnt = 0;
			Move_Data(IR_CODE2);
		}
		if(Receive_Flag3)
		{
			Receive_Flag3 = 0;
			Tim_Cnt = 0;
			Move_Data(IR_CODE3);
		}
		if(Tim_Cnt >= 20)
		{
			Tim_Cnt = 0;
			Move_Data(0);
		}		
	}
	TIM_ClearITPendingBit(TIM10,TIM_IT_Update);  //??3y?D????????
}


//��ʱ��3�жϷ�����
extern float encoder_count1;
extern float encoder_count2;
int count_TIM4,count_TIM5;
float count_TIM4_delt,count_TIM5_delt;
//this function calculates the motor's RPM based on encoder ticks and delta time
void TIM7_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM7,TIM_IT_Update)==SET) //����ж�
	{
		count_TIM4 = TIM_GetCounter(TIM4);
		count_TIM5 = TIM_GetCounter(TIM5);
		count_TIM4_delt =(count_TIM4 - 32768)/4.0;
		count_TIM5_delt =(count_TIM5 - 32768)/4.0;
		encoder_count1 = count_TIM4_delt*20*60/17100.0;//50ms��ʱ����convert the time from milliseconds to minutes 
		encoder_count2 = count_TIM5_delt*20*60/17100.0;//50ms��ʱ����convert the time from milliseconds to minutes

		TIM_SetCounter(TIM4,32768); 
		TIM_SetCounter(TIM5,32768);
		
	}
	TIM_ClearITPendingBit(TIM7,TIM_IT_Update);  //����жϱ�־λ
}


#ifdef __cplusplus
}
#endif


