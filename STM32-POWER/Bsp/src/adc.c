#include "main.h"

#define N 50 //ÿ��ͨ����50��
#define M 7  //7��ͨ��

vu16 AD_Value[N][M]; //�������ADC���������Ҳ��DMA��Ŀ���ַ
vu16 After_filter[M]; //���������ƽ��ֵ��Ľ��
_ADC_VALUE_ ADC_VALUE;


void Adc_Init(void)
{
	ADC_InitTypeDef ADC_InitStructure; 
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_ADC1	, ENABLE );	  //ʹ��ADC1ͨ��ʱ��
	
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);   //����ADC��Ƶ����6 72M/6=12,ADC���ʱ�䲻�ܳ���14M

	//PB0 ��Ϊ5V���ͨ��           ADC1_IN8        
	GPIO_InitStructure.GPIO_Pin = V5V_DET_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//ģ����������
	GPIO_Init(V5V_DET_GPIO_Port, &GPIO_InitStructure);	

	//PA1 ��Ϊ3.3V���ͨ��    ADC1_IN1                  
	GPIO_InitStructure.GPIO_Pin = V3V3_DET_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//ģ����������
	GPIO_Init(V3V3_DET_GPIO_Port, &GPIO_InitStructure);
	
	//PA4 ��Ϊ12V���ͨ��      ADC1_IN4               
	GPIO_InitStructure.GPIO_Pin = V12_DET_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//ģ����������
	GPIO_Init(V12_DET_GPIO_Port, &GPIO_InitStructure);
	
	//PA5 ��Ϊ19V���ͨ��     ADC1_IN5        
	GPIO_InitStructure.GPIO_Pin = V19V_1_DET_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//ģ����������
	GPIO_Init(V19V_1_DET_GPIO_Port, &GPIO_InitStructure);
	
	//PA6 ��Ϊ19V���ͨ��   ADC1_IN6               
	GPIO_InitStructure.GPIO_Pin = V19V_2_DET_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//ģ����������
	GPIO_Init(V19V_2_DET_GPIO_Port, &GPIO_InitStructure);
	
	//PB1 ��Ϊϵͳ�������ͨ��ADC1_IN9
	GPIO_InitStructure.GPIO_Pin = CUR_DET_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//ģ����������
	GPIO_Init(CUR_DET_GPIO_Port, &GPIO_InitStructure);
	
	//PA7 ��Ϊϵͳ�����ѹ���ͨ��  ADC1_IN7              
	GPIO_InitStructure.GPIO_Pin = ADC_Batt_V_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//ģ����������
	GPIO_Init(ADC_Batt_V_GPIO_Port, &GPIO_InitStructure);
	
	ADC_DeInit(ADC1);  //��λADC1 

	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;	//ADC����ģʽ:ADC1��ADC2�����ڶ���ģʽ
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;	//ģ��ת��������ɨ��ģʽ
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;	//ģ��ת������������ת��ģʽ
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//ת��������������ⲿ��������
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	//ADC�����Ҷ���
	ADC_InitStructure.ADC_NbrOfChannel = 7;	//˳����й���ת����ADCͨ������Ŀ
	ADC_Init(ADC1, &ADC_InitStructure);	//����ADC_InitStruct��ָ���Ĳ�����ʼ������ADCx�ļĴ���   

	ADC_Cmd(ADC1, ENABLE);	//ʹ��ָ����ADC1
	
	ADC_ResetCalibration(ADC1);	//ʹ�ܸ�λУ׼  
	 
	while(ADC_GetResetCalibrationStatus(ADC1));	//�ȴ���λУ׼����
	
	ADC_StartCalibration(ADC1);	 //����ADУ׼
 
	while(ADC_GetCalibrationStatus(ADC1));	 //�ȴ�У׼����
}

void ADC_DMA_Configuration(void)
{
	DMA_InitTypeDef DMA_InitStructure;
	DMA_DeInit(DMA1_Channel1); //��DMA��ͨ��1�Ĵ�������Ϊȱʡֵ
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&ADC1->DR; //DMA����ADC����ַ
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&AD_Value; //DMA�ڴ����ַ
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC; //�ڴ���Ϊ���ݴ����Ŀ���ַ
	DMA_InitStructure.DMA_BufferSize = N*M; //DMAͨ����DMA����Ĵ�С
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; //�����ַ�Ĵ�������
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; //�ڴ��ַ�Ĵ�������
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; //���ݿ��Ϊ16λ
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord; //���ݿ��Ϊ16λ
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular; //������ѭ������ģʽ
	DMA_InitStructure.DMA_Priority = DMA_Priority_High; //DMAͨ��1ӵ�и����ȼ�
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable; //DMAͨ��1û������Ϊ�ڴ浽�ڴ�Ĵ���
	DMA_Init(DMA1_Channel1, &DMA_InitStructure); //��ʼ��DMAͨ��1
}
//���ADCֵ
//ch:ͨ��ֵ 
u16 Get_Adc(u8 ch)   
{
  	//����ָ��ADC�Ĺ�����ͨ����һ�����У�����ʱ��
	ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_239Cycles5 );	//ADC1,ADCͨ��,����ʱ��Ϊ239.5����	  			    
  
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);		//ʹ��ָ����ADC1�����ת����������	
	 
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));//�ȴ�ת������

	return ADC_GetConversionValue(ADC1);	//�������һ��ADC1�������ת�����
}

u16 Get_Adc_Average(u8 ch,u8 times)
{
	u32 temp_val=0;
	u8 t;
	for(t=0;t<times;t++)
	{
		temp_val+=Get_Adc(ch);
		delay_ms(5);
	}
	return temp_val/times;
} 	

void GetValue(void)
{
	u16 adcx;
	
	adcx=Get_Adc_Average(ADC_Channel_1,10);
	ADC_VALUE.ADC_Val_3V3 = 2*adcx*3.3f/4096;
	adcx=Get_Adc_Average(ADC_Channel_4,10);
	ADC_VALUE.ADC_Val_12V = 7.8*adcx*3.3f/4096;
	adcx=Get_Adc_Average(ADC_Channel_5,10);
	ADC_VALUE.ADC_Val_19V1 = 7.8*adcx*3.3f/4096;
	adcx=Get_Adc_Average(ADC_Channel_6,10);
	ADC_VALUE.ADC_Val_19V2 = 7.8*adcx*3.3f/4096;
	adcx=Get_Adc_Average(ADC_Channel_7,10);
	ADC_VALUE.ADC_Val_BAT = 10*adcx*3.3f/4096;
	adcx=Get_Adc_Average(ADC_Channel_8,10);
	ADC_VALUE.ADC_Val_5V = 2*adcx*3.3f/4096;
	adcx=Get_Adc_Average(ADC_Channel_9,10);
	ADC_VALUE.ADC_Val_CUR = 4*adcx*3.3f/4096;
}
u16 GetVolt(u16 advalue)
{
	return (u16)(advalue * 3.3 / 4096); 
}

void filter(void)
{
	int sum = 0;
	u8 i,count;
	for(i=0;i<12;i++)
	{
		for ( count=0;count<N;count++)
		{	
			sum += AD_Value[count][i];
		}		
		After_filter[i]=sum/N;
		sum=0;
	}
}
void Value(u16 *value)
{
	u8 i;
	
	filter();
	for(i=0;i<M;i++)
	{
		value[i] = GetVolt(After_filter[i]);
	}
	delay_ms(500);
}
