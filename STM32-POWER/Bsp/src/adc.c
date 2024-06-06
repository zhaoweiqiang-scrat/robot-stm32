#include "main.h"

#define N 50 //每次通道采50次
#define M 7  //7个通道

vu16 AD_Value[N][M]; //用来存放ADC采样结果，也是DMA的目标地址
vu16 After_filter[M]; //用来存放求平均值后的结果
_ADC_VALUE_ ADC_VALUE;


void Adc_Init(void)
{
	ADC_InitTypeDef ADC_InitStructure; 
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_ADC1	, ENABLE );	  //使能ADC1通道时钟
	
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);   //设置ADC分频因子6 72M/6=12,ADC最大时间不能超过14M

	//PB0 作为5V检测通道           ADC1_IN8        
	GPIO_InitStructure.GPIO_Pin = V5V_DET_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//模拟输入引脚
	GPIO_Init(V5V_DET_GPIO_Port, &GPIO_InitStructure);	

	//PA1 作为3.3V检测通道    ADC1_IN1                  
	GPIO_InitStructure.GPIO_Pin = V3V3_DET_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//模拟输入引脚
	GPIO_Init(V3V3_DET_GPIO_Port, &GPIO_InitStructure);
	
	//PA4 作为12V检测通道      ADC1_IN4               
	GPIO_InitStructure.GPIO_Pin = V12_DET_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//模拟输入引脚
	GPIO_Init(V12_DET_GPIO_Port, &GPIO_InitStructure);
	
	//PA5 作为19V检测通道     ADC1_IN5        
	GPIO_InitStructure.GPIO_Pin = V19V_1_DET_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//模拟输入引脚
	GPIO_Init(V19V_1_DET_GPIO_Port, &GPIO_InitStructure);
	
	//PA6 作为19V检测通道   ADC1_IN6               
	GPIO_InitStructure.GPIO_Pin = V19V_2_DET_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//模拟输入引脚
	GPIO_Init(V19V_2_DET_GPIO_Port, &GPIO_InitStructure);
	
	//PB1 作为系统电流检测通道ADC1_IN9
	GPIO_InitStructure.GPIO_Pin = CUR_DET_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//模拟输入引脚
	GPIO_Init(CUR_DET_GPIO_Port, &GPIO_InitStructure);
	
	//PA7 作为系统输入电压检测通道  ADC1_IN7              
	GPIO_InitStructure.GPIO_Pin = ADC_Batt_V_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//模拟输入引脚
	GPIO_Init(ADC_Batt_V_GPIO_Port, &GPIO_InitStructure);
	
	ADC_DeInit(ADC1);  //复位ADC1 

	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;	//ADC工作模式:ADC1和ADC2工作在独立模式
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;	//模数转换工作在扫描模式
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;	//模数转换工作在连续转换模式
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//转换由软件而不是外部触发启动
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	//ADC数据右对齐
	ADC_InitStructure.ADC_NbrOfChannel = 7;	//顺序进行规则转换的ADC通道的数目
	ADC_Init(ADC1, &ADC_InitStructure);	//根据ADC_InitStruct中指定的参数初始化外设ADCx的寄存器   

	ADC_Cmd(ADC1, ENABLE);	//使能指定的ADC1
	
	ADC_ResetCalibration(ADC1);	//使能复位校准  
	 
	while(ADC_GetResetCalibrationStatus(ADC1));	//等待复位校准结束
	
	ADC_StartCalibration(ADC1);	 //开启AD校准
 
	while(ADC_GetCalibrationStatus(ADC1));	 //等待校准结束
}

void ADC_DMA_Configuration(void)
{
	DMA_InitTypeDef DMA_InitStructure;
	DMA_DeInit(DMA1_Channel1); //将DMA的通道1寄存器重设为缺省值
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&ADC1->DR; //DMA外设ADC基地址
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&AD_Value; //DMA内存基地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC; //内存作为数据传输的目标地址
	DMA_InitStructure.DMA_BufferSize = N*M; //DMA通道的DMA缓存的大小
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; //外设地址寄存器不变
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; //内存地址寄存器递增
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; //数据宽度为16位
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord; //数据宽度为16位
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular; //工作在循环缓存模式
	DMA_InitStructure.DMA_Priority = DMA_Priority_High; //DMA通道1拥有高优先级
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable; //DMA通道1没有设置为内存到内存的传输
	DMA_Init(DMA1_Channel1, &DMA_InitStructure); //初始化DMA通道1
}
//获得ADC值
//ch:通道值 
u16 Get_Adc(u8 ch)   
{
  	//设置指定ADC的规则组通道，一个序列，采样时间
	ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_239Cycles5 );	//ADC1,ADC通道,采样时间为239.5周期	  			    
  
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);		//使能指定的ADC1的软件转换启动功能	
	 
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));//等待转换结束

	return ADC_GetConversionValue(ADC1);	//返回最近一次ADC1规则组的转换结果
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
