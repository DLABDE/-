#include "app.h"

static void ADC_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
		
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;	    
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ; 
	GPIO_Init(GPIOA, &GPIO_InitStructure);		
}

//static void ADC_NVIC_Config(void)
//{
//  NVIC_InitTypeDef NVIC_InitStructure;
//	//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
//  NVIC_InitStructure.NVIC_IRQChannel = ADC_IRQn;
//  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
//  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	
//  NVIC_Init(&NVIC_InitStructure);
//}

static void ADC_Mode_Config(void)
{
	ADC_InitTypeDef ADC_InitStructure;
  ADC_CommonInitTypeDef ADC_CommonInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);// 开启ADC时钟
	ADC_DeInit();	//复位
 
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;//独立模式
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;//两个采样阶段之间的延迟5个时钟
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled; //DMA失能
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;//预分频4分频。ADCCLK=PCLK2/4=84/4=21Mhz,ADC时钟最好不要超过36Mhz 
  ADC_CommonInit(&ADC_CommonInitStructure);//初始化
	
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;//12位模式
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;//非扫描模式	
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;//关闭连续转换
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;//禁止触发检测，使用软件触发
  //ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;//外部触发通道（这里使用软件触发，随意）
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//右对齐	
  ADC_InitStructure.ADC_NbrOfConversion = 1;//1个转换在规则序列中 也就是只转换规则序列1 
  ADC_Init(ADC1, &ADC_InitStructure);//ADC初始化
	
	ADC_Cmd(ADC1, ENABLE);// 使能ADC
}

u16 GetA_ADC()
{
  ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 1, ADC_SampleTime_84Cycles);// 配置 ADC 通道转换顺序为1，采样时间越高精度越高
	//ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);// ADC 转换结束产生中断
  ADC_SoftwareStartConv(ADC1);//开始adc转换(软件触发)
	while(!ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC));//等待ADC转换结束
	ADC_ClearITPendingBit(ADC1,ADC_FLAG_EOC);
	return ADC_GetConversionValue(ADC1);
}

u16 GetAng_ADC()
{
  ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 1, ADC_SampleTime_28Cycles);
  ADC_SoftwareStartConv(ADC1);
	while(!ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC));
	ADC_ClearITPendingBit(ADC1,ADC_FLAG_EOC);
	return ADC_GetConversionValue(ADC1);
}

void THE_ADC_Init()
{
	ADC_GPIO_Config();
	ADC_Mode_Config();
	//ADC_NVIC_Config();
}
