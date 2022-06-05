#include "app.h"

static void ADC_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
		
	// 配置 IO
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;	    
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ; 
	GPIO_Init(GPIOA, &GPIO_InitStructure);		
}

static void ADC_Mode_Config(void)
{
	ADC_InitTypeDef ADC_InitStructure;
  ADC_CommonInitTypeDef ADC_CommonInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);// 开启ADC时钟

  // ADC Common 结构体 参数 初始化
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;// 独立ADC模式
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;// 时钟为fpclk x分频	
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;// 禁止DMA直接访问模式	
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_20Cycles;// 采样时间间隔	
  ADC_CommonInit(&ADC_CommonInitStructure);
	
  //ADC Init 结构体 参数 初始化
	ADC_StructInit(&ADC_InitStructure);
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;// ADC 分辨率
  ADC_InitStructure.ADC_ScanConvMode = DISABLE; // 禁止扫描模式，多通道采集才需要	
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE; // 连续转换	
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;//禁止外部边沿触发
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;//外部触发通道（这里使用软件触发，随意）
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//数据右对齐	
  ADC_InitStructure.ADC_NbrOfConversion = 1;//转换通道 1个                        
  ADC_Init(ADC1, &ADC_InitStructure);
 
  // 配置 ADC 通道转换顺序为1，采样时间为3个时钟周期
  ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 1, ADC_SampleTime_56Cycles);
	ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);// ADC 转换结束产生中断
  ADC_Cmd(ADC1, ENABLE);// 使能ADC
  ADC_SoftwareStartConv(ADC1);//开始adc转换，软件触发
}

static void ADC_NVIC_Config(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	
  NVIC_InitStructure.NVIC_IRQChannel = ADC_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	
  NVIC_Init(&NVIC_InitStructure);
}

void THEADC_Init(void)
{
	ADC_GPIO_Config();
	ADC_Mode_Config();
	ADC_NVIC_Config();
}
