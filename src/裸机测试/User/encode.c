#include "app.h"

extern struct system_msg sys_msg;

/*
编码器-->GPIOA 0,1	-->TIM2
*/
void Encoder_Init_TIM5(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  GPIO_InitTypeDef GPIO_InitStructure;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	GPIO_PinAFConfig(GPIOA,GPIO_PinSource0,GPIO_AF_TIM2); 
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource1,GPIO_AF_TIM2);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;    
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;       
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; 
  GPIO_Init(GPIOA, &GPIO_InitStructure);                       

  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  TIM_TimeBaseStructure.TIM_Prescaler = 0x0; // 预分频器
  TIM_TimeBaseStructure.TIM_Period = 1000; //设定计数器自动重装值
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//选择时钟分频：不分频
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;////TIM向上计数
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	
  TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);//使用编码器模式3
	
	TIM_ICInitTypeDef TIM_ICInitStructure;
  TIM_ICStructInit(&TIM_ICInitStructure);
  TIM_ICInitStructure.TIM_ICFilter = 0xf;
  TIM_ICInit(TIM2, &TIM_ICInitStructure);

  TIM_SetCounter(TIM2,0);
  TIM_Cmd(TIM2, ENABLE);
}


void Read_Encoder()
{
	u16 encode=0;
	for(u8 i=0;i<5;i++)
	{
		encode+=(short)TIM2 -> CNT;
	}
	sys_msg.mot_speed=encode/5;
	TIM2 -> CNT=0;
}


void config_motor()
{
	Encoder_Init_TIM5();
}
