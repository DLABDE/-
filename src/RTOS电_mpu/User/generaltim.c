#include "app.h"

//通用定时器 TIMx,x[1,8]中断优先级配置
static void TIMx_NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure; 
	
    //NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);// 设置中断组为0
    NVIC_InitStructure.NVIC_IRQChannel = GENERAL_TIM_IRQn; // 设置中断来源
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;	// 设置抢占优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;// 设置子优先级	
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/*
TIM_TimeBaseInitTypeDef结构体
 *-----------------------------------------------------------------------------
 * TIM_Prescaler         都有
 * TIM_CounterMode			 TIMx,x[6,7]没有
 * TIM_Period            都有
 * TIM_ClockDivision     TIMx,x[6,7]没有
 * TIM_RepetitionCounter 仅TIMx,x[1,8]有
 *-----------------------------------------------------------------------------
*/
//TOUT=(TIM_Period+1)*(TIM_Prescaler+1)/(TIMxCLK)
//10*8400/84000000=1ms
static void TIM_Mode_Config(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	
	// 此处默认通用定时器时钟源TIMxCLK = HCLK/2=84MHz 
  RCC_APB1PeriphClockCmd(GENERAL_TIM_CLK, ENABLE);//打开定时器时钟源

  TIM_TimeBaseStructure.TIM_Prescaler = 8400-1;	//配置预分频系数，定时器频率TIMxCLK/(TIM_Prescaler+1)=10000Hz
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;// 向上计数方式
  TIM_TimeBaseStructure.TIM_Period = 20-1; //配置自动装载值，累计 TIM_Period个后产生一个更新或者中断 	
  TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;// 输入捕获
  
	TIM_TimeBaseInit(GENERAL_TIM, &TIM_TimeBaseStructure);//初始化定时器
	
	TIM_ClearFlag(GENERAL_TIM, TIM_FLAG_Update);// 清除定时器更新中断标志位
	TIM_ITConfig(GENERAL_TIM,TIM_IT_Update,ENABLE);// 开启定时器更新中断
	TIM_Cmd(GENERAL_TIM, ENABLE);	// 使能定时器
}

void TIMx_Configuration(void)
{
	TIMx_NVIC_Configuration();	
  TIM_Mode_Config();
}
