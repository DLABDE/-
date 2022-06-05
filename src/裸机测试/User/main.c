#include "stm32f4xx.h"
#include <stdio.h>

#include "app.h"

struct system_msg sys_msg;
struct system_flag sys_flag;

__IO uint16_t ADC_Value;

void delay(uint32_t time)
{
	for(;time>0;time--);
}

void board_init()
{
	led_inint();
	usartinit();
	THEADC_Init();
	TIM14_PWM_Init(500-1,12-1);//84M/84=1Mhz的计数频率,重装载值1000，所以PWM频率为 1M/1000=1Khz.     
	TIMx_Configuration();
	config_motor();
}

int main(void)
{
	u16 led0pwmval=0;    
	u8 dir=1;
	board_init();
	while(1)
	{ 
		delay(0xffff);
		if(dir)led0pwmval++;//dir==1 led0pwmval递增
		else led0pwmval--;	//dir==0 led0pwmval递减 
 		if(led0pwmval>500-1)dir=0;//led0pwmval到达xxx后，方向为递减
		if(led0pwmval==0)dir=1;	//led0pwmval递减到0后，方向改为递增
		
		Usart_SendByte( USART1, 0xff);
		Usart_SendByte( USART1, 0x01);
		Usart_SendHalfWord( USART1, led0pwmval);
		Usart_SendByte( USART1, 0xaa);

		
		sys_msg.mot_A10ma =200*(float) ADC_Value/4096*(float)3.3; // 读取转换的AD值,*10电压（3300mv/10）*2电流(ma)
		Usart_SendByte( USART1, 0xff);
		Usart_SendByte( USART1, 0x02);
		Usart_SendHalfWord( USART1, sys_msg.mot_A10ma);
		Usart_SendByte( USART1, 0xaa);
		
		if(sys_flag.encode_flag)
		{
			sys_flag.encode_flag=0;
			Read_Encoder();
			Usart_SendByte( USART1, 0xff);
			Usart_SendByte( USART1, 0x03);
			Usart_SendHalfWord( USART1, sys_msg.mot_speed);
			Usart_SendByte( USART1, 0xaa);
		}
		
		TIM_SetCompare1(TIM14,led0pwmval);	//修改比较值，修改占空比
	}
}
