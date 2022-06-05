#ifndef _PWM_H_
#define _PWM_H_

#include "stm32f4xx.h"

#define ARR_PWM	(1000-1)	//自动重装值
#define PSC_PWM	(12-1)		//时钟预分频数

void set_pwm(int16_t mot_pwm);
void PWM_Init(void);//1000-1,84-1;  84M/84=1Mhz计数频率,重装载值1000; PWM频率为 1M/1000=1Khz.   


#endif /*_PWM_H_*/
