#ifndef _PWM_H_
#define _PWM_H_

#include "stm32f4xx.h"

#define ARR_PWM	(1000-1)	//�Զ���װֵ
#define PSC_PWM	(12-1)		//ʱ��Ԥ��Ƶ��

void set_pwm(int16_t mot_pwm);
void PWM_Init(void);//1000-1,84-1;  84M/84=1Mhz����Ƶ��,��װ��ֵ1000; PWMƵ��Ϊ 1M/1000=1Khz.   


#endif /*_PWM_H_*/
