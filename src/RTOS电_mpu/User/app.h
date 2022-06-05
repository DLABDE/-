#ifndef _APP_H
#define _APP_H

#include "stm32f4xx.h"
#include <stdio.h>

#include "iwdg.h"
#include "generaltim.h"
#include "usart.h"
#include "adc.h"
#include "pwm.h"
#include "led.h"
#include "encode.h"
#include "pid.h"
#include "oled.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 

struct system_msg
{
	int16_t mot_pwm;	//����PWM
	int16_t mot_speed;//ʵ���ٶ�
	float mot_ang;	//ʵ�ʽǶ�
	u16 mot_Ama;		//ʵ�ʵ���
	int16_t mot_tag_speed;	//Ŀ���ٶ�
	float mot_tag_ang;	//Ŀ��Ƕ�
	u16 dead_A;//��ת����
};

struct system_flag
{
	u8 own_flag;
};

#endif /*_APP_H*/
