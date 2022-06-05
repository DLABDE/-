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
	int16_t mot_pwm;	//给定PWM
	int16_t mot_speed;//实际速度
	float mot_ang;	//实际角度
	u16 mot_Ama;		//实际电流
	int16_t mot_tag_speed;	//目标速度
	float mot_tag_ang;	//目标角度
	u16 dead_A;//堵转电流
};

struct system_flag
{
	u8 own_flag;
};

#endif /*_APP_H*/
