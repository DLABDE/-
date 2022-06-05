#ifndef _APP_H
#define _APP_H

#include "stm32f4xx.h"
#include <stdio.h>

#include "generaltim.h"
#include "usart.h"
#include "adc.h"
#include "pwm.h"
#include "led.h"
#include "encode.h"
#include "pid.h"

struct system_msg
{
	u16 mot_pwm;
	u16 mot_speed;
	u16 mot_A10ma;
	u16 mot_tag_speed;
};

struct system_flag
{
	u8 encode_flag;
};

#endif /*_APP_H*/
