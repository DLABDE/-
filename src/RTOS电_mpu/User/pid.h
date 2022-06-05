#ifndef _PID_H
#define _PID_H

#include "stm32f4xx.h"


struct pid_msg {
	float SetSpeed;	//目标
	float ActualSpeed;	//实际
	float output;//输出值
	
	float Err; 	//误差值
	float Err_last;	//上次误差值
	float Err_llast;//上上次误差
	float output_last; //上次输出值
	float Kp, Ki, Kd;	
};


void PID_Init(void);

//pid结构体，目标，实际，输出上限，输出下限
int16_t PID_Cal(struct pid_msg * pid_mot,int16_t tag,int16_t act,int16_t deadup,int16_t deaddown);

#endif /*_PID_H*/
