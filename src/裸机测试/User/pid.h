#ifndef _PID_H
#define _PID_H

#include "stm32f4xx.h"

void PID_Init(void);
float PID_Cal(float Speed,float actspeed);
void pid_outpwm(void);

#endif /*_PID_H*/
