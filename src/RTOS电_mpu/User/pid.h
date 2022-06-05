#ifndef _PID_H
#define _PID_H

#include "stm32f4xx.h"


struct pid_msg {
	float SetSpeed;	//Ŀ��
	float ActualSpeed;	//ʵ��
	float output;//���ֵ
	
	float Err; 	//���ֵ
	float Err_last;	//�ϴ����ֵ
	float Err_llast;//���ϴ����
	float output_last; //�ϴ����ֵ
	float Kp, Ki, Kd;	
};


void PID_Init(void);

//pid�ṹ�壬Ŀ�꣬ʵ�ʣ�������ޣ��������
int16_t PID_Cal(struct pid_msg * pid_mot,int16_t tag,int16_t act,int16_t deadup,int16_t deaddown);

#endif /*_PID_H*/
