#include "app.h"

extern struct system_msg sys_msg;

struct pid {
	float SetSpeed;	//Ŀ��
	float ActualSpeed;	//ʵ��
	float Err; 	//���ֵ
	float Err_last;	//�ϴ����ֵ
	float Kp, Ki, Kd;	
	float Integral; //����ֵ
} pid_mot;

void PID_Init() 
{
	pid_mot.Kp = 2;
	pid_mot.Ki = 0.1;
	pid_mot.Kd = 0;
}

float PID_Cal(float Speed,float actspeed)
{
	pid_mot.ActualSpeed=actspeed;
	pid_mot.SetSpeed = Speed;
	pid_mot.Err = pid_mot.SetSpeed - pid_mot.ActualSpeed;	//���;����
	pid_mot.Integral += pid_mot.Err;	//����;����
	pid_mot.ActualSpeed = pid_mot.Kp*pid_mot.Err+pid_mot.Ki*pid_mot.Integral+pid_mot.Kd*(pid_mot.Err-pid_mot.Err_last);//����λ����PID���ƵĹ�ʽ
	pid_mot.Err_last = pid_mot.Err;
	return pid_mot.ActualSpeed;//PID���ƺ��ʵ�����ֵ
}

void pid_outpwm()
{
	sys_msg.mot_pwm=(int)PID_Cal(sys_msg.mot_tag_speed,sys_msg.mot_speed);
}
