#include "app.h"

extern struct system_msg sys_msg;

struct pid {
	float SetSpeed;	//目标
	float ActualSpeed;	//实际
	float Err; 	//误差值
	float Err_last;	//上次误差值
	float Kp, Ki, Kd;	
	float Integral; //积分值
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
	pid_mot.Err = pid_mot.SetSpeed - pid_mot.ActualSpeed;	//误差;比例
	pid_mot.Integral += pid_mot.Err;	//误差和;积分
	pid_mot.ActualSpeed = pid_mot.Kp*pid_mot.Err+pid_mot.Ki*pid_mot.Integral+pid_mot.Kd*(pid_mot.Err-pid_mot.Err_last);//根据位置型PID控制的公式
	pid_mot.Err_last = pid_mot.Err;
	return pid_mot.ActualSpeed;//PID控制后的实际输出值
}

void pid_outpwm()
{
	sys_msg.mot_pwm=(int)PID_Cal(sys_msg.mot_tag_speed,sys_msg.mot_speed);
}
