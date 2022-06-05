#include "app.h"

extern struct system_msg sys_msg;
extern struct pid_msg pid_motspeed;
extern struct pid_msg pid_motang;

void PID_Init() 
{
	pid_motspeed.Kp = 25;
	pid_motspeed.Ki = 0;
	pid_motspeed.Kd = 0;
	
	pid_motang.Kp = 2;
	pid_motang.Ki = 0.3;
	pid_motang.Kd = 0.04;
}

int16_t PID_Cal(struct pid_msg * pid_mot,int16_t tag,int16_t act,int16_t deadup,int16_t deaddown)
{
	pid_mot->ActualSpeed=act;	//ʵ��ֵ
	pid_mot->SetSpeed = tag;	//Ŀ��ֵ
	pid_mot->Err = pid_mot->SetSpeed - pid_mot->ActualSpeed;	//�������
	
	pid_mot->output = pid_mot->output_last + \
										pid_mot->Kp*(pid_mot->Err-pid_mot->Err_last) + \
										pid_mot->Ki*pid_mot->Err + \
										pid_mot->Kd*(pid_mot->Err - 2*pid_mot->Err_last + pid_mot->Err_llast);
										//����������PID���ƵĹ�ʽ
	
	//��������������
	if(pid_mot->output>deadup)
		pid_mot->output=deadup;
	if(pid_mot->output<deaddown)
		pid_mot->output=deaddown;
	
	//����������
	if(sys_msg.mot_Ama>sys_msg.dead_A)
	{
		if(pid_mot->output > 50)
			pid_mot->output=pid_mot->output - 10*(sys_msg.mot_Ama>sys_msg.dead_A);
		else if(pid_mot->output < -50)
			pid_mot->output=pid_mot->output + 10*(sys_msg.mot_Ama>sys_msg.dead_A);
	}
		
	
	
	pid_mot->output_last=pid_mot->output;//�����ϴ����ֵ
	pid_mot->Err_llast=pid_mot->Err_last;	//�������ϴ����
	pid_mot->Err_last = pid_mot->Err;	//�����ϴ����
	
	return pid_mot->output;//PID�����
}
