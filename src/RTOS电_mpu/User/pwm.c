#include "app.h"


static void pwm_gpio_init()
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;       //��������ת
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;     
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;    
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;  
	GPIO_Init(GPIOA,&GPIO_InitStructure);            
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource7,GPIO_AF_TIM14); //GPIOA7����Ϊ��ʱ��14
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;        			//PWM���
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;     
	GPIO_Init(GPIOA,&GPIO_InitStructure);             
}

static void TIM14_PWM_Init()
{		 					 	
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14,ENABLE);  
	
	TIM_TimeBaseStructure.TIM_Prescaler=PSC_PWM;  //��ʱ����Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_Period=ARR_PWM;   //�Զ���װ��ֵ
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM14,&TIM_TimeBaseStructure);//��ʼ����ʱ��14
	
	//��ʼ��TIM14 Channel1 PWMģʽ	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //�������:TIM����Ƚϼ��Ը�
	TIM_OC1Init(TIM14, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM1 4OC1

	TIM_OC1PreloadConfig(TIM14, TIM_OCPreload_Enable);  //ʹ��TIM14��CCR1�ϵ�Ԥװ�ؼĴ���
 
  TIM_ARRPreloadConfig(TIM14,ENABLE);//ARPEʹ�� 
	
	TIM_Cmd(TIM14, ENABLE);  //ʹ��TIM14
}  

void set_pwm(int16_t mot_pwm)
{
	if(mot_pwm>=0)//��ת
	{
		GPIO_SetBits(GPIOA,GPIO_Pin_5);
		TIM_SetCompare1(TIM14,mot_pwm);
	}
	else	//��ת
	{
		GPIO_ResetBits(GPIOA,GPIO_Pin_5);
		TIM_SetCompare1(TIM14,(-mot_pwm));
	}
}

void PWM_Init()
{
	pwm_gpio_init();
	TIM14_PWM_Init();
}

