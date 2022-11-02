#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"

#include "app.h"
//#include "bmp.h"


#define DEAD_PWM		(1000)//PWM输出限制
#define DEAD_SPEED	(80)	//速度限制
#define DE_ANG_DOWN	(0)		//角度限制
#define DE_ANG_UP		(235)


struct system_flag sys_flag;//系统flag
struct system_msg sys_msg;//电机参数
struct pid_msg pid_motspeed;//速度PID(内环)
struct pid_msg pid_motang;//角度PID(外环)

//任务句柄
static TaskHandle_t AppTaskCreate_Handle = NULL;
static TaskHandle_t Ang_Task_Handle = NULL;
static TaskHandle_t SpeedPID_Task_Handle = NULL;
static TaskHandle_t AngPID_Task_Handle = NULL;
static TaskHandle_t Iwdg_Task_Handle = NULL;
static TaskHandle_t Usarttx_Task_Handle = NULL;

//函数申明
static void AppTaskCreate(void);
static void Ang_Task(void* parameter);
static void SpeedPID_Task(void* parameter);
static void AngPID_Task(void* parameter);
static void Iwdg_Task(void* pvParameters);
static void Usarttx_Task(void* pvParameters);
static void BSP_Init(void);


static void Ang_Task(void* parameter)
{
	sys_flag.own_flag=1;
	
	//char Bufferr[15];
	float ang;
	float pitch,roll,yaw; 		//欧拉角
	short aacx,aacy,aacz;		//加速度传感器原始数据
	short gyrox,gyroy,gyroz;	//陀螺仪原始数据
	//short temp;					//温度
	
	MPU_Init();	//调用了系统延时，必须在任务中进行
	while(mpu_dmp_init());
	//OLED_Clear();
	//OLED_ShowString(0,0,"mpu ok");
	
	//TickType_t xLastWakeTime;
	while (1)
	{
		//vTaskDelayUntil(&xLastWakeTime,50);
		vTaskDelay(2);
		if(sys_flag.own_flag==1)
		{
			if(mpu_dmp_get_data(&pitch,&roll,&yaw)==0)
			{
				//temp=MPU_Get_Temperature();	//得到温度值
				MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//得到加速度传感器数据
				MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//得到陀螺仪数据
				ang=roll+(float)DE_ANG_UP/2;
				if (ang<DE_ANG_DOWN)	ang=DE_ANG_DOWN;
				if (ang>DE_ANG_UP)		ang=DE_ANG_UP;
				sys_msg.mot_tag_ang=ang;
				//OLED_ShowString(0,4,"roll:");
				//sprintf(Bufferr, "%0.2f", roll);
				//OLED_ShowString(40,4, Bufferr);
			}
		}
	}
}

static void AngPID_Task(void* parameter)
{
	TickType_t xLastWakeTime;
	sys_msg.mot_tag_ang=(int)DE_ANG_UP/2;
	while (1)
	{
		vTaskDelayUntil(&xLastWakeTime,5);
		sys_msg.mot_ang=((double) GetAng_ADC()/(double)4096*270)-30;//读取角度
		sys_msg.mot_tag_speed=PID_Cal(&pid_motang,sys_msg.mot_tag_ang,sys_msg.mot_ang,DEAD_SPEED,(-DEAD_SPEED));//PID角度
	}
}


static void SpeedPID_Task(void* parameter)
{	   
	TickType_t xLastWakeTime;
	while (1)
	{
		vTaskDelayUntil(&xLastWakeTime,1);
		sys_msg.mot_Ama =(double)2000/7.8*((double) GetA_ADC()/(double)4096*3.3); // 读取电流MA
		sys_msg.mot_pwm=PID_Cal(&pid_motspeed,sys_msg.mot_tag_speed,sys_msg.mot_speed,DEAD_PWM,(-DEAD_PWM));//PID速度
		set_pwm(sys_msg.mot_pwm);	//修改占空比
	}
}

static void Iwdg_Task(void* parameter)
{	
	while (1)
	{
		vTaskDelay(500);
		GPIO_ToggleBits(GPIOA,GPIO_Pin_6);
		IWDG_Feed();
	}
}

static void Usarttx_Task(void* parameter)
{	
	while (1)
	{
		vTaskDelay(8);
		Usart_Dataframe( 0x01, sys_msg.mot_pwm);
		Usart_Dataframe( 0x02, sys_msg.mot_Ama);
		Usart_Dataframe( 0x03, sys_msg.mot_speed);
		Usart_Dataframe( 0x04, sys_msg.mot_tag_speed);
		Usart_Dataframe( 0x05, sys_msg.mot_ang);
		Usart_Dataframe( 0x06, sys_msg.mot_tag_ang);
	}
}

//解析下达数据
void eat_msg(uint8_t *res,uint8_t max)
{
	if(max>=5)
	{
		int16_t data = (*((u16 *)(res+2)));
		
		if(res[1]==0x01)//设置目标角度
		{
			if(data<DE_ANG_DOWN)	data=DE_ANG_DOWN;
			if(data>DE_ANG_UP)		data=DE_ANG_UP;
			sys_flag.own_flag=0;
			sys_msg.mot_tag_ang=data;
		}
	}
}

static void AppTaskCreate(void)
{
  taskENTER_CRITICAL();      //进入临界区
	
	xTaskCreate((TaskFunction_t )SpeedPID_Task, 
                        (const char*    )"SpeedPID_Task",
                        (uint16_t       )512,  
                        (void*          )NULL,	
                        (UBaseType_t    )4,	   
                        (TaskHandle_t*  )&SpeedPID_Task_Handle);
												
	xTaskCreate((TaskFunction_t )AngPID_Task, 
                        (const char*    )"AngPID_Task",
                        (uint16_t       )512,  
                        (void*          )NULL,	
                        (UBaseType_t    )4,	   
                        (TaskHandle_t*  )&AngPID_Task_Handle);
	xTaskCreate((TaskFunction_t )Ang_Task, 
                        (const char*    )"Ang_Task",
                        (uint16_t       )512,  
                        (void*          )NULL,	
                        (UBaseType_t    )3,	   
                        (TaskHandle_t*  )&Ang_Task_Handle);

	xTaskCreate((TaskFunction_t )Iwdg_Task, 
                        (const char*    )"Iwdg_Task",
                        (uint16_t       )512,   //任务栈大小
                        (void*          )NULL,	//入口函数参数
                        (UBaseType_t    )2,	    //优先级
                        (TaskHandle_t*  )&Iwdg_Task_Handle);
												
	xTaskCreate((TaskFunction_t )Usarttx_Task, 
                        (const char*    )"Usarttx_Task",
                        (uint16_t       )512,  
                        (void*          )NULL,	
                        (UBaseType_t    )3,	   
                        (TaskHandle_t*  )&Usarttx_Task_Handle);
																										
	
  vTaskDelete(AppTaskCreate_Handle); //删除AppTaskCreate任务
  
  taskEXIT_CRITICAL();            //退出临界区
}

//板卡初始化
static void BSP_Init(void)
{
	//中断优先级分组为4(4bit都用来表示抢占优先级,范围:0~15),不要再次分组!!
	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );
	
	led_inint();
	GPIO_OLED_InitConfig();
	usartinit();
	THE_ADC_Init();
	PWM_Init();  
	TIMx_Configuration();
	config_motor();
	PID_Init();
	IWDG_Config(IWDG_Prescaler_64 ,625);//看门狗1s
	sys_msg.dead_A=700;
	
	printf("This is STM32F4 and FreeRTOS\r\n");
	OLED_Clear();
	OLED_ShowString(0,0,"This is RTOS");
}

int main(void)
{	
  BSP_Init();
	
	xTaskCreate((TaskFunction_t )AppTaskCreate,  /* 任务入口函数 */
                        (const char*    )"AppTaskCreate",/* 任务名字 */
                        (uint16_t       )512,  /* 任务栈大小 */
                        (void*          )NULL,/* 任务入口函数参数 */
                        (UBaseType_t    )1, /* 任务的优先级 */
                        (TaskHandle_t*  )&AppTaskCreate_Handle);/* 任务控制块指针 */ 
  vTaskStartScheduler();
  while(1);   
}
