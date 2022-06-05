#include "app.h"

void led_inint(void)//初始化
{
	//1、打开外设时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);//打开ANB1总线外设时钟使能
	//2、外设初始化结构体
	GPIO_InitTypeDef gpio_init;
	//3、配置结构体成员
	gpio_init.GPIO_Pin=GPIO_Pin_6;//6、7口
	//gpio_init.GPIO_Pin=GPIO_Pin_6|GPIO_Pin_7;//6、7口
	gpio_init.GPIO_Speed=GPIO_Low_Speed;//低速
	gpio_init.GPIO_PuPd=GPIO_PuPd_NOPULL;
	gpio_init.GPIO_Mode=GPIO_Mode_OUT;//输出模式
	gpio_init.GPIO_OType=GPIO_OType_PP;//推挽输出
	//4、调用初始化函数，将成员参数写入寄存器
	GPIO_Init(GPIOA,&gpio_init);
}
