#include "app.h"

/*
 * 设置 IWDG 的超时时间
 * Tout = prv/40 * rlv (s)
 * rlv:预分频器值，取值范围为：0-0XFFF
 * IWDG_Config(IWDG_Prescaler_64 ,625);  // IWDG 1s 超时溢出 
 *						（64/40）*625 = 1s
 
 Tout=((4*2^prv)*rlv)/32
 */

void IWDG_Config(uint8_t prv ,uint16_t rlv)
{	
	IWDG_WriteAccessCmd( IWDG_WriteAccess_Enable );// 使能 预分频寄存器PR和重装载寄存器RLR可写（取消写保护）
	IWDG_SetPrescaler( prv );// 设置预分频器值
	IWDG_SetReload( rlv );// 设置重装载寄存器值
	IWDG_ReloadCounter();// 把重装载寄存器的值放到计数器中
	IWDG_Enable();	// 使能 IWDG
}

void IWDG_Feed(void)// 喂狗
{
	IWDG_ReloadCounter();
}

/*
if (RCC_GetFlagStatus(RCC_FLAG_IWDGRST) != RESET)//判断是否看门口复位
		{
			RCC_ClearFlag();//清除标志
		}
*/
