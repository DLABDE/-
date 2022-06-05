/**
  ******************************************************************************
  * @file    FMC_SDRAM/stm32f4xx_it.c 
  * @author  MCD Application Team
  * @version V1.0.1
  * @date    11-November-2013
  * @brief   Main Interrupt Service Routines.
  *         This file provides template for all exceptions handler and
  *         peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"

/** @addtogroup STM32F429I_DISCOVERY_Examples
  * @{
  */

/** @addtogroup FMC_SDRAM
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {}
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {}
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {}
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {}
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
//void SVC_Handler(void)
//{}

/**
  * @brief  This function handles PendSV_Handler exception.
  * @param  None
  * @retval None
  */
//void PendSV_Handler(void)
//{}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
//void SysTick_Handler(void)
//{}
	
	
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"

#include "app.h"
/**
  * @brief  This function handles TIM interrupt request.
  * @param  None
  * @retval None
 */	
 
//定时器中断
extern struct system_flag sys_flag;
void  GENERAL_TIM_IRQHandler (void)
{
	uint32_t ulReturn;
  ulReturn = taskENTER_CRITICAL_FROM_ISR();//进入临界段，临界段可嵌套
	if ( TIM_GetITStatus( GENERAL_TIM, TIM_IT_Update) != RESET ) 
	{	
		TIM_ClearITPendingBit(GENERAL_TIM , TIM_IT_Update);  
		Read_Encoder();//读取电机速度
	}		 	
	taskEXIT_CRITICAL_FROM_ISR(ulReturn);
}


//ADC采集中断
//__IO uint16_t ADC_Value;
//extern __IO uint16_t ADC_Value;
//void ADC_IRQHandler(void)
//{
//	if(ADC_GetITStatus(ADC1,ADC_IT_EOC)==SET)
//	{
//		ADC_Value = ADC_GetConversionValue(ADC1);
//	}
//	ADC_ClearITPendingBit(ADC1,ADC_IT_EOC);
//}


//串口1中断
void eat_msg(uint8_t *res,uint8_t max);
#define Max_BUFF_Len 5
uint8_t Uart2_Buffer[Max_BUFF_Len];
uint8_t Uart2_Rx=0;

void USART1_IRQHandler(void)
{
	uint32_t ulReturn;
  ulReturn = taskENTER_CRITICAL_FROM_ISR();//进入临界段，临界段可嵌套
	
	//USART_GetITStatus(USART2, USART_IT_IDLE) != RESET//查询是否空中断
    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)//查询是否接收中断
    {  
			Uart2_Buffer[Uart2_Rx] = USART_ReceiveData(USART1);     //接收串口数据到buff缓冲区
      Uart2_Rx++;
			
			if(Uart2_Buffer[Uart2_Rx-1] == FRA_END)    //接收到尾标识
        {
            if(Uart2_Buffer[0] == FRA_HEAD)                      //检测到头标识
            {
                //printf("%s",Uart2_Buffer);  							
								eat_msg(Uart2_Buffer,Uart2_Rx);//处理之
                Uart2_Rx=0;                                   
            } 
            else
            {
                Uart2_Rx=0;     
            }
        }
				else if(Uart2_Rx == Max_BUFF_Len)//超出最大限额
				{
					Uart2_Rx=0;
				}
		}
		taskEXIT_CRITICAL_FROM_ISR(ulReturn);  
}

/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f429_439xx.s).                         */
/******************************************************************************/

/**
  * @}
  */ 

/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
