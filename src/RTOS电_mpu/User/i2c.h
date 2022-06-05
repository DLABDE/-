#ifndef __I2C_H
#define __I2C_H

#include "stm32f4xx.h"

//端口引脚宏定义
#define  GPIO_CLK  				RCC_AHB1Periph_GPIOE
#define  GPIO_X						GPIOE
#define  GPIO_SCLK_Pin    GPIO_Pin_11							/* D0 */
#define  GPIO_SDA_Pin     GPIO_Pin_12							/* D1 */

//IO操作函数
#define SCL_Clr() GPIO_ResetBits(GPIO_X,GPIO_SCLK_Pin)	//RES
#define SCL_Set() GPIO_SetBits(GPIO_X,GPIO_SCLK_Pin)
#define SDA_Clr() GPIO_ResetBits(GPIO_X,GPIO_SDA_Pin)	//DC
#define SDA_Set() GPIO_SetBits(GPIO_X,GPIO_SDA_Pin)
#define READ_SDA  GPIO_ReadInputDataBit(GPIO_X,GPIO_SDA_Pin)


void IIC_Init(void);                //初始化IIC的IO口				 
void IIC_Start(void);				//发送IIC开始信号
void IIC_Stop(void);	  			//发送IIC停止信号
void IIC_Send_Byte(u8 txd);			//IIC发送一个字节
u8 IIC_Read_Byte(unsigned char ack);//IIC读取一个字节
u8 IIC_Wait_Ack(void); 				//IIC等待ACK信号
void IIC_Ack(void);					//IIC发送ACK信号
void IIC_NAck(void);				//IIC不发送ACK信号

void IIC_Write_One_Byte(u8 daddr,u8 addr,u8 data);
u8 IIC_Read_One_Byte(u8 daddr,u8 addr);	  

#endif /* __I2C_H */
