#ifndef __I2C_H
#define __I2C_H

#include "stm32f4xx.h"

//�˿����ź궨��
#define  GPIO_CLK  				RCC_AHB1Periph_GPIOE
#define  GPIO_X						GPIOE
#define  GPIO_SCLK_Pin    GPIO_Pin_11							/* D0 */
#define  GPIO_SDA_Pin     GPIO_Pin_12							/* D1 */

//IO��������
#define SCL_Clr() GPIO_ResetBits(GPIO_X,GPIO_SCLK_Pin)	//RES
#define SCL_Set() GPIO_SetBits(GPIO_X,GPIO_SCLK_Pin)
#define SDA_Clr() GPIO_ResetBits(GPIO_X,GPIO_SDA_Pin)	//DC
#define SDA_Set() GPIO_SetBits(GPIO_X,GPIO_SDA_Pin)
#define READ_SDA  GPIO_ReadInputDataBit(GPIO_X,GPIO_SDA_Pin)


void IIC_Init(void);                //��ʼ��IIC��IO��				 
void IIC_Start(void);				//����IIC��ʼ�ź�
void IIC_Stop(void);	  			//����IICֹͣ�ź�
void IIC_Send_Byte(u8 txd);			//IIC����һ���ֽ�
u8 IIC_Read_Byte(unsigned char ack);//IIC��ȡһ���ֽ�
u8 IIC_Wait_Ack(void); 				//IIC�ȴ�ACK�ź�
void IIC_Ack(void);					//IIC����ACK�ź�
void IIC_NAck(void);				//IIC������ACK�ź�

void IIC_Write_One_Byte(u8 daddr,u8 addr,u8 data);
u8 IIC_Read_One_Byte(u8 daddr,u8 addr);	  

#endif /* __I2C_H */
