#ifndef __OLED_H
#define __OLED_H

#include "stm32f4xx.h"

#include "stdlib.h" 	 
#include <string.h>
#include <stdio.h>

/**
  * OLED模式设置
	* 0:spi
	* 1:并行
	* 2:IIC
  */
#define OLED_MODE 				2

#define SIZE 							16
#define XLevelL						0x00
#define XLevelH						0x10
#define Max_Column				128
#define Max_Row						64
#define	Brightness				0xFF 
#define X_WIDTH 					128
#define Y_WIDTH 					64	    



/*并行口定义*/
#define DATAOUT(x) GPIO_Write(GPIOA,x);



/*SPI iic定义*/
#define IIC_ADS 0x78			//设备地址
#define OLED_CMD  0				//写命令
#define OLED_DATA 1				//写数据
//端口引脚宏定义
#define  GPIO_OLED_CLK  				RCC_AHB1Periph_GPIOE
#define  GPIO_OLED_X						GPIOE
#define  GPIO_OLED_SCLK_Pin     GPIO_Pin_7							/* D0 */
#define  GPIO_OLED_PIN_Pin     	GPIO_Pin_8							/* D1 */
#define  GPIO_OLED_RES_Pin     	GPIO_Pin_9							/* RES */
#define  GPIO_OLED_DC_Pin     	GPIO_Pin_10							/* DC */
//引脚电平设置
/*
注意：需要配置的有RES、DC、CLK、PIN四个引脚，接线CS可不接，当选模式0的时候要接CS
#define OLED_CS_Clr()  GPIO_ResetBits(GPIOA,GPIO_Pin_8)	//CS 片选 => 置零或悬空 当选模式0的时候要连
#define OLED_CS_Set()  GPIO_SetBits(GPIOA,GPIO_Pin_8)
*/
#define OLED_RST_Clr() GPIO_ResetBits(GPIO_OLED_X,GPIO_OLED_RES_Pin)	//RES
#define OLED_RST_Set() GPIO_SetBits(GPIO_OLED_X,GPIO_OLED_RES_Pin)
#define OLED_DC_Clr() GPIO_ResetBits(GPIO_OLED_X,GPIO_OLED_DC_Pin)	//DC
#define OLED_DC_Set() GPIO_SetBits(GPIO_OLED_X,GPIO_OLED_DC_Pin)
#define OLED_SCLK_Clr() GPIO_ResetBits(GPIO_OLED_X,GPIO_OLED_SCLK_Pin)//CLK		SCL
#define OLED_SCLK_Set() GPIO_SetBits(GPIO_OLED_X,GPIO_OLED_SCLK_Pin)
#define OLED_SDIN_Clr() GPIO_ResetBits(GPIO_OLED_X,GPIO_OLED_PIN_Pin)//PIN		SDA
#define OLED_SDIN_Set() GPIO_SetBits(GPIO_OLED_X,GPIO_OLED_PIN_Pin)


/* OLED控制用函数 */
void OLED_Clear(void);																																											 /* OLED清屏 */
void OLED_Display_On(void);                                                                                  /* OLED开 */
void OLED_ShowPosture(void);																																								 /* 提示信息 */
void OLED_Display_Off(void);                                                                                 /* OLED关 */
void GPIO_OLED_InitConfig(void);                                                                             /* OLED初始化 */
void OLED_WR_Byte(uint8_t dat,uint8_t cmd);	                                                                 /* 写字节 */
void OLED_Set_Pos(unsigned char x, unsigned char y);                                                         /* 设置坐标 */
void OLED_ShowChar(uint8_t x,uint8_t y,uint8_t chr);                                                         /* 显示字符 */
void OLED_ShowString(uint8_t x,uint8_t y, char *p);	                                                     		 /* 显示字符串 */
void OLED_ShowCHinese(uint8_t x,uint8_t y,uint8_t no);                                                       /* 显示中文 */
void OLED_ShowNum(uint8_t x,uint8_t y,u32 num,uint8_t len,uint8_t size);                                     /* 显示数字 */
void OLED_DrawBMP(unsigned char x0, unsigned char y0,unsigned char x1, unsigned char y1,unsigned char BMP[]);/* 显示图片 */
void Board_MPU_Angle_Show(void);
void Platform_MPU_Angle_Show(void);
void Lipo_Voltage_Display(void);
void DHT11_TemHum_Display(void);
void USB_ON_Dispaly(void);
void USB_OFF_Dispaly(void);

#endif  /* __OLED_H */
	 



