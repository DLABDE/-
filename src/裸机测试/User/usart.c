#include "app.h"
#include <stdio.h>

/* 中断设置 */
static void nvicinit(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//中断源
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

/* 1,gpio初始化 */
static void gpiousart()
{
	/* 结构体初始化 */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	GPIO_InitTypeDef gpio_init;

	gpio_init.GPIO_Speed=GPIO_Fast_Speed;
	gpio_init.GPIO_PuPd=GPIO_PuPd_UP;
	gpio_init.GPIO_Mode=GPIO_Mode_AF;//复用模式
	gpio_init.GPIO_OType=GPIO_OType_PP;//推挽输出
	
	gpio_init.GPIO_Pin=GPIO_Pin_9;//TX
	GPIO_Init(GPIOA,&gpio_init);
	gpio_init.GPIO_Pin=GPIO_Pin_10;//RX
	GPIO_Init(GPIOA,&gpio_init);
	
	/* gpio复用 */
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1);
	
}

/* 2,设置串口 */
void usartinit()
{
	gpiousart();
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	//初始化结构体
  USART_InitTypeDef USART_InitStructure;
		
  USART_InitStructure.USART_BaudRate = 115200;//波特率
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长8位
  USART_InitStructure.USART_StopBits = USART_StopBits_1;//停止位1位
  USART_InitStructure.USART_Parity = USART_Parity_No;//不校验
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//不用硬件流
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;//usart模式RX+TX
  USART_Init(USART1, &USART_InitStructure); //初始化
	
	#if EN_USART1_RX	
	nvicinit();//中断配置
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//使能串口接收中断(不使用中断函数时注释之)
	//USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);//使能串口空中断（帧中断），空一个字节时间后产生中断（串口收到一帧数据后，发生的中断）（以发送完成）
	//void Uatr1_Back_IRQHandler()
	#endif
	
  USART_Cmd(USART1, ENABLE);//使能串口
}


//解析下达数据
void eat_msg(uint8_t *res,uint8_t max)
{

}


/*发送一个字符*/
void Usart_SendByte( USART_TypeDef * pUSARTx, uint8_t ch)
{
	USART_SendData(pUSARTx,ch);//串口+数据
	while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == 0);//等待发送数据寄存器为空
}

/*发送字符串*/
void Usart_string( USART_TypeDef * pUSARTx, char *str)
{
	unsigned int k=0;
  do 
  {
      Usart_SendByte( pUSARTx, *(str + k) );
      k++;
  } while(*(str + k)!='\0');
  while(USART_GetFlagStatus(pUSARTx,USART_FLAG_TC)==0);
}

/*发送一个16位数*/
void Usart_SendHalfWord( USART_TypeDef * pUSARTx, uint16_t ch)
{
	uint8_t temp_h, temp_l;
	
	/* 取出高八位 */
	temp_h = (ch&0XFF00)>>8;
	/* 取出低八位 */
	temp_l = ch&0XFF;
	
	/* 发送高八位 */
	USART_SendData(pUSARTx,temp_h);	
	while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == 0);
	
	/* 发送低八位 */
	USART_SendData(pUSARTx,temp_l);	
	while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == 0);	
}


///重定向c库函数printf到串口，重定向后可使用printf函数
int fputc(int ch, FILE *f)
{
		//发送一个字节数据到串口
		USART_SendData(USART1, (uint8_t) ch);
		
		//等待发送完毕
		while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);		
	
		return (ch);
}

///重定向c库函数scanf到串口，重写向后可使用scanf、getchar等函数
int fgetc(FILE *f)
{
		//等待串口输入数据
		while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET);

		return (int)USART_ReceiveData(USART1);
}
