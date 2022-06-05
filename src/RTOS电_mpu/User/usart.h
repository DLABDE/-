#ifndef _USART_H
#define _USART_H

#include <stdio.h>
#include "stm32f4xx.h"

#define EN_USART1_RX 	1		//ʹ��\��ֹ����1����

#define FRA_HEAD	0xa5	//֡ͷ
#define FRA_END		0xaa	//֡β


void usartinit(void);
void Usart_Dataframe(uint8_t control,uint16_t data);
void Usart_SendByte( USART_TypeDef * pUSARTx, uint8_t ch);
void Usart_SendHalfWord( USART_TypeDef * pUSARTx, uint16_t ch);
void Usart_string( USART_TypeDef * pUSARTx, char *str);

#endif /*_USART_H*/
