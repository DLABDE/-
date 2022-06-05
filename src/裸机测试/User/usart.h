#ifndef _USART_H
#define _USART_H

#include <stdio.h>
#include "stm32f4xx.h"

#define EN_USART1_RX 	1		//使能/禁止串口1接收

void usartinit(void);
void Usart_SendByte( USART_TypeDef * pUSARTx, uint8_t ch);
void Usart_SendHalfWord( USART_TypeDef * pUSARTx, uint16_t ch);
void Usart_string( USART_TypeDef * pUSARTx, char *str);

#endif /*_USART_H*/
