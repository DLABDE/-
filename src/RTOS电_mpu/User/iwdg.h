#ifndef _IWDG_H_
#define _IWDG_H_

#include "stm32f4xx.h"

void IWDG_Config(uint8_t prv ,uint16_t rlv);
void IWDG_Feed(void);

#endif /* _IWDG_H_ */
