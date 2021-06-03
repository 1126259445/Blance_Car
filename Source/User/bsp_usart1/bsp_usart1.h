#ifndef __USART1_H
#define	__USART1_H

#include "stm32f10x.h"
#include <stdio.h>

void USART1_Config(void);
void NVIC_Configuration(void);
int fputc(int ch, FILE *f);
void USART1_IRQHandler(void);

#endif /* __USART1_H */
