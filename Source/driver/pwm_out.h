#ifndef _PWM_OUT_H
#define _PWM_OUT_H

#include"stm32f10x.h"
#include"stdio.h"

static void TIM3_GPIO_Config(void);
static void TIM3_Mode_Config(void);
static void NVIC_Timer3_Config(void);
void TIM3_Config(void);

#endif
