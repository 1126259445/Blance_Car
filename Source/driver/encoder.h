#ifndef __ENCODER_H
#define __ENCODER_H
#include <stm32f10x.h>	 
  /**************************************************************************
作者：平衡小车之家
我的淘宝小店：http://shop114407458.taobao.com/
**************************************************************************/
#define ENCODER_TIM_PERIOD (u16)(65535)   //不可大于65535 因为F103的定时器是16位的。
void Encoder_Init_TIM2(void);
void Encoder_Init_TIM4(void);
void NVIC_Timer2_Config(void);
void NVIC_Timer4_Config(void);
int Read_Encoder(u8 TIMX);
void TIM2_IRQHandler(void);
void TIM4_IRQHandler(void);
#endif
