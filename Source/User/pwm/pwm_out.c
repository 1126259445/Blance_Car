#include"pwm_out.h"

static void TIM3_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
}

static void NVIC_Timer3_Config(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	 //配置中断优先组
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	//配置串口中断
	NVIC_InitStructure.NVIC_IRQChannel=TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=2;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
  //初始化中断
	NVIC_Init(&NVIC_InitStructure);
}

static void TIM3_Mode_Config(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	//基本定时器设置
	TIM_TimeBaseStructure.TIM_Period=7199;
	TIM_TimeBaseStructure.TIM_Prescaler=0;
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);
	
	//pwm设置
	TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_High;
	
	TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse=0;
	TIM_OC1Init(TIM3,&TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM3,TIM_OCPreload_Enable);
	
	TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse=0;
	TIM_OC2Init(TIM3,&TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM3,TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM3,ENABLE);                        //自动装载重装载寄存器
	TIM_Cmd(TIM3,ENABLE);	                                    //TIM3使能
	
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);                  //PWM波中断使能
	NVIC_Timer3_Config();                                     //定时器3中断嵌套配置
}

void TIM3_Config(void)
{
	TIM3_GPIO_Config();
	TIM3_Mode_Config();
}


