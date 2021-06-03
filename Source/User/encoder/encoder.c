#include "encoder.h"
  /**************************************************************************
作者：平衡小车之家
我的淘宝小店：http://shop114407458.taobao.com/
**************************************************************************/
/**************************************************************************
函数功能：把TIM2初始化为编码器接口模式
入口参数：无
返回  值：无
**************************************************************************/
void NVIC_Timer2_Config(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	//配置中断优先组
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	//配置串口中断
	NVIC_InitStructure.NVIC_IRQChannel=TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=3;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
  //初始化中断
	NVIC_Init(&NVIC_InitStructure);
}

void NVIC_Timer4_Config(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	//配置中断优先组
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	//配置串口中断
	NVIC_InitStructure.NVIC_IRQChannel=TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=4;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
  //初始化中断
	NVIC_Init(&NVIC_InitStructure);
}

void Encoder_Init_TIM2(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM2_ICInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2|RCC_APB2Periph_GPIOA,ENABLE);//使能定时器TIM2和GPIOA的时钟 
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0|GPIO_Pin_1;//选定PA0和PA1
	GPIO_InitStructure.GPIO_Mode= GPIO_Mode_IN_FLOATING;//浮空输入
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	//基本定时器设置
	TIM_TimeBaseStructure.TIM_Period=ENCODER_TIM_PERIOD-1;//设定计数器重装
	TIM_TimeBaseStructure.TIM_Prescaler=0;//预分频
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;//时钟分频，不分频
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;//选择计数模式，边沿对齐模式,向上计数
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);
	
	TIM2_ICInitStructure.TIM_Channel = TIM_Channel_1|TIM_Channel_2; //选择IC1映射到TI1上,IC2映射到TI2上
	TIM2_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;//映射到TI1和TI2上
	TIM2_ICInitStructure.TIM_ICFilter=0x03;//滤波1
	TIM_ICInit(TIM2, &TIM2_ICInitStructure);
	
	TIM_EncoderInterfaceConfig(TIM2,3,0,0); //SMS='011' 所有的输入均在上升沿和下降沿有效
																					//CC2P='0'	 IC2FP2不反相，IC2FP2=TI2
																					//CC1P='0'	 IC1FP1不反相，IC1FP1=TI1
	TIM_ITConfig(TIM2,TIM_IT_Update|TIM_IT_Trigger ,ENABLE);  //TIM3中断使能，允许更新中断，允许触发中断
	TIM_ARRPreloadConfig(TIM2,ENABLE);                        //自动装载重装载寄存器
	TIM_Cmd(TIM2,ENABLE);	                                    //TIM3使能
	NVIC_Timer2_Config();
}
/**************************************************************************
函数功能：把TIM4初始化为编码器接口模式
入口参数：无
返回  值：无
**************************************************************************/
void Encoder_Init_TIM4(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM4_ICInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4|RCC_APB2Periph_GPIOB,ENABLE);//使能定时器TIM2和GPIOA的时钟 
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_6|GPIO_Pin_7;//选定PA0和PA1
	GPIO_InitStructure.GPIO_Mode= GPIO_Mode_IN_FLOATING;//浮空输入
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
	//基本定时器设置
	TIM_TimeBaseStructure.TIM_Period=ENCODER_TIM_PERIOD-1;//设定计数器重装
	TIM_TimeBaseStructure.TIM_Prescaler=0;//预分频
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;//时钟分频，不分频
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;//选择计数模式，边沿对齐模式,向上计数
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseStructure);
		
	TIM4_ICInitStructure.TIM_Channel = TIM_Channel_1|TIM_Channel_2; //选择IC1映射到TI1上,IC2映射到TI2上
	TIM4_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;//映射到TI1和TI2上
	TIM4_ICInitStructure.TIM_ICFilter=0x03;//滤波1
	TIM_ICInit(TIM4, &TIM4_ICInitStructure);
	
	TIM_EncoderInterfaceConfig(TIM4,3,0,0); //SMS='011' 所有的输入均在上升沿和下降沿有效
																					//CC2P='0'	 IC2FP2不反相，IC2FP2=TI2	
	TIM_ITConfig(TIM4,TIM_IT_Update|TIM_IT_Trigger ,ENABLE);  //TIM4中断使能，允许更新中断，允许触发中断
	TIM_ARRPreloadConfig(TIM4,ENABLE);                        //自动装载重装载寄存器
	TIM_Cmd(TIM4,ENABLE);	                                    //TIM4使能
	NVIC_Timer4_Config();
}
/**************************************************************************
函数功能：单位时间读取编码器计数
入口参数：定时器
返回  值：速度值
**************************************************************************/
int Read_Encoder(u8 TIMX)
{
	int Encoder_TIM;    
	switch(TIMX)
	{
		case 2:  Encoder_TIM= (short)TIM2 -> CNT;  TIM2 -> CNT=0;break;
		case 4:  Encoder_TIM= (short)TIM4 -> CNT;  TIM4 -> CNT=0;break;	
		default:  Encoder_TIM=0;
	}
	return Encoder_TIM;
}

/**************************************************************************
函数功能：TIM4中断服务函数
入口参数：无
返回  值：无
**************************************************************************/
void TIM4_IRQHandler(void)
{ 		    		  			    
	if(TIM4->SR&0X0001)//溢出中断
	{    				   				     	    	
	}				   
	TIM4->SR&=~(1<<0);//清除中断标志位 	    
}
/**************************************************************************
函数功能：TIM2中断服务函数
入口参数：无
返回  值：无
**************************************************************************/
void TIM2_IRQHandler(void)
{ 		    		  			    
	if(TIM2->SR&0X0001)//溢出中断
	{    				   				     	    	
	}				   
	TIM2->SR&=~(1<<0);//清除中断标志位 	    
}

