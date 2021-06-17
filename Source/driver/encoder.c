#include "encoder.h"
  /**************************************************************************
���ߣ�ƽ��С��֮��
�ҵ��Ա�С�꣺http://shop114407458.taobao.com/
**************************************************************************/
/**************************************************************************
�������ܣ���TIM2��ʼ��Ϊ�������ӿ�ģʽ
��ڲ�������
����  ֵ����
**************************************************************************/
void NVIC_Timer2_Config(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	//�����ж�������
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	//���ô����ж�
	NVIC_InitStructure.NVIC_IRQChannel=TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=3;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
  //��ʼ���ж�
	NVIC_Init(&NVIC_InitStructure);
}

void NVIC_Timer4_Config(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	//�����ж�������
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	//���ô����ж�
	NVIC_InitStructure.NVIC_IRQChannel=TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=4;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
  //��ʼ���ж�
	NVIC_Init(&NVIC_InitStructure);
}

void Encoder_Init_TIM2(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM2_ICInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2|RCC_APB2Periph_GPIOA,ENABLE);//ʹ�ܶ�ʱ��TIM2��GPIOA��ʱ�� 
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0|GPIO_Pin_1;//ѡ��PA0��PA1
	GPIO_InitStructure.GPIO_Mode= GPIO_Mode_IN_FLOATING;//��������
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	//������ʱ������
	TIM_TimeBaseStructure.TIM_Period=ENCODER_TIM_PERIOD-1;//�趨��������װ
	TIM_TimeBaseStructure.TIM_Prescaler=0;//Ԥ��Ƶ
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;//ʱ�ӷ�Ƶ������Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;//ѡ�����ģʽ�����ض���ģʽ,���ϼ���
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);
	
	TIM2_ICInitStructure.TIM_Channel = TIM_Channel_1|TIM_Channel_2; //ѡ��IC1ӳ�䵽TI1��,IC2ӳ�䵽TI2��
	TIM2_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;//ӳ�䵽TI1��TI2��
	TIM2_ICInitStructure.TIM_ICFilter=0x03;//�˲�1
	TIM_ICInit(TIM2, &TIM2_ICInitStructure);
	
	TIM_EncoderInterfaceConfig(TIM2,3,0,0); //SMS='011' ���е�������������غ��½�����Ч
																					//CC2P='0'	 IC2FP2�����࣬IC2FP2=TI2
																					//CC1P='0'	 IC1FP1�����࣬IC1FP1=TI1
	TIM_ITConfig(TIM2,TIM_IT_Update|TIM_IT_Trigger ,ENABLE);  //TIM3�ж�ʹ�ܣ���������жϣ��������ж�
	TIM_ARRPreloadConfig(TIM2,ENABLE);                        //�Զ�װ����װ�ؼĴ���
	TIM_Cmd(TIM2,ENABLE);	                                    //TIM3ʹ��
	NVIC_Timer2_Config();
}
/**************************************************************************
�������ܣ���TIM4��ʼ��Ϊ�������ӿ�ģʽ
��ڲ�������
����  ֵ����
**************************************************************************/
void Encoder_Init_TIM4(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM4_ICInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4|RCC_APB2Periph_GPIOB,ENABLE);//ʹ�ܶ�ʱ��TIM2��GPIOA��ʱ�� 
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_6|GPIO_Pin_7;//ѡ��PA0��PA1
	GPIO_InitStructure.GPIO_Mode= GPIO_Mode_IN_FLOATING;//��������
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
	//������ʱ������
	TIM_TimeBaseStructure.TIM_Period=ENCODER_TIM_PERIOD-1;//�趨��������װ
	TIM_TimeBaseStructure.TIM_Prescaler=0;//Ԥ��Ƶ
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;//ʱ�ӷ�Ƶ������Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;//ѡ�����ģʽ�����ض���ģʽ,���ϼ���
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseStructure);
		
	TIM4_ICInitStructure.TIM_Channel = TIM_Channel_1|TIM_Channel_2; //ѡ��IC1ӳ�䵽TI1��,IC2ӳ�䵽TI2��
	TIM4_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;//ӳ�䵽TI1��TI2��
	TIM4_ICInitStructure.TIM_ICFilter=0x03;//�˲�1
	TIM_ICInit(TIM4, &TIM4_ICInitStructure);
	
	TIM_EncoderInterfaceConfig(TIM4,3,0,0); //SMS='011' ���е�������������غ��½�����Ч
																					//CC2P='0'	 IC2FP2�����࣬IC2FP2=TI2	
	TIM_ITConfig(TIM4,TIM_IT_Update|TIM_IT_Trigger ,ENABLE);  //TIM4�ж�ʹ�ܣ���������жϣ��������ж�
	TIM_ARRPreloadConfig(TIM4,ENABLE);                        //�Զ�װ����װ�ؼĴ���
	TIM_Cmd(TIM4,ENABLE);	                                    //TIM4ʹ��
	NVIC_Timer4_Config();
}
/**************************************************************************
�������ܣ���λʱ���ȡ����������
��ڲ�������ʱ��
����  ֵ���ٶ�ֵ
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
�������ܣ�TIM4�жϷ�����
��ڲ�������
����  ֵ����
**************************************************************************/
void TIM4_IRQHandler(void)
{ 		    		  			    
	if(TIM4->SR&0X0001)//����ж�
	{    				   				     	    	
	}				   
	TIM4->SR&=~(1<<0);//����жϱ�־λ 	    
}
/**************************************************************************
�������ܣ�TIM2�жϷ�����
��ڲ�������
����  ֵ����
**************************************************************************/
void TIM2_IRQHandler(void)
{ 		    		  			    
	if(TIM2->SR&0X0001)//����ж�
	{    				   				     	    	
	}				   
	TIM2->SR&=~(1<<0);//����жϱ�־λ 	    
}

