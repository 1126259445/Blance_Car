#include "bsp_usart1.h"

int Flag_Qian,Flag_Hou,Flag_Left,Flag_Right;

void USART1_Config(void)
{
	//����ṹ��
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	//����GPIOA��USART1��ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA,ENABLE);
	//GPIOA�ĳ�ʼ������
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	//USART1������
	USART_InitStructure.USART_BaudRate=9600;
	USART_InitStructure.USART_WordLength=USART_WordLength_8b;
	USART_InitStructure.USART_StopBits=USART_StopBits_1;
	USART_InitStructure.USART_Parity=USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode=USART_Mode_Rx|USART_Mode_Tx;
	USART_Init(USART1,&USART_InitStructure);
	
	//ʹ�ܴ���1�����ж�
	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);
	//ʹ�ܴ���1
	USART_Cmd(USART1,ENABLE);
}
void NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	//�����ж�������
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	//���ô����ж�
	NVIC_InitStructure.NVIC_IRQChannel=USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=1;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
  //��ʼ���ж�
	NVIC_Init(&NVIC_InitStructure);
}

//���¶���C�⺯��printf��scanf��USART1
int fputc(int ch, FILE *f)
{
	//����һ���ֽڵ�USART1
	USART_SendData(USART1,(uint8_t)ch);
  //�ȴ��������
	while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET);
	return ch;
}

void USART1_IRQHandler(void)
{
	uint8_t uart_receive;
	if(USART_GetITStatus(USART1,USART_IT_RXNE)!=RESET)
	{
    uart_receive=USART_ReceiveData(USART1);
	if(uart_receive>10)  //Ĭ��ʹ��appΪ��MiniBalanceV3.5 ��ΪMiniBalanceV3.5��ң��ָ��ΪA~H ��HEX������10
	{			
		if(uart_receive==0x5A)	Flag_Qian=0,Flag_Hou=0,Flag_Left=0,Flag_Right=0;//////////////ɲ��
		else if(uart_receive==0x41)	Flag_Qian=1,Flag_Hou=0,Flag_Left=0,Flag_Right=0;//////////////ǰ
		else if(uart_receive==0x45)	Flag_Qian=0,Flag_Hou=1,Flag_Left=0,Flag_Right=0;//////////////��
		else if(uart_receive==0x42||uart_receive==0x43||uart_receive==0x44)	
													Flag_Qian=0,Flag_Hou=0,Flag_Left=0,Flag_Right=1;  //��
		else if(uart_receive==0x46||uart_receive==0x47||uart_receive==0x48)	    //��
													Flag_Qian=0,Flag_Hou=0,Flag_Left=1,Flag_Right=0;
		else Flag_Qian=0,Flag_Hou=0,Flag_Left=0,Flag_Right=0;//////////////ɲ��
	}
  }
}
