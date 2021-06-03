#include "bsp_usart1.h"

int Flag_Qian,Flag_Hou,Flag_Left,Flag_Right;

void USART1_Config(void)
{
	//构造结构体
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	//开启GPIOA和USART1的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA,ENABLE);
	//GPIOA的初始化设置
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	//USART1的设置
	USART_InitStructure.USART_BaudRate=9600;
	USART_InitStructure.USART_WordLength=USART_WordLength_8b;
	USART_InitStructure.USART_StopBits=USART_StopBits_1;
	USART_InitStructure.USART_Parity=USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode=USART_Mode_Rx|USART_Mode_Tx;
	USART_Init(USART1,&USART_InitStructure);
	
	//使能串口1接收中断
	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);
	//使能串口1
	USART_Cmd(USART1,ENABLE);
}
void NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	//配置中断优先组
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	//配置串口中断
	NVIC_InitStructure.NVIC_IRQChannel=USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=1;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
  //初始化中断
	NVIC_Init(&NVIC_InitStructure);
}

//重新定向C库函数printf和scanf到USART1
int fputc(int ch, FILE *f)
{
	//发送一个字节到USART1
	USART_SendData(USART1,(uint8_t)ch);
  //等待发送完毕
	while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET);
	return ch;
}

void USART1_IRQHandler(void)
{
	uint8_t uart_receive;
	if(USART_GetITStatus(USART1,USART_IT_RXNE)!=RESET)
	{
    uart_receive=USART_ReceiveData(USART1);
	if(uart_receive>10)  //默认使用app为：MiniBalanceV3.5 因为MiniBalanceV3.5的遥控指令为A~H 其HEX都大于10
	{			
		if(uart_receive==0x5A)	Flag_Qian=0,Flag_Hou=0,Flag_Left=0,Flag_Right=0;//////////////刹车
		else if(uart_receive==0x41)	Flag_Qian=1,Flag_Hou=0,Flag_Left=0,Flag_Right=0;//////////////前
		else if(uart_receive==0x45)	Flag_Qian=0,Flag_Hou=1,Flag_Left=0,Flag_Right=0;//////////////后
		else if(uart_receive==0x42||uart_receive==0x43||uart_receive==0x44)	
													Flag_Qian=0,Flag_Hou=0,Flag_Left=0,Flag_Right=1;  //左
		else if(uart_receive==0x46||uart_receive==0x47||uart_receive==0x48)	    //右
													Flag_Qian=0,Flag_Hou=0,Flag_Left=1,Flag_Right=0;
		else Flag_Qian=0,Flag_Hou=0,Flag_Left=0,Flag_Right=0;//////////////刹车
	}
  }
}
