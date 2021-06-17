#ifndef __USART1_H
#define	__USART1_H

#include "stm32f10x.h"
#include <stdio.h>

#define RECIVE_BUF_SIZE 256
/* 串口定义 */
typedef struct
{
	DMA_TypeDef *DMA;
	uint32_t DMA_TCIF;
	//DMA_Stream_TypeDef *DMA_Stream;
}_comm_type_t;

/*普通串口消息接受队列定义*/
typedef struct
{
	uint8_t in;
	uint8_t out;
	uint8_t buf[RECIVE_BUF_SIZE];
}_comm_recv_queue_t;


/* 普通串口发送消息帧 */
typedef struct
{
	uint8_t index;
	uint8_t len;
	uint8_t buf[64];
} __attribute__((packed)) _uart_msg_buf_t;

/* 普通串口消息发送队列定义 */
typedef struct
{
	uint8_t in;
	uint8_t out;
	_uart_msg_buf_t msg_array[20];
} _comm_send_queue_t;


void USART1_Config(void);
void NVIC_Configuration(void);
int fputc(int ch, FILE *f);
void USART1_IRQHandler(void);

#endif /* __USART1_H */

