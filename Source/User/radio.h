#ifndef __RADIO_H__
#define __RADIO_H__
#include "stm32f10x.h"


#define RADIO_PORT           USART2
#define RADIO_BAUD_RATE      115200


/* 控制通道类型定义 */
enum
{
	ROLL = 0,
	PITCH = 1,
	GAS = 2,
	THR = 3,
	MODE = 4,
};

typedef struct 
{
	int8_t roll;
	int8_t pitch;
	int8_t throttle;
	int8_t yaw;
	uint8_t mod;
}_rc_channel_t;
extern _rc_channel_t rc;

extern void radio_Task(void *pvParameter);
#endif



