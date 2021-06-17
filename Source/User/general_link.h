#ifndef __GENERAL_LINK_H__
#define __GENERAL_LINK_H__
#include "stm32f10x.h"
#include "serial.h"

/* <-- GENERAL_LINK 消息定义 --> */
#define UART_PACKAGE_SIZE						64		/* 单帧最大有效数据长度 */
#define UART_HEADER1							0x55	/* 帧头1 */
#define UART_HEADER2							0xAA	/* 帧头2 */

/* 基本数据 */
#define GENERAL_LINK_CMD_RC_DATA				100 	/* 基本RC控制 */
#define	GENERAL_LINK_CMD_STATE		   			200		/* 飞行器姿态 */

#define GENERAL_LINK_CMD_HEARTBEAT              255     /* 0xff 心跳包，用于检测链路状态 */

#define LINK_STATUS_WIRED_UP                    ((uint32_t)0x00000001)
#define LINK_STATUS_WIRED_DOWN                  ((uint32_t)0x00000002)
#define LINK_STATUS_RADIO_UP                    ((uint32_t)0x00000004)
#define LINK_STATUS_RADIO_DOWN                  ((uint32_t)0x00000008)


/* 通讯协议状态转换定义 */
typedef enum
{
	UART_STATE_HEADER1,
	UART_STATE_HEADER2,
	UART_STATE_CMD,
	UART_STATE_LENGTH,
	UART_STATE_LENGTH2,
	UART_STATE_PAYLOAD,
	UART_STATE_CHECKSUM1,
	UART_STATE_CHECKSUM2
}_uart_state_t;

typedef struct
{
    uint8_t header[2];
    uint8_t cmd;
    uint8_t length;
    uint8_t payload[UART_PACKAGE_SIZE];
    uint8_t chksum[2];
} __attribute__((packed)) _uart_frame_t;

//协议解析状态
typedef struct
{
	uint8_t	cnt;
	uint8_t	parse_status;
    uint32_t rec_cnt;
	uint32_t err_cnt;
	uint8_t checksum[2];
} __attribute__((packed)) _recv_data_stat_t;

void general_link_legacy_msg_forward(serialPort_t *s, const void *send_buf, uint16_t buf_len);
void general_link_legacy_msg_send(serialPort_t *s, uint8_t msg_id, const void *msg_buf, uint16_t msg_len);
uint8_t general_link_legacy_parse_char(_uart_frame_t *uart_frame, _recv_data_stat_t *uart_status, uint8_t data);
#endif

