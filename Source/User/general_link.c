#include "stm32f10x.h"
#include "general_link.h"
#include "serial.h"
#include "string.h"

void aeelink_legacy_msg_forward(serialPort_t *s, const void *send_buf, uint16_t buf_len)
{
    serialWriteBlock(s, (uint8_t *)send_buf, buf_len);
}

/* 原生协议数据打包 */
void general_link_legacy_msg_send(serialPort_t *s, uint8_t msg_id, const void *msg_buf, uint16_t msg_len)
{
    uint8_t sendbuf[70];
	uint8_t i = 0;
	
	/* 检验变量 */
	uint8_t _checka = 0;
	uint8_t _checkb = 0;
	
	/* 添加帧头 */
	sendbuf[0] = 0x55;
	sendbuf[1] = 0xaa;
	sendbuf[2] = msg_id;
	sendbuf[3] = msg_len;
	
	/* 复制消息内容 */
	memcpy(sendbuf + 4, (const void *)msg_buf, msg_len);
	
	/* 检验计算 */
	for (i = 2; i < msg_len + 4; i++)
	{
		_checka += sendbuf[i];
		_checkb += _checka;
	}
	
	/* 添加校验 */
	sendbuf[msg_len + 4] = _checka;
	sendbuf[msg_len + 5] = _checkb;
    
    serialWriteBlock(s, sendbuf, msg_len + 6);
}

/* AEE协议解析 */
uint8_t general_link_legacy_parse_char(_uart_frame_t *uart_frame, _recv_data_stat_t *uart_status, uint8_t data)
{
	uint8_t _parse_sta = 0;
	
	/* 数据包解析 */
	switch (uart_status->parse_status)
	{
	/* 帧头检测 */
	case UART_STATE_HEADER1:
        uart_frame->header[0] = UART_HEADER1;
		uart_status->parse_status = (UART_HEADER1 == data) ? UART_STATE_HEADER2 : UART_STATE_HEADER1;
		break;

	case UART_STATE_HEADER2:
		if (UART_HEADER2 == data)
		{
            uart_frame->header[1] = UART_HEADER2;
			uart_status->parse_status = UART_STATE_CMD;
		}
		else
		{
			uart_status->parse_status = UART_STATE_HEADER1;
			uart_status->err_cnt++;
		}
		break;

	case UART_STATE_CMD:
		uart_status->checksum[0] = data;
		uart_status->checksum[1] = data;
		uart_frame->cmd = data;
		uart_status->parse_status = UART_STATE_LENGTH;
		break;

	case UART_STATE_LENGTH:
		uart_status->checksum[0] += data;
		uart_status->checksum[1] += uart_status->checksum[0];
		uart_status->cnt = 0;
		uart_frame->length = data;
    
        if (uart_frame->length > 64)
        {
            uart_status->parse_status = UART_STATE_HEADER1;
        }
        else
        {
            uart_status->parse_status = (0 == uart_frame->length) ? UART_STATE_CHECKSUM1 : UART_STATE_PAYLOAD;
        }
		break;
			
	case UART_STATE_PAYLOAD:
		uart_status->checksum[0] += data;
		uart_status->checksum[1] += uart_status->checksum[0];
		uart_frame->payload[uart_status->cnt++] = data;
	
		if (uart_status->cnt >= (uart_frame->length))
		{
			uart_status->parse_status = UART_STATE_CHECKSUM1;
		}
		break;
				
	case UART_STATE_CHECKSUM1:
		if (uart_status->checksum[0] == data)
		{
            uart_frame->payload[uart_frame->length] = data;
			uart_status->parse_status = UART_STATE_CHECKSUM2;
		}
		else
		{
			uart_status->parse_status = UART_STATE_HEADER1;
			uart_status->err_cnt++;
		}
		break;
				
	case UART_STATE_CHECKSUM2:
		uart_status->parse_status = UART_STATE_HEADER1;
	
		if (uart_status->checksum[1] == data)
		{
            uart_frame->payload[uart_frame->length + 1] = data;
			_parse_sta = 1;
            uart_status->rec_cnt++;
		}
		else
		{
			_parse_sta = 0;
			uart_status->err_cnt++;
		}
		break;
	
	default:
		 break;
	}
	
	return _parse_sta;
}

/* end of file */
