#include "radio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "serial.h"
#include "general_link.h"


_rc_channel_t rc;




void radio_Task(void *pvParameter)
{
	serialPort_t *radio_serial = serialOpen(RADIO_PORT, RADIO_BAUD_RATE, USART_HardwareFlowControl_None, 512, 512);
	
	 _uart_frame_t uart_frame;
	_recv_data_stat_t uart_parse_status = {0, UART_STATE_HEADER1, 0, 0, {0, 0}};
	
	while(1)
	{
		 while (serialAvailable(radio_serial))
        {
            if (general_link_legacy_parse_char(&uart_frame, &uart_parse_status, serialRead(radio_serial)))
			{
				switch (uart_frame.cmd)
				{          
				/* Ò£¿ØÊý¾Ý */
				case GENERAL_LINK_CMD_RC_DATA:
					rc.roll = uart_frame.payload[0];
					rc.pitch = uart_frame.payload[1];
					rc.throttle = uart_frame.payload[2];
					rc.yaw = uart_frame.payload[3];
					rc.mod = uart_frame.payload[4];
					general_link_legacy_msg_send(radio_serial,GENERAL_LINK_CMD_RC_DATA,NULL,0);
				break;
					
				/* ÐÄÌø */
				case GENERAL_LINK_CMD_HEARTBEAT:
					general_link_legacy_msg_send(radio_serial,GENERAL_LINK_CMD_HEARTBEAT,NULL,0);
				break;
				}
			}
		}
		
		vTaskDelay(10);
	}
}

