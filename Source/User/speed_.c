#include "encoder.h"
#include "motor.h"
#include "FreeRTOS.h"
#include "task.h"
#include "speed.h"


/*以较高的频率采集当前角度值*/
void Encode_Task(void *pvParameter)
{
	while(1)
	{
		Get_Angle(); //读取当前角度
		Blance_Car.Speed->Motor1_Speed = Read_Encoder(2);
		Blance_Car.Speed->Motor2_Speed = -Read_Encoder(4);//读出两个轮子的速度
		vTaskDelay(5);
	}
}

