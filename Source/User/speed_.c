#include "encoder.h"
#include "motor.h"
#include "FreeRTOS.h"
#include "task.h"
#include "speed.h"


/*�Խϸߵ�Ƶ�ʲɼ���ǰ�Ƕ�ֵ*/
void Encode_Task(void *pvParameter)
{
	while(1)
	{
		Get_Angle(); //��ȡ��ǰ�Ƕ�
		Blance_Car.Speed->Motor1_Speed = Read_Encoder(2);
		Blance_Car.Speed->Motor2_Speed = -Read_Encoder(4);//�����������ӵ��ٶ�
		vTaskDelay(5);
	}
}

