#include "pid_countrol.h"
#include "motor.h"
#include "FreeRTOS.h"
#include "task.h"

/*PID��������*/
void PID_control_Task(void *pvParameter)
{
	while(1)
	{		
		/*����Ϳ��ƻ���PWMֵ*/
		Blance_Car.Pwm->Balance_Pwm = Balance_PD();//ƽ��PID����
		Blance_Car.Pwm->Speed_Pwm = Speed_PI(Blance_Car.Speed->Motor1_Speed,Blance_Car.Speed->Motor2_Speed);//�ٶȻ�PID����
		Blance_Car.Pwm->Turn_Pwm = Balance_Turn(Blance_Car.Speed->Motor1_Speed,Blance_Car.Speed->Motor2_Speed,Blance_Car.Sensor->Turn_Gyro);//ת�򻷿���		
		/*�������յ�PWM���ֵ*/
		Blance_Car.Pwm->Final_Motor1_Pwm = Blance_Car.Pwm->Balance_Pwm + Blance_Car.Pwm->Speed_Pwm - Blance_Car.Pwm->Turn_Pwm;
		Blance_Car.Pwm->Final_Motor2_Pwm = Blance_Car.Pwm->Balance_Pwm + Blance_Car.Pwm->Speed_Pwm + Blance_Car.Pwm->Turn_Pwm;		
		/*��С��״̬���м���жϲ����ս�PWMֵ������õ����*/
	
		if(Pick_Up(Blance_Car.Sensor->Accel_Z,Blance_Car.Sensor->Balance_Angle,Blance_Car.Speed->Motor1_Speed,Blance_Car.Speed->Motor2_Speed))       //===����Ƿ�С��������
			Blance_Car.Flag_Stop=1;	                                                     //===���������͹رյ��
		if(Put_Down(Blance_Car.Sensor->Balance_Angle,Blance_Car.Speed->Motor1_Speed,Blance_Car.Speed->Motor2_Speed))              //===����Ƿ�С��������
			Blance_Car.Flag_Stop=0;  
		if(Angle_Test() == 0 )
			Motor_Adj(Blance_Car.Pwm->Final_Motor1_Pwm,Blance_Car.Pwm->Final_Motor2_Pwm);
		
		vTaskDelay(10);
	}
}

