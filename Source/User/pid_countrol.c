#include "pid_countrol.h"
#include "motor.h"
#include "FreeRTOS.h"
#include "task.h"

/*PID控制任务*/
void PID_control_Task(void *pvParameter)
{
	while(1)
	{		
		/*计算和控制环的PWM值*/
		Blance_Car.Pwm->Balance_Pwm = Balance_PD();//平衡PID控制
		Blance_Car.Pwm->Speed_Pwm = Speed_PI(Blance_Car.Speed->Motor1_Speed,Blance_Car.Speed->Motor2_Speed);//速度环PID控制
		Blance_Car.Pwm->Turn_Pwm = Balance_Turn(Blance_Car.Speed->Motor1_Speed,Blance_Car.Speed->Motor2_Speed,Blance_Car.Sensor->Turn_Gyro);//转向环控制		
		/*计算最终的PWM输出值*/
		Blance_Car.Pwm->Final_Motor1_Pwm = Blance_Car.Pwm->Balance_Pwm + Blance_Car.Pwm->Speed_Pwm - Blance_Car.Pwm->Turn_Pwm;
		Blance_Car.Pwm->Final_Motor2_Pwm = Blance_Car.Pwm->Balance_Pwm + Blance_Car.Pwm->Speed_Pwm + Blance_Car.Pwm->Turn_Pwm;		
		/*对小车状态进行检测判断并最终将PWM值输出作用到电机*/
	
		if(Pick_Up(Blance_Car.Sensor->Accel_Z,Blance_Car.Sensor->Balance_Angle,Blance_Car.Speed->Motor1_Speed,Blance_Car.Speed->Motor2_Speed))       //===检查是否小车被那起
			Blance_Car.Flag_Stop=1;	                                                     //===如果被拿起就关闭电机
		if(Put_Down(Blance_Car.Sensor->Balance_Angle,Blance_Car.Speed->Motor1_Speed,Blance_Car.Speed->Motor2_Speed))              //===检查是否小车被放下
			Blance_Car.Flag_Stop=0;  
		if(Angle_Test() == 0 )
			Motor_Adj(Blance_Car.Pwm->Final_Motor1_Pwm,Blance_Car.Pwm->Final_Motor2_Pwm);
		
		vTaskDelay(10);
	}
}

