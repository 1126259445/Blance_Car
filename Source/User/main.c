#include "stm32f10x.h"
#include "bsp_usart1.h"
#include "anbt_dmp_mpu6050.h"
#include "anbt_dmp_driver.h"
#include "anbt_i2c.h"
#include "DataScope_DP.h"
#include "pwm_out.h"
#include "motor.h"
#include "encoder.h"

#include "FreeRTOS.h"
#include "task.h"

/*以较高的频率采集当前角度值*/
void Serial_Task(void *pvParameter)
{
	while(1)
	{
		Get_Angle(); //读取当前角度
		Blance_Car.Speed->Motor1_Speed = Read_Encoder(2);
		Blance_Car.Speed->Motor2_Speed = -Read_Encoder(4);//读出两个轮子的速度
		vTaskDelay(5);
	}
}

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
		//Xianfu_Pwm();//限制幅度
		if(Pick_Up(Blance_Car.Sensor->Accel_Z,Blance_Car.Sensor->Balance_Angle,Blance_Car.Speed->Motor1_Speed,Blance_Car.Speed->Motor2_Speed))       //===检查是否小车被那起
			Flag_Stop=1;	                                                     //===如果被拿起就关闭电机
		if(Put_Down(Blance_Car.Sensor->Balance_Angle,Blance_Car.Speed->Motor1_Speed,Blance_Car.Speed->Motor2_Speed))              //===检查是否小车被放下
			Flag_Stop=0;  
		if(Angle_Test() == 0 )
			Motor_Adj(Blance_Car.Pwm->Final_Motor1_Pwm,Blance_Car.Pwm->Final_Motor2_Pwm);
		
		vTaskDelay(10);
	}
}

void afxInitTask(void *pvParameter)
{
	xTaskCreate(Serial_Task, "Serial_Task", configMINIMAL_STACK_SIZE*2, NULL, tskIDLE_PRIORITY+3, NULL);
	xTaskCreate(PID_control_Task, "PID_control_Task", configMINIMAL_STACK_SIZE*2, NULL, tskIDLE_PRIORITY+3, NULL);
	vTaskDelete(NULL);
}


int main(void)
{
	USART1_Config();                                               //串口，9600波特率串口初始化
	NVIC_Configuration();                                          
	
	ANBT_I2C_Configuration();	                                      //IIC初始化 
	AnBT_DMP_MPU6050_Init();		                                   	//6050DMP初始化	
	
	Encoder_Init_TIM2();                                              
	Encoder_Init_TIM4();                                            //编码器测试计时器初始化
	
	Motor_GPIO_Config();                                            //电机管脚初始化配置
	TIM3_Config();                                                  //pwm波输出配置，放在最后防止提前进入中断
	
	xTaskCreate(afxInitTask, "InitTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL);
	/* FreeRTOS调度器开始工作 */
	vTaskStartScheduler();
	
	/* 此处不会执行到 */
	while(1)
	{	
	//	MPU6050_Pose();
  //printf("%d   /n",Read_Encoder(2));
		//DataScope_SendData(Pitch,Roll,Yaw,0,0,0,0,0,0);   //向上位机发送数据
	}
}

