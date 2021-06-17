#include "stm32f10x.h"
#include "bsp_usart1.h"
#include "anbt_dmp_mpu6050.h"
#include "anbt_dmp_driver.h"
#include "anbt_i2c.h"
#include "DataScope_DP.h"
#include "pwm_out.h"
#include "motor.h"
#include "encoder.h"
#include "radio.h"
#include "pid_countrol.h"
#include "speed.h"

#include "FreeRTOS.h"
#include "task.h"



void afxInitTask(void *pvParameter)
{
	xTaskCreate(Encode_Task, "Encode_Task", configMINIMAL_STACK_SIZE*2, NULL, tskIDLE_PRIORITY+3, NULL);
	xTaskCreate(PID_control_Task, "PID_control_Task", configMINIMAL_STACK_SIZE*2, NULL, tskIDLE_PRIORITY+3, NULL);
	xTaskCreate(radio_Task, "radio_Task", configMINIMAL_STACK_SIZE*2, NULL, tskIDLE_PRIORITY+3, NULL);
	vTaskDelete(NULL);
}


int main(void)
{
	USART1_Config();                                                //串口，9600波特率串口初始化
	NVIC_Configuration();                                          
	
	ANBT_I2C_Configuration();	                                    //IIC初始化 
	AnBT_DMP_MPU6050_Init();		                                //6050DMP初始化	
	
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

