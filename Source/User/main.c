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

/*�Խϸߵ�Ƶ�ʲɼ���ǰ�Ƕ�ֵ*/
void Serial_Task(void *pvParameter)
{
	while(1)
	{
		Get_Angle(); //��ȡ��ǰ�Ƕ�
		Blance_Car.Speed->Motor1_Speed = Read_Encoder(2);
		Blance_Car.Speed->Motor2_Speed = -Read_Encoder(4);//�����������ӵ��ٶ�
		vTaskDelay(5);
	}
}

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
		//Xianfu_Pwm();//���Ʒ���
		if(Pick_Up(Blance_Car.Sensor->Accel_Z,Blance_Car.Sensor->Balance_Angle,Blance_Car.Speed->Motor1_Speed,Blance_Car.Speed->Motor2_Speed))       //===����Ƿ�С��������
			Flag_Stop=1;	                                                     //===���������͹رյ��
		if(Put_Down(Blance_Car.Sensor->Balance_Angle,Blance_Car.Speed->Motor1_Speed,Blance_Car.Speed->Motor2_Speed))              //===����Ƿ�С��������
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
	USART1_Config();                                               //���ڣ�9600�����ʴ��ڳ�ʼ��
	NVIC_Configuration();                                          
	
	ANBT_I2C_Configuration();	                                      //IIC��ʼ�� 
	AnBT_DMP_MPU6050_Init();		                                   	//6050DMP��ʼ��	
	
	Encoder_Init_TIM2();                                              
	Encoder_Init_TIM4();                                            //���������Լ�ʱ����ʼ��
	
	Motor_GPIO_Config();                                            //����ܽų�ʼ������
	TIM3_Config();                                                  //pwm��������ã���������ֹ��ǰ�����ж�
	
	xTaskCreate(afxInitTask, "InitTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL);
	/* FreeRTOS��������ʼ���� */
	vTaskStartScheduler();
	
	/* �˴�����ִ�е� */
	while(1)
	{	
	//	MPU6050_Pose();
  //printf("%d   /n",Read_Encoder(2));
		//DataScope_SendData(Pitch,Roll,Yaw,0,0,0,0,0,0);   //����λ����������
	}
}

