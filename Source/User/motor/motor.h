#ifndef _MOTOR_H
#define _MOTOR_H

#include "stm32f10x.h"

#define INT0_0 GPIO_ResetBits(GPIOB,GPIO_Pin_12)
#define INT0_1 GPIO_SetBits(GPIOB,GPIO_Pin_12)
#define INT1_0 GPIO_ResetBits(GPIOB,GPIO_Pin_13)
#define INT1_1 GPIO_SetBits(GPIOB,GPIO_Pin_13)
#define INT2_0 GPIO_ResetBits(GPIOB,GPIO_Pin_14)
#define INT2_1 GPIO_SetBits(GPIOB,GPIO_Pin_14)
#define INT3_0 GPIO_ResetBits(GPIOB,GPIO_Pin_15)
#define INT3_1 GPIO_SetBits(GPIOB,GPIO_Pin_15)


typedef struct
{
	float Balance_Angle;
	float Balance_Gyro;
	float Turn_Gyro;
	float Accel_Z;
}Sensor_t;

typedef struct
{
	int Motor1_Speed;
	int Motor2_Speed;
}Speed_t;

typedef struct
{
	int Balance_Pwm;
	int Speed_Pwm;
	int Turn_Pwm;
	
	int Final_Motor1_Pwm;
	int Final_Motor2_Pwm;
}Pwm_t;

typedef struct
{
	Sensor_t *Sensor;
	Speed_t *Speed;
	Pwm_t	*Pwm;
}Car_t;
extern Car_t Blance_Car;


extern int Flag_Stop,pick;




void TIM3_IRQHandler(void);
void Get_Angle(void);
int Balance_PD(void);
int Speed_PI(int Encoder_Left,int Encoder_Right);
int Balance_Turn(int Encoder_Left,int Encoder_Right,float gyro);
void Motor_GPIO_Config(void);
void Motor_Adj(int Moto21,int Motor2);
void Xianfu_Pwm(void);
int Angle_Test(void);
int Pick_Up(float Acceleration,float Angle,int encoder_left,int encoder_right);
int Put_Down(float Angle,int encoder_left,int encoder_right);


#endif
