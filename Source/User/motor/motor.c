#include "motor.h"
#include "math.h"
#include "encoder.h"
#include "anbt_dmp_mpu6050.h"
#include "anbt_dmp_driver.h"
#include "anbt_i2c.h"
#include "DataScope_DP.h"


Car_t Blance_Car;


int Flag_Stop,pick;
extern short gyro[3];
extern short accel[3];
extern int Flag_Qian,Flag_Hou,Flag_Left,Flag_Right;


void TIM3_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM3,TIM_IT_Update)!=RESET)
	{
//		num++;
//		if(num == 25)
//			Get_Angle();//���ߵĲ���Ƶ��һ
//		if(num >= 50 )
//		{
//			num=0;
//			Blance_Car.Speed->Motor1_Speed=Read_Encoder(2);
//			Blance_Car.Speed->Motor2_Speed=-Read_Encoder(4);//�����������ӵ��ٶ�
//			Get_Angle();//��
//			Blance_Car.Pwm->Balance_Pwm=Balance_PD();//ƽ��PID����
//			Blance_Car.Pwm->Speed_Pwm=Speed_PI(Blance_Car.Speed->Motor1_Speed,Blance_Car.Speed->Motor2_Speed);//�ٶȻ�PID����
//			Blance_Car.Pwm->Turn_Pwm=Balance_Turn(Blance_Car.Speed->Motor1_Speed,Blance_Car.Speed->Motor2_Speed,Blance_Car.Sensor->Turn_Gyro);//ת�򻷿���		  
//			Motor1= Blance_Car.Pwm->Balance_Pwm+Blance_Car.Pwm->Speed_Pwm-Blance_Car.Pwm->Turn_Pwm;
//			Motor2= Blance_Car.Pwm->Balance_Pwm+Blance_Car.Pwm->Speed_Pwm+Blance_Car.Pwm->Turn_Pwm;		
//			Xianfu_Pwm();//���Ʒ���
//			if(Pick_Up(Blance_Car.Sensor->Accel_Z,Blance_Car.Sensor->Balance_Angle,Blance_Car.Speed->Motor1_Speed,Blance_Car.Speed->Motor2_Speed))       //===����Ƿ�С��������
//				Flag_Stop=1;	                                                     //===���������͹رյ��
//			if(Put_Down(Blance_Car.Sensor->Balance_Angle,Blance_Car.Speed->Motor1_Speed,Blance_Car.Speed->Motor2_Speed))              //===����Ƿ�С��������
//				Flag_Stop=0;  
//			if(Angle_Test() == 0 )
//				Motor_Adj(Motor1,Motor2);
//		}
	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);
	}
}


void Get_Angle(void)
{
	MPU6050_Pose();
	Blance_Car.Sensor->Balance_Angle=Pitch;
	Blance_Car.Sensor->Balance_Gyro=gyro[1];
	Blance_Car.Sensor->Turn_Gyro=gyro[2];
	Blance_Car.Sensor->Accel_Z=accel[2];
}

int Balance_PD(void)
{
	static float Balance;
	Balance=Blance_Car.Sensor->Balance_Angle*160+Blance_Car.Sensor->Balance_Gyro*1.36;
	return Balance;
}

int Speed_PI(int Encoder_Left,int Encoder_Right)
{
	static float Speed,Encoder_Least,Encoder,Target_Speed,Movement;
	static float Encoder_Integral;
	float kp=138,ki=kp/200;
	
	 //=============ң��ǰ�����˲���=======================// 
	Target_Speed=100;             
	if(1==Flag_Qian)    
		Movement=-Target_Speed/2;	         //===ǰ����־λ��1 
	else if(1==Flag_Hou)	
		Movement=Target_Speed/2;         //===���˱�־λ��1
	else  Movement=0;	

   //=============�ٶ�PI������=======================//	
	Encoder_Least =(Encoder_Left+Encoder_Right)-0;                    //===��ȡ�����ٶ�ƫ��==�����ٶȣ����ұ�����֮�ͣ�-Ŀ���ٶȣ��˴�Ϊ�㣩 
	Encoder *= 0.7;		                                                //===һ�׵�ͨ�˲���       
	Encoder += Encoder_Least*0.3;	                                    //===һ�׵�ͨ�˲���    
	Encoder_Integral +=Encoder;                                       //===���ֳ�λ�� ����ʱ�䣺10ms
	Encoder_Integral=Encoder_Integral-Movement;                       //===����ң�������ݣ�����ǰ������
	if(Encoder_Integral>15000)  Encoder_Integral=15000;             //===�����޷�
	if(Encoder_Integral<-15000)	Encoder_Integral=-15000;              //===�����޷�	
	Speed=Encoder*kp+Encoder_Integral*ki;                             //===�ٶȿ���	
	if(Flag_Stop==1)   Encoder_Integral=0;      //===����رպ��������
	 
	return Speed;
}

int Balance_Turn(int encoder_left,int encoder_right,float gyro)//ת�����
{
	static float Turn_Target,Turn,Kp=50,Kd=0;
	float Turn_Amplitude=25;   
	if(1==Flag_Left)	         
		Turn_Target-=2;        //===����ת��ң������
	else if(1==Flag_Right)	     
		Turn_Target+=2;        //===����ת��ң������
	else Turn_Target=0;                                            //===����ת��ң������
	if(Turn_Target>Turn_Amplitude)  
		Turn_Target=Turn_Amplitude;    //===ת���ٶ��޷�
	if(Turn_Target<-Turn_Amplitude) 
		Turn_Target=-Turn_Amplitude;   //===ת���ٶ��޷�
	if(Flag_Qian==1||Flag_Hou==1) 
		Kd=0.2;                         //===����ת��ң������ֱ�����ߵ�ʱ�����������Ǿ;���    
	else Kd=0;                                   
  	//=============ת��PD������=======================//
	Turn=Turn_Target*Kp+gyro*Kd;                 //===���Z�������ǽ���PD����
     
	return  Turn;
}


void Motor_Adj(int Motor1,int Motor2)
{
	/*���������õ�����ֵ�жϵ����ת��*/
	if( Motor1>0 )
	{
		INT0_1;
		INT1_0;
	}else {
		INT0_0;
		INT1_1;
	}

	if( Motor2>0 )
	{
		INT2_1;
		INT3_0;
	}else {		
		INT2_0;
		INT3_1;
  }
	/*�������յ����PWM��������ֵһֱ����ֵ��*/
	TIM3->CCR1=fabs(Motor1)+410;  
	TIM3->CCR2=fabs(Motor2)+410;//����������
}

//void Xianfu_Pwm(void)
//{
//	  int Amplitude=6900;    //===PWM������7200 ������6900
//    if(Motor1<-Amplitude) Motor1=-Amplitude;	
//		if(Motor1>Amplitude)  Motor1=Amplitude;	
//	  if(Motor2<-Amplitude) Motor2=-Amplitude;	
//		if(Motor2>Amplitude)  Motor2=Amplitude;		
//}

int Angle_Test(void)
{
	if(Blance_Car.Sensor->Balance_Angle>35||Blance_Car.Sensor->Balance_Angle<-35||Flag_Stop == 1)
	{
		INT0_0;
		INT1_0;
		INT2_0;
		INT3_0;
		return 1;
  }else 
	  return 0;
}


/**************************************************************************
�������ܣ����С���Ƿ�����
��ڲ�����int
����  ֵ��unsigned int
**************************************************************************/
int Pick_Up(float Acceleration,float Angle,int encoder_left,int encoder_right)
{ 		   
	static u16 flag,count1;
	if(++count1>10) count1=0;
	if(Acceleration>26000&&(Angle>-20)&&(Angle<20)&&(pick==0))   //����С������0�ȸ��������� 
	flag=1;
	if(flag==1)                                                             
	{
	if(fabs(encoder_left+encoder_right)>100)                                 //����3��С������̥��Ϊ�������ﵽ����ת��   
	{
		flag=0;   
		pick=1;				
		return 1;                                                               //��⵽С��������
	}
	}
	 return 0;
}
/**************************************************************************
�������ܣ����С���Ƿ񱻷���
��ڲ�����int
����  ֵ��unsigned int
**************************************************************************/
int Put_Down(float Angle,int encoder_left,int encoder_right)
{ 		   
	static u16 flag,count;	 
	if(Flag_Stop==0)                           //��ֹ���      
	return 0;	                 
	if(flag == 0 && pick == 1)                                               
	{
	  if(Angle>(-3)&&Angle<(3)&&encoder_left==0&&encoder_right==0)         //����1��С������0�ȸ�����
		flag=1; 
	} 
	if(flag==1)                                               
	{
	  if(++count>300)                                          //��ʱ���ٵȴ�1.5s
	  {
			count=0;
			flag=0;
			pick=0;
			return 1;                                            //��⵽С��������
		}
	}
	return 0;
}


void Motor_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
}

