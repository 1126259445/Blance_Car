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
//			Get_Angle();//更高的采样频率一
//		if(num >= 50 )
//		{
//			num=0;
//			Blance_Car.Speed->Motor1_Speed=Read_Encoder(2);
//			Blance_Car.Speed->Motor2_Speed=-Read_Encoder(4);//读出两个轮子的速度
//			Get_Angle();//二
//			Blance_Car.Pwm->Balance_Pwm=Balance_PD();//平衡PID控制
//			Blance_Car.Pwm->Speed_Pwm=Speed_PI(Blance_Car.Speed->Motor1_Speed,Blance_Car.Speed->Motor2_Speed);//速度环PID控制
//			Blance_Car.Pwm->Turn_Pwm=Balance_Turn(Blance_Car.Speed->Motor1_Speed,Blance_Car.Speed->Motor2_Speed,Blance_Car.Sensor->Turn_Gyro);//转向环控制		  
//			Motor1= Blance_Car.Pwm->Balance_Pwm+Blance_Car.Pwm->Speed_Pwm-Blance_Car.Pwm->Turn_Pwm;
//			Motor2= Blance_Car.Pwm->Balance_Pwm+Blance_Car.Pwm->Speed_Pwm+Blance_Car.Pwm->Turn_Pwm;		
//			Xianfu_Pwm();//限制幅度
//			if(Pick_Up(Blance_Car.Sensor->Accel_Z,Blance_Car.Sensor->Balance_Angle,Blance_Car.Speed->Motor1_Speed,Blance_Car.Speed->Motor2_Speed))       //===检查是否小车被那起
//				Flag_Stop=1;	                                                     //===如果被拿起就关闭电机
//			if(Put_Down(Blance_Car.Sensor->Balance_Angle,Blance_Car.Speed->Motor1_Speed,Blance_Car.Speed->Motor2_Speed))              //===检查是否小车被放下
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
	
	 //=============遥控前进后退部分=======================// 
	Target_Speed=100;             
	if(1==Flag_Qian)    
		Movement=-Target_Speed/2;	         //===前进标志位置1 
	else if(1==Flag_Hou)	
		Movement=Target_Speed/2;         //===后退标志位置1
	else  Movement=0;	

   //=============速度PI控制器=======================//	
	Encoder_Least =(Encoder_Left+Encoder_Right)-0;                    //===获取最新速度偏差==测量速度（左右编码器之和）-目标速度（此处为零） 
	Encoder *= 0.7;		                                                //===一阶低通滤波器       
	Encoder += Encoder_Least*0.3;	                                    //===一阶低通滤波器    
	Encoder_Integral +=Encoder;                                       //===积分出位移 积分时间：10ms
	Encoder_Integral=Encoder_Integral-Movement;                       //===接收遥控器数据，控制前进后退
	if(Encoder_Integral>15000)  Encoder_Integral=15000;             //===积分限幅
	if(Encoder_Integral<-15000)	Encoder_Integral=-15000;              //===积分限幅	
	Speed=Encoder*kp+Encoder_Integral*ki;                             //===速度控制	
	if(Flag_Stop==1)   Encoder_Integral=0;      //===电机关闭后清除积分
	 
	return Speed;
}

int Balance_Turn(int encoder_left,int encoder_right,float gyro)//转向控制
{
	static float Turn_Target,Turn,Kp=50,Kd=0;
	float Turn_Amplitude=25;   
	if(1==Flag_Left)	         
		Turn_Target-=2;        //===接收转向遥控数据
	else if(1==Flag_Right)	     
		Turn_Target+=2;        //===接收转向遥控数据
	else Turn_Target=0;                                            //===接收转向遥控数据
	if(Turn_Target>Turn_Amplitude)  
		Turn_Target=Turn_Amplitude;    //===转向速度限幅
	if(Turn_Target<-Turn_Amplitude) 
		Turn_Target=-Turn_Amplitude;   //===转向速度限幅
	if(Flag_Qian==1||Flag_Hou==1) 
		Kd=0.2;                         //===接收转向遥控数据直立行走的时候增加陀螺仪就纠正    
	else Kd=0;                                   
  	//=============转向PD控制器=======================//
	Turn=Turn_Target*Kp+gyro*Kd;                 //===结合Z轴陀螺仪进行PD控制
     
	return  Turn;
}


void Motor_Adj(int Motor1,int Motor2)
{
	/*根据设设置的期望值判断电机的转向*/
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
	/*设置最终电机的PWM输出（这个值一直是正值）*/
	TIM3->CCR1=fabs(Motor1)+410;  
	TIM3->CCR2=fabs(Motor2)+410;//加入电机死区
}

//void Xianfu_Pwm(void)
//{
//	  int Amplitude=6900;    //===PWM满幅是7200 限制在6900
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
函数功能：检测小车是否被拿起
入口参数：int
返回  值：unsigned int
**************************************************************************/
int Pick_Up(float Acceleration,float Angle,int encoder_left,int encoder_right)
{ 		   
	static u16 flag,count1;
	if(++count1>10) count1=0;
	if(Acceleration>26000&&(Angle>-20)&&(Angle<20)&&(pick==0))   //条件小车是在0度附近被拿起 
	flag=1;
	if(flag==1)                                                             
	{
	if(fabs(encoder_left+encoder_right)>100)                                 //条件3，小车的轮胎因为正反馈达到最大的转速   
	{
		flag=0;   
		pick=1;				
		return 1;                                                               //检测到小车被拿起
	}
	}
	 return 0;
}
/**************************************************************************
函数功能：检测小车是否被放下
入口参数：int
返回  值：unsigned int
**************************************************************************/
int Put_Down(float Angle,int encoder_left,int encoder_right)
{ 		   
	static u16 flag,count;	 
	if(Flag_Stop==0)                           //防止误检      
	return 0;	                 
	if(flag == 0 && pick == 1)                                               
	{
	  if(Angle>(-3)&&Angle<(3)&&encoder_left==0&&encoder_right==0)         //条件1，小车是在0度附近的
		flag=1; 
	} 
	if(flag==1)                                               
	{
	  if(++count>300)                                          //超时不再等待1.5s
	  {
			count=0;
			flag=0;
			pick=0;
			return 1;                                            //检测到小车被放下
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

