

#ifndef __DATA_PRTOCOL_H
#define __DATA_PRTOCOL_H
 
 #include "stm32f10x.h"
 
extern unsigned char DataScope_OutPut_Buffer[42];	   //待发送帧数据缓存区

void DataScope_SendData(float Pitch,float Roll,float Yaw,float Accel_x,float Accel_y,float Accel_z,float Gyro_x,float Gyro_y,float Gyro_z);//发送数据

void DataScope_Get_Channel_Data(float Data,unsigned char Channel);    // 写通道数据至 待发送帧数据缓存区

unsigned char DataScope_Data_Generate(unsigned char Channel_Number);  // 发送帧数据生成函数 
 
 void delay_ms(uint32_t ms);//延时50ms左右
 
#endif 



