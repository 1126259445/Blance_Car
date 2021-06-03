

#ifndef __DATA_PRTOCOL_H
#define __DATA_PRTOCOL_H
 
 #include "stm32f10x.h"
 
extern unsigned char DataScope_OutPut_Buffer[42];	   //������֡���ݻ�����

void DataScope_SendData(float Pitch,float Roll,float Yaw,float Accel_x,float Accel_y,float Accel_z,float Gyro_x,float Gyro_y,float Gyro_z);//��������

void DataScope_Get_Channel_Data(float Data,unsigned char Channel);    // дͨ�������� ������֡���ݻ�����

unsigned char DataScope_Data_Generate(unsigned char Channel_Number);  // ����֡�������ɺ��� 
 
 void delay_ms(uint32_t ms);//��ʱ50ms����
 
#endif 



