/*************************************************************************************************************
Բ�㲩ʿС���������2014������Դ��������:
��Դ��������ο�,Բ�㲩ʿ����Դ�����ṩ�κ���ʽ�ĵ���,Ҳ������ʹ�ø�Դ��������ֵ���ʧ����.
�û�������ѧϰ��Ŀ���޸ĺ�ʹ�ø�Դ����.
���û����޸ĸ�Դ����ʱ,�����Ƴ��ò��ְ�Ȩ��Ϣ�����뱣��ԭ������.

������Ϣ������ʹٷ���վwww.etootle.com, �ٷ�����:http://weibo.com/xiaosizhou
**************************************************************************************************************/
#include "anbt_i2c.h"
#include "stdio.h"

void ANBT_I2C_Configuration(void)			
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	//
	printf("M-4,Init I2C Device.");
	//
	GPIO_InitStructure.GPIO_Pin = ANBT_I2C_SCL | ANBT_I2C_SDA;					//Բ�㲩ʿ:����ʹ�õ�I2C��
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;   //Բ�㲩ʿ:����I2C�������������ٶ�
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;	  //Բ�㲩ʿ:����I2CΪ��©���
	GPIO_Init(ANBT_I2C_PORT, &GPIO_InitStructure); 
	//
	GPIO_InitStructure.GPIO_Pin = AnBT_MPU6050_INT;					//Բ�㲩ʿ:����ʹ�õ�I2C��
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;   //Բ�㲩ʿ:����I2C�������������ٶ�
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;	  //Բ�㲩ʿ:����I2CΪ��©���
	GPIO_Init(AnBT_MPU6050_INT_PORT, &GPIO_InitStructure); 
	//
	ANBT_I2C_SCL_1; 
	ANBT_I2C_SDA_1;
	ANBT_I2C_DELAY;
}

void ANBT_I2C_Delay(u32 dly)               
{
	while(--dly);	//dly=100: 8.75us; dly=100: 85.58 us (SYSCLK=72MHz)
}

u8 ANBT_I2C_START(void)
{ 
	ANBT_I2C_SDA_1; 
 	ANBT_I2C_NOP;
  // 
 	ANBT_I2C_SCL_1; 
 	ANBT_I2C_NOP;    
	//
 	if(!ANBT_I2C_SDA_STATE) return ANBT_I2C_BUS_BUSY;
	//
 	ANBT_I2C_SDA_0;
 	ANBT_I2C_NOP;
  //
 	ANBT_I2C_SCL_0;  
 	ANBT_I2C_NOP; 
	//
 	if(ANBT_I2C_SDA_STATE) return ANBT_I2C_BUS_ERROR;
	//
 	return ANBT_I2C_READY;
}

void ANBT_I2C_STOP(void)
{
 	ANBT_I2C_SDA_0; 
 	ANBT_I2C_NOP;
  // 
 	ANBT_I2C_SCL_1; 
 	ANBT_I2C_NOP;    
	//
 	ANBT_I2C_SDA_1;
 	ANBT_I2C_NOP;
}

void ANBT_I2C_SendACK(void)
{
 	ANBT_I2C_SDA_0;
 	ANBT_I2C_NOP;
 	ANBT_I2C_SCL_1;
 	ANBT_I2C_NOP;
 	ANBT_I2C_SCL_0; 
 	ANBT_I2C_NOP;  
}

void ANBT_I2C_SendNACK(void)
{
	ANBT_I2C_SDA_1;
	ANBT_I2C_NOP;
	ANBT_I2C_SCL_1;
	ANBT_I2C_NOP;
	ANBT_I2C_SCL_0; 
	ANBT_I2C_NOP;  
}

u8 ANBT_I2C_SendByte(u8 anbt_i2c_data)
{
 	u8 i;
 	
	ANBT_I2C_SCL_0;
 	for(i=0;i<8;i++)
 	{  
  		if(anbt_i2c_data&0x80) ANBT_I2C_SDA_1;
   		else ANBT_I2C_SDA_0;
			//
  		anbt_i2c_data<<=1;
  		ANBT_I2C_NOP;
			//
  		ANBT_I2C_SCL_1;
  		ANBT_I2C_NOP;
  		ANBT_I2C_SCL_0;
  		ANBT_I2C_NOP; 
 	}
	//
 	ANBT_I2C_SDA_1; 
 	ANBT_I2C_NOP;
 	ANBT_I2C_SCL_1;
 	ANBT_I2C_NOP;   
 	if(ANBT_I2C_SDA_STATE)
 	{
  		ANBT_I2C_SCL_0;
  		return ANBT_I2C_NACK;
 	}
 	else
 	{
  		ANBT_I2C_SCL_0;
  		return ANBT_I2C_ACK;  
 	}    
}

u8 ANBT_I2C_ReceiveByte(void)
{
	u8 i,anbt_i2c_data;
	//
 	ANBT_I2C_SDA_1;
 	ANBT_I2C_SCL_0; 
 	anbt_i2c_data=0;
	//
 	for(i=0;i<8;i++)
 	{
  		ANBT_I2C_SCL_1;
  		ANBT_I2C_NOP; 
  		anbt_i2c_data<<=1;
			//
  		if(ANBT_I2C_SDA_STATE)	anbt_i2c_data|=0x01; 
  
  		ANBT_I2C_SCL_0;  
  		ANBT_I2C_NOP;         
 	}
	ANBT_I2C_SendNACK();
 	return anbt_i2c_data;
}

//������������IIC����
/*******************************************************************************************************/

u8 ANBT_I2C_ReceiveByte_WithACK(void)
{
	u8 i,anbt_i2c_data;
	//
 	ANBT_I2C_SDA_1;
 	ANBT_I2C_SCL_0; 
 	anbt_i2c_data=0;
	//
 	for(i=0;i<8;i++)
 	{
  		ANBT_I2C_SCL_1;
  		ANBT_I2C_NOP; 
  		anbt_i2c_data<<=1;
			//
  		if(ANBT_I2C_SDA_STATE)	anbt_i2c_data|=0x01; 
  
  		ANBT_I2C_SCL_0;  
  		ANBT_I2C_NOP;         
 	}
	ANBT_I2C_SendACK();
 	return anbt_i2c_data;
}

void ANBT_I2C_Receive6Bytes(u8 *anbt_i2c_data_buffer)
{
	u8 i,j;
	u8 anbt_i2c_data;

	for(j=0;j<5;j++)
	{
		ANBT_I2C_SDA_1;
		ANBT_I2C_SCL_0; 
		anbt_i2c_data=0;
		//
		for(i=0;i<8;i++)
		{
  		ANBT_I2C_SCL_1;
  		ANBT_I2C_NOP; 
  		anbt_i2c_data<<=1;
			//
  		if(ANBT_I2C_SDA_STATE)	anbt_i2c_data|=0x01; 
  
  		ANBT_I2C_SCL_0;  
  		ANBT_I2C_NOP;         
		}
		anbt_i2c_data_buffer[j]=anbt_i2c_data;
		ANBT_I2C_SendACK();
	}
	//
	ANBT_I2C_SDA_1;
	ANBT_I2C_SCL_0; 
	anbt_i2c_data=0;
	for(i=0;i<8;i++)
	{
  	ANBT_I2C_SCL_1;
  	ANBT_I2C_NOP; 
  	anbt_i2c_data<<=1;
			//
  	if(ANBT_I2C_SDA_STATE)	anbt_i2c_data|=0x01; 
  
  	ANBT_I2C_SCL_0;  
  	ANBT_I2C_NOP;         
	}
	anbt_i2c_data_buffer[5]=anbt_i2c_data;
	ANBT_I2C_SendNACK();
}

void ANBT_I2C_Receive12Bytes(u8 *anbt_i2c_data_buffer)
{
	u8 i,j;
	u8 anbt_i2c_data;

	for(j=0;j<11;j++)
	{
		ANBT_I2C_SDA_1;
		ANBT_I2C_SCL_0; 
		anbt_i2c_data=0;
		//
		for(i=0;i<8;i++)
		{
  		ANBT_I2C_SCL_1;
  		ANBT_I2C_NOP; 
  		anbt_i2c_data<<=1;
			//
  		if(ANBT_I2C_SDA_STATE)	anbt_i2c_data|=0x01; 
  
  		ANBT_I2C_SCL_0;  
  		ANBT_I2C_NOP;         
		}
		anbt_i2c_data_buffer[j]=anbt_i2c_data;
		ANBT_I2C_SendACK();
	}
	//
	ANBT_I2C_SDA_1;
	ANBT_I2C_SCL_0; 
	anbt_i2c_data=0;
	for(i=0;i<8;i++)
	{
  	ANBT_I2C_SCL_1;
  	ANBT_I2C_NOP; 
  	anbt_i2c_data<<=1;
			//
  	if(ANBT_I2C_SDA_STATE)	anbt_i2c_data|=0x01; 
  
  	ANBT_I2C_SCL_0;  
  	ANBT_I2C_NOP;         
	}
	anbt_i2c_data_buffer[11]=anbt_i2c_data;
	ANBT_I2C_SendNACK();
}

void ANBT_I2C_Receive14Bytes(u8 *anbt_i2c_data_buffer)
{
	u8 i,j;
	u8 anbt_i2c_data;

	for(j=0;j<13;j++)
	{
		ANBT_I2C_SDA_1;
		ANBT_I2C_SCL_0; 
		anbt_i2c_data=0;
		//
		for(i=0;i<8;i++)
		{
  		ANBT_I2C_SCL_1;
  		ANBT_I2C_NOP; 
  		anbt_i2c_data<<=1;
			//
  		if(ANBT_I2C_SDA_STATE)	anbt_i2c_data|=0x01; 
  
  		ANBT_I2C_SCL_0;  
  		ANBT_I2C_NOP;         
		}
		anbt_i2c_data_buffer[j]=anbt_i2c_data;
		ANBT_I2C_SendACK();
	}
	//
	ANBT_I2C_SDA_1;
	ANBT_I2C_SCL_0; 
	anbt_i2c_data=0;
	for(i=0;i<8;i++)
	{
  	ANBT_I2C_SCL_1;
  	ANBT_I2C_NOP; 
  	anbt_i2c_data<<=1;
			//
  	if(ANBT_I2C_SDA_STATE)	anbt_i2c_data|=0x01; 
  
  	ANBT_I2C_SCL_0;  
  	ANBT_I2C_NOP;         
	}
	anbt_i2c_data_buffer[13]=anbt_i2c_data;
	ANBT_I2C_SendNACK();
}

void AnBT_DMP_Delay_us(u32 dly)
{
	u8 i;
	while(dly--) for(i=0;i<10;i++);
}
//
void AnBT_DMP_Delay_ms(u32 dly)
{
	while(dly--) AnBT_DMP_Delay_us(1000);
}
//

u8 AnBT_DMP_I2C_Write(u8 anbt_dev_addr, u8 anbt_reg_addr, u8 anbt_i2c_len, u8 *anbt_i2c_data_buf)
{		
		u8 i;
		ANBT_I2C_START();
		ANBT_I2C_SendByte(anbt_dev_addr << 1 | 0);					//Բ�㲩ʿ:����������д��ַ
		ANBT_I2C_SendByte(anbt_reg_addr);  //Բ�㲩ʿ:����������PWM��ַ
		for (i=0;i<anbt_i2c_len;i++) ANBT_I2C_SendByte(anbt_i2c_data_buf[i]); //Բ�㲩ʿ:����������PWMֵ
		ANBT_I2C_STOP();
		return 0x00;
}
u8 AnBT_DMP_I2C_Read(u8 anbt_dev_addr, u8 anbt_reg_addr, u8 anbt_i2c_len, u8 *anbt_i2c_data_buf)
{
	
		ANBT_I2C_START();
		ANBT_I2C_SendByte(anbt_dev_addr << 1 | 0);			//Բ�㲩ʿ:����������д��ַ
		ANBT_I2C_SendByte(anbt_reg_addr);  //Բ�㲩ʿ:����������ID��ַ
		ANBT_I2C_START();
		ANBT_I2C_SendByte(anbt_dev_addr << 1 | 1);      //Բ�㲩ʿ:���������Ƕ���ַ
		//
    while (anbt_i2c_len) 
		{
			if (anbt_i2c_len==1) *anbt_i2c_data_buf =ANBT_I2C_ReceiveByte();  
      else *anbt_i2c_data_buf =ANBT_I2C_ReceiveByte_WithACK();
      anbt_i2c_data_buf++;
      anbt_i2c_len--;
    }
		ANBT_I2C_STOP();
    return 0x00;
}
