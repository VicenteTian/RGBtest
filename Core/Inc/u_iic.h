#ifndef __U_IIC_H
#define __U_IIC_H
#include "stm32f1xx_hal.h"
//IO��������

#define IIC_SCL_H    HAL_GPIO_WritePin(GPIOB,MPU6050_SCL_Pin,GPIO_PIN_SET) //SCL             2.�޸�ģ��iic����ʱ��Ҫ�޸�
#define IIC_SCL_L    HAL_GPIO_WritePin(GPIOB,MPU6050_SCL_Pin,GPIO_PIN_RESET)
#define IIC_SDA_H    HAL_GPIO_WritePin(GPIOB,MPU6050_SDA_Pin,GPIO_PIN_SET)
#define IIC_SDA_L    HAL_GPIO_WritePin(GPIOB,MPU6050_SDA_Pin,GPIO_PIN_RESET)
#define READ_SDA     HAL_GPIO_ReadPin(GPIOB,MPU6050_SDA_Pin)  //����SDA


//IIC_1���в�������
void delay_us(uint16_t time);
void MPU6050_SDA_IN(void);
void MPU6050_SDA_OUT(void);
void MPU6050_IIC_Init(void);        //��ʼ��IIC��IO��3.�޸�ģ��iic����ʱ��Ҫ�޸�
void MPU6050_IIC_Start(void);			  //����IIC��ʼ�ź�
void MPU6050_IIC_Stop(void);	  	  //����IICֹͣ�ź�
void IIC_Ack(void);					//IIC����ACK�ź�
void IIC_NAck(void);				//IIC������ACK�ź�
uint8_t MPU6050_IIC_WaitAck(void); 		 //IIC�ȴ�ACK�ź�

void IIC_SendByte(uint8_t data);  //IIC����һ���ֽ�
uint8_t IIC_ReadByte(uint8_t ack);//IIC��ȡһ���ֽ�
uint8_t IIC_ReadByteFromSlave(uint8_t I2C_Addr,uint8_t reg,uint8_t *buf);
uint8_t IIC_ReadMultByteFromSlave(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data);
uint8_t IIC_WriteByteToSlave(uint8_t I2C_Addr,uint8_t reg,uint8_t buf);
uint8_t IIC_WriteMultByteToSlave(uint8_t dev, uint8_t reg, uint8_t length, uint8_t* data);



#endif
