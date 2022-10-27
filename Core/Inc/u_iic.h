#ifndef __U_IIC_H
#define __U_IIC_H
#include "stm32f1xx_hal.h"
//IO操作函数

#define IIC_SCL_H    HAL_GPIO_WritePin(GPIOB,MPU6050_SCL_Pin,GPIO_PIN_SET) //SCL             2.修改模拟iic引脚时需要修改
#define IIC_SCL_L    HAL_GPIO_WritePin(GPIOB,MPU6050_SCL_Pin,GPIO_PIN_RESET)
#define IIC_SDA_H    HAL_GPIO_WritePin(GPIOB,MPU6050_SDA_Pin,GPIO_PIN_SET)
#define IIC_SDA_L    HAL_GPIO_WritePin(GPIOB,MPU6050_SDA_Pin,GPIO_PIN_RESET)
#define READ_SDA     HAL_GPIO_ReadPin(GPIOB,MPU6050_SDA_Pin)  //输入SDA


//IIC_1所有操作函数
void delay_us(uint16_t time);
void MPU6050_SDA_IN(void);
void MPU6050_SDA_OUT(void);
void MPU6050_IIC_Init(void);        //初始化IIC的IO口3.修改模拟iic引脚时需要修改
void MPU6050_IIC_Start(void);			  //发送IIC开始信号
void MPU6050_IIC_Stop(void);	  	  //发送IIC停止信号
void IIC_Ack(void);					//IIC发送ACK信号
void IIC_NAck(void);				//IIC不发送ACK信号
uint8_t MPU6050_IIC_WaitAck(void); 		 //IIC等待ACK信号

void IIC_SendByte(uint8_t data);  //IIC发送一个字节
uint8_t IIC_ReadByte(uint8_t ack);//IIC读取一个字节
uint8_t IIC_ReadByteFromSlave(uint8_t I2C_Addr,uint8_t reg,uint8_t *buf);
uint8_t IIC_ReadMultByteFromSlave(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data);
uint8_t IIC_WriteByteToSlave(uint8_t I2C_Addr,uint8_t reg,uint8_t buf);
uint8_t IIC_WriteMultByteToSlave(uint8_t dev, uint8_t reg, uint8_t length, uint8_t* data);



#endif
