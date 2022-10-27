#include "u_iic.h"
#include "gpio.h"
void delay_us(uint16_t time)
{    
   uint16_t i=0;  
   while(time--)
   {
      i=72;  
      while(i--) ;    
   }
}
/******************************************************************************
*��  ����void MPU6050_SDA_IN(void)
*�����ܣ�SDA�˿ڳ�ʼ��Ϊ����
*��  ������
*����ֵ����
*��  ע����
*******************************************************************************/
void MPU6050_SDA_IN(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = MPU6050_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}
/******************************************************************************
*��  ����void MPU6050_SDA_OUT(void)
*�����ܣ�SDA�˿ڳ�ʼ��Ϊ����
*��  ������
*����ֵ����
*��  ע����
*******************************************************************************/
void MPU6050_SDA_OUT(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = MPU6050_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}
/******************************************************************************
*��  ����void IIC_Init(void)
*�����ܣ�IIC��ʼ��
*��  ������
*����ֵ����
*��  ע����
*******************************************************************************/
void MPU6050_IIC_Init(void)
{
	IIC_SCL_H;
	IIC_SDA_H;
}
/******************************************************************************
*��  ����void IIC_Start(void)
*�����ܣ�����IIC��ʼ�ź�
*��  ������
*����ֵ����
*��  ע����
*******************************************************************************/
void MPU6050_IIC_Start(void)
{
        MPU6050_SDA_OUT();
	IIC_SDA_H;
	IIC_SCL_H;
    delay_us(4);
 	IIC_SDA_L; //START:when CLK is high,DATA change form high to low
	delay_us(4);
	IIC_SCL_L; //ǯסI2C���ߣ�׼�����ͻ��������
}

/******************************************************************************
*��  ����void IIC_Stop(void)
*�����ܣ�����IICֹͣ�ź�
*��  ������
*����ֵ����
*��  ע����
*******************************************************************************/
void MPU6050_IIC_Stop(void)
{
        MPU6050_SDA_OUT();
	IIC_SCL_L;
	IIC_SDA_L; //STOP:when CLK is high DATA change form low to high
  delay_us(4);
	IIC_SCL_H;
	IIC_SDA_H; //����I2C���߽����ź�
  delay_us(4);
}

/******************************************************************************
*��  ��: uint8_t IIC_WaitAck(void)
*������: �ȴ�Ӧ���źŵ��� ����ЧӦ�𣺴ӻ���9�� SCL=0 ʱ SDA ���ӻ�����,
                            ���� SCL = 1ʱ SDA��ȻΪ�ͣ�
*��  ������
*����ֵ��1������Ӧ��ʧ��
         0������Ӧ��ɹ�
*��  ע���ӻ���������Ӧ��
*******************************************************************************/
uint8_t MPU6050_IIC_WaitAck(void)
{
	uint8_t ucErrTime=0;
	MPU6050_SDA_IN(); //SDA����Ϊ����  ���ӻ���һ���͵�ƽ��ΪӦ��
	IIC_SDA_H;delay_us(1);
	IIC_SCL_H;delay_us(1);;
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			MPU6050_IIC_Stop();
			return 1;
		}
	}
	IIC_SCL_L; //ʱ�����0
	return 0;
}

/******************************************************************************
*��  ��: void IIC_Ack(void)
*������: ����ACKӦ�� ������������һ���ֽ����ݺ�����������ACK֪ͨ�ӻ�һ��
                       �ֽ���������ȷ���գ�
*��  ������
*����ֵ����
*��  ע���������ӻ���Ӧ��
*******************************************************************************/

void IIC_Ack(void)
{
	IIC_SCL_L;
    MPU6050_SDA_OUT();
	IIC_SDA_L;
	delay_us(1);
	IIC_SCL_H;
	delay_us(2);
	IIC_SCL_L;
}

/******************************************************************************
*��  ��: void IIC_NAck(void)
*������: ����NACKӦ�� ���������������һ���ֽ����ݺ�����������NACK֪ͨ�ӻ�
                        ���ͽ������ͷ�SDA,�Ա���������ֹͣ�źţ�
*��  ������
*����ֵ����
*��  ע���������ӻ���Ӧ��
*******************************************************************************/
void IIC_NAck(void)
{
	IIC_SCL_L;
        MPU6050_SDA_OUT();
	IIC_SDA_H;
	delay_us(1);
	IIC_SCL_H;
	delay_us(1);
	IIC_SCL_L;
}

/******************************************************************************
*��  ����void IIC_SendByte(uint8_t txd)
*��  �ܣ�IIC����һ���ֽ�
*��  ����data Ҫд������
*����ֵ����
*��  ע���������ӻ���
*******************************************************************************/
void IIC_SendByte(uint8_t data)
{
    uint8_t t;
    MPU6050_SDA_OUT();
    IIC_SCL_L; //����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {
      if((data&0x80)>>7)IIC_SDA_H;
			else IIC_SDA_L;
      data<<=1;
			delay_us(1);
		  IIC_SCL_H;
		  delay_us(1);
			IIC_SCL_L;
		  delay_us(1);
    }
}

/******************************************************************************
*��  ����uint8_t IIC_ReadByte(uint8_t ack)
*��  �ܣ�IIC��ȡһ���ֽ�
*��  ����ack=1 ʱ���������ݻ�û������ ack=0 ʱ����������ȫ���������
*����ֵ����
*��  ע���ӻ���������
*******************************************************************************/
uint8_t IIC_ReadByte(uint8_t ack)
{
	uint8_t i,receive=0;
        MPU6050_SDA_IN();
  for(i=0;i<8;i++ )
	{
     IIC_SCL_L;
     delay_us(1);
		 IIC_SCL_H;
     receive<<=1;
     if(READ_SDA)receive++; //�ӻ����͵ĵ�ƽ
	   delay_us(1);
   }
    if(ack)
        IIC_Ack(); //����ACK
    else
        IIC_NAck(); //����nACK
    return receive;
}

/******************************************************************************
*��  ����uint8_t IIC_ReadByteFromSlave(uint8_t I2C_Addr,uint8_t addr)
*�����ܣ���ȡָ���豸 ָ���Ĵ�����һ��ֵ
*��  ����I2C_Addr  Ŀ���豸��ַ
		     reg	     �Ĵ�����ַ
         *buf      ��ȡ����Ҫ�洢�ĵ�ַ
*����ֵ������ 1ʧ�� 0�ɹ�
*��  ע����
*******************************************************************************/
uint8_t IIC_ReadByteFromSlave(uint8_t I2C_Addr,uint8_t reg,uint8_t *buf)
{
	MPU6050_IIC_Start();
	IIC_SendByte(I2C_Addr);	 //���ʹӻ���ַ
	if(MPU6050_IIC_WaitAck()) //����ӻ�δӦ�������ݷ���ʧ��
	{
		MPU6050_IIC_Stop();
		return 1;
	}
	IIC_SendByte(reg); //���ͼĴ�����ַ
	MPU6050_IIC_WaitAck();

	MPU6050_IIC_Start();
	IIC_SendByte(I2C_Addr+1); //�������ģʽ
	MPU6050_IIC_WaitAck();
	*buf=IIC_ReadByte(0);
        MPU6050_IIC_Stop(); //����һ��ֹͣ����
	return 0;
}

/******************************************************************************
*��  ����uint8_t IIC_WriteByteFromSlave(uint8_t I2C_Addr,uint8_t addr��uint8_t buf))
*�����ܣ�д��ָ���豸 ָ���Ĵ�����һ��ֵ
*��  ����I2C_Addr  Ŀ���豸��ַ
		     reg	     �Ĵ�����ַ
         buf       Ҫд�������
*����ֵ��1 ʧ�� 0�ɹ�
*��  ע����
*******************************************************************************/
uint8_t IIC_WriteByteToSlave(uint8_t I2C_Addr,uint8_t reg,uint8_t data)
{
	MPU6050_IIC_Start();
	IIC_SendByte(I2C_Addr); //���ʹӻ���ַ
	if(MPU6050_IIC_WaitAck())
	{
		MPU6050_IIC_Stop();
		return 1; //�ӻ���ַд��ʧ��
	}
	IIC_SendByte(reg); //���ͼĴ�����ַ
  MPU6050_IIC_WaitAck();
	IIC_SendByte(data);
	if(MPU6050_IIC_WaitAck())
	{
		MPU6050_IIC_Stop();
		return 1; //����д��ʧ��
	}
	MPU6050_IIC_Stop(); //����һ��ֹͣ����

  //return 1; //status == 0;
	return 0;
}

/******************************************************************************
*��  ����uint8_t IICreadBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data)
*�����ܣ���ȡָ���豸 ָ���Ĵ����� length��ֵ
*��  ����dev     Ŀ���豸��ַ
		     reg	   �Ĵ�����ַ
         length  Ҫ�����ֽ���
		     *data   ���������ݽ�Ҫ��ŵ�ָ��
*����ֵ��1�ɹ� 0ʧ��
*��  ע����
*******************************************************************************/
uint8_t IIC_ReadMultByteFromSlave(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data)
{
  uint8_t count = 0;
	uint8_t temp;
	MPU6050_IIC_Start();
	IIC_SendByte(dev); //���ʹӻ���ַ
	if(MPU6050_IIC_WaitAck())
	{
		MPU6050_IIC_Stop();
		return 1; //�ӻ���ַд��ʧ��
	}
	IIC_SendByte(reg); //���ͼĴ�����ַ
  MPU6050_IIC_WaitAck();
	MPU6050_IIC_Start();
	IIC_SendByte(dev+1); //�������ģʽ
	MPU6050_IIC_WaitAck();
  for(count=0;count<length;count++)
	{
		if(count!=(length-1))
		temp = IIC_ReadByte(1); //��ACK�Ķ�����
		else
		temp = IIC_ReadByte(0); //���һ���ֽ�NACK

		data[count] = temp;
	}
    MPU6050_IIC_Stop(); //����һ��ֹͣ����
    //return count;
	 return 0;
}

/******************************************************************************
*��  ����uint8_t IICwriteBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t* data)
*�����ܣ�������ֽ�д��ָ���豸 ָ���Ĵ���
*��  ����dev     Ŀ���豸��ַ
		     reg	   �Ĵ�����ַ
         length  Ҫд���ֽ���
		     *data   Ҫд������ݽ�Ҫ��ŵ�ָ��
*����ֵ��1�ɹ� 0ʧ��
*��  ע����
*******************************************************************************/
uint8_t IIC_WriteMultByteToSlave(uint8_t dev, uint8_t reg, uint8_t length, uint8_t* data)
{

 	uint8_t count = 0;
	MPU6050_IIC_Start();
	IIC_SendByte(dev); //���ʹӻ���ַ
	if(MPU6050_IIC_WaitAck())
	{
		MPU6050_IIC_Stop();
		return 1; //�ӻ���ַд��ʧ��
	}
	IIC_SendByte(reg); //���ͼĴ�����ַ
  MPU6050_IIC_WaitAck();
	for(count=0;count<length;count++)
	{
		IIC_SendByte(data[count]);
		if(MPU6050_IIC_WaitAck()) //ÿһ���ֽڶ�Ҫ�ȴӻ�Ӧ��
		{
			MPU6050_IIC_Stop();
			return 1; //����д��ʧ��
		}
	}
	MPU6050_IIC_Stop(); //����һ��ֹͣ����

	return 0;
}




