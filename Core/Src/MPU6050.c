#include "MPU6050.h"
#include "UART.h"
#include "u_iic.h"
float var_MPU6050[7]={0};                //���Ҫ��������λ���Բ�����ʾ���ݵ�����     ������
float MPU_error=0;                     //��̬��ƫ��
float HBAngle=0;                       //�����˲���ĽǶ�
uint8_t state=0;
uint8_t last_state=5;
uint8_t MPU6050_Init(void)
{
    uint8_t res;
    MPU6050_IIC_Init();     ////MPU6050 ֧��400K I2C
    res=MPU_Read_Byte(MPU6050_ADDR,WHO_AM_I);           //��ȡMPU6050��ID
//    if(res!=MPU6050_ID) //����ID��ȷ
//    {
//        printf("ID=%#X\r\n",res);
//        printf("MPU6050 is fail!\n");
//    }
//    else  printf("MPU6050 is OK!\n");

    res = 0;
    res += MPU_Write_Byte(MPU6050_ADDR,MPU_PWR_MGMT1_REG,0X80);//��λMPU6050
    HAL_Delay(100);  //��ʱ100ms
    res += MPU_Write_Byte(MPU6050_ADDR,MPU_PWR_MGMT1_REG,0X00);//����MPU6050
    res += MPU_Set_Gyro_Fsr(3);					        	//�����Ǵ�����,��2000dps
    res += MPU_Set_Accel_Fsr(1);					       	 	//���ٶȴ�����,��4g
    res += MPU_Set_Rate(50);						       	 	//���ò�����50Hz
    res += MPU_Write_Byte(MPU6050_ADDR,MPU_CFG_REG,0x02);      //�������ֵ�ͨ�˲���   98hz
    res += MPU_Write_Byte(MPU6050_ADDR,MPU_INT_EN_REG,0X00);   //�ر������ж�
    res += MPU_Write_Byte(MPU6050_ADDR,MPU_USER_CTRL_REG,0X00);//I2C��ģʽ�ر�
//    MPU_Write_Byte(MPU6050_ADDR,MPU_FIFO_EN_REG,0X00);	//�ر�FIFO
//    MPU_Write_Byte(MPU6050_ADDR,MPU_INTBP_CFG_REG,0X80);//INT���ŵ͵�ƽ��Ч
    res += MPU_Write_Byte(MPU6050_ADDR,MPU_PWR_MGMT1_REG,0X01);  	//����CLKSEL,PLL X��Ϊ�ο�
    res += MPU_Write_Byte(MPU6050_ADDR,MPU_PWR_MGMT2_REG,0X00);  	//���ٶ��������Ƕ�����

    /*if(res == 0)  //����Ĵ�����д��ɹ�
    {
        printf("MPU set is OK!\n");
    }
    else return 1;*/

    return res;
}



//����MPU6050�����Ǵ����������̷�Χ
//fsr:0,��250dps;1,��500dps;2,��1000dps;3,��2000dps
//����ֵ:0,���óɹ�
//    ����,����ʧ��
uint8_t MPU_Set_Gyro_Fsr(uint8_t fsr)
{
	return MPU_Write_Byte(MPU6050_ADDR,MPU_GYRO_CFG_REG,fsr<<3);//���������������̷�Χ
}
//����MPU6050���ٶȴ����������̷�Χ
//fsr:0,��2g;1,��4g;2,��8g;3,��16g
//����ֵ:0,���óɹ�
//    ����,����ʧ��
uint8_t MPU_Set_Accel_Fsr(uint8_t fsr)
{
	return MPU_Write_Byte(MPU6050_ADDR,MPU_ACCEL_CFG_REG,fsr<<3);//���ü��ٶȴ����������̷�Χ
}

//����MPU6050�����ֵ�ͨ�˲���
//lpf:���ֵ�ͨ�˲�Ƶ��(Hz)
//����ֵ:0,���óɹ�
//    ����,����ʧ��
uint8_t MPU_Set_LPF(uint16_t lpf)
{
	uint8_t data=0;
	if(lpf>=188)data=1;
	else if(lpf>=98)data=2;
	else if(lpf>=42)data=3;
	else if(lpf>=20)data=4;
	else if(lpf>=10)data=5;
	else data=6;
	return MPU_Write_Byte(MPU6050_ADDR,MPU_CFG_REG,data);//�������ֵ�ͨ�˲���
}

//����MPU6050�Ĳ�����(�ٶ�Fs=1KHz)
//rate:4~1000(Hz)
//����ֵ:0,���óɹ�
//    ����,����ʧ��
uint8_t MPU_Set_Rate(uint16_t rate)
{
	uint8_t data;
	if(rate>1000)rate=1000;
	if(rate<4)rate=4;
	data=1000/rate-1;
	data=MPU_Write_Byte(MPU6050_ADDR,MPU_SAMPLE_RATE_REG,data);	//�������ֵ�ͨ�˲���
 	return MPU_Set_LPF(rate/2);	//�Զ�����LPFΪ�����ʵ�һ��
}

//�õ��¶�ֵ
//����ֵ:�¶�ֵ(������100��)
short MPU_Get_Temperature(void)
{
    uint8_t buf[2];
    short raw;
	float temp;
	MPU_Read_Len(MPU6050_ADDR,MPU_TEMP_OUTH_REG,2,buf);
    raw=((uint16_t)buf[0]<<8)|buf[1];
    temp=21+((double)raw)/333.87;
    return (short)temp*100;
}
//�õ�������ֵ(ԭʼֵ)
//gx,gy,gz:������x,y,z���ԭʼ����(������)
//����ֵ:0,�ɹ�
//    ����,�������
uint8_t MPU_Get_Gyroscope(short *gx,short *gy,short *gz)
{
    uint8_t buf[6],res;
	res=MPU_Read_Len(MPU6050_ADDR,MPU_GYRO_XOUTH_REG,6,buf);
	if(res==0)
	{
		*gx=((uint16_t)buf[0]<<8)|buf[1];
		*gy=((uint16_t)buf[2]<<8)|buf[3];
		*gz=((uint16_t)buf[4]<<8)|buf[5];
	}
    return res;
}
//�õ����ٶ�ֵ(ԭʼֵ)
//gx,gy,gz:������x,y,z���ԭʼ����(������)
//����ֵ:0,�ɹ�
//    ����,�������
uint8_t MPU_Get_Accelerometer(short *ax,short *ay,short *az)
{
    uint8_t buf[6],res;
	res=MPU_Read_Len(MPU6050_ADDR,MPU_ACCEL_XOUTH_REG,6,buf);
	if(res==0)
	{
		*ax=((uint16_t)buf[0]<<8)|buf[1];
		*ay=((uint16_t)buf[2]<<8)|buf[3];
		*az=((uint16_t)buf[4]<<8)|buf[5];
	}
    return res;
}

//�õ��Ӽ�ֵ���¶�ֵ�����ٶ�ֵ(ԭʼֵ)
//gx,gy,gz:������x,y,z���ԭʼ����(������)
//����ֵ:0,�ɹ�
//    ����,�������
uint8_t MPU_Get_Raw_data(short *ax,short *ay,short *az,short *gx,short *gy,short *gz)
{
    uint8_t buf[14],res;
	res=MPU_Read_Len(MPU6050_ADDR,MPU_ACCEL_XOUTH_REG,14,buf);
	if(res==0)
	{
		*ax=((uint16_t)buf[0]<<8)|buf[1];
		*ay=((uint16_t)buf[2]<<8)|buf[3];
		*az=((uint16_t)buf[4]<<8)|buf[5];
        *gx=((uint16_t)buf[8]<<8)|buf[9];
		*gy=((uint16_t)buf[10]<<8)|buf[11];
		*gz=((uint16_t)buf[12]<<8)|buf[13];
	}
    return res;
}


//IIC����д
//addr:������ַ
//reg:�Ĵ�����ַ
//len:д�볤��
//buf:������
//����ֵ:0,����
//    ����,�������
uint8_t MPU_Write_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{
    return IIC_WriteMultByteToSlave(addr, reg, len, buf);
}

//IIC������
//addr:������ַ
//reg:Ҫ��ȡ�ļĴ�����ַ
//len:Ҫ��ȡ�ĳ���
//buf:��ȡ�������ݴ洢��
//����ֵ:0,����
//    ����,�������
uint8_t MPU_Read_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{
    return IIC_ReadMultByteFromSlave(addr, reg, len, buf);
}


//IICдһ���ֽ�
//devaddr:����IIC��ַ
//reg:�Ĵ�����ַ
//data:����
//����ֵ:0,����
//    ����,�������
uint8_t MPU_Write_Byte(uint8_t addr,uint8_t reg,uint8_t value)
{
    return IIC_WriteByteToSlave(addr, reg, value);
}

//IIC��һ���ֽ�
//reg:�Ĵ�����ַ
//����ֵ:����������
uint8_t MPU_Read_Byte(uint8_t addr,uint8_t reg)
{
    uint8_t value;
    IIC_ReadByteFromSlave(addr, reg, &value);
    return value;
}

void get_MPU_error(void)
{
    uint16_t i=0;
    double temp=0;
    int16_t aacx,aacy,aacz;	//���ٶȴ�����ԭʼ����
    int16_t gyrox,gyroy,gyroz;	//������ԭʼ����
    for(i=0;i<700;i++)
    {
       MPU_Get_Raw_data(&aacx,&aacy,&aacz,&gyrox,&gyroy,&gyroz);	//�õ����ٶȴ���������
       temp+=2000.0*gyroz/32768;
    }
    MPU_error=temp/700;


}
void Fetch_MPU6050(void)
{
  int16_t aacx,aacy,aacz;	//���ٶȴ�����ԭʼ����
  int16_t gyrox,gyroy,gyroz;	//������ԭʼ����

   // temp=MPU_Get_Temperature();	//�õ��¶�ֵ
    MPU_Get_Raw_data(&aacx,&aacy,&aacz,&gyrox,&gyroy,&gyroz);	//�õ����ٶȴ���������
    var_MPU6050[0]=4*9.8*aacx/32768;
    var_MPU6050[1]=4*9.8*aacy/32768;
    var_MPU6050[2]=4*9.8*aacz/32768;
    var_MPU6050[3]=2000.0*gyrox/32768;
    var_MPU6050[4]=2000.0*gyroy/32768;
    var_MPU6050[5]=2000.0*gyroz/32768;
    vcan_sendware((uint8_t *)var_MPU6050, sizeof(var_MPU6050));
	
}
void cf_angle(float A_angle, float G_angleSpeed)
{
    const float CF_K = 0.5; // ���׻����˲�ϵ��
    const float DT =0.005;
    static float x1 = 0, y1 = 0, z1 = 0;
    float d_a = A_angle -  HBAngle;
    x1 = d_a * CF_K * CF_K;
    y1 = x1 * DT + y1;
    z1 = y1 + d_a * 2 * CF_K + G_angleSpeed;
    HBAngle = z1 * DT +  HBAngle;
}

