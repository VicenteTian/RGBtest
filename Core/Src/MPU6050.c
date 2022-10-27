#include "MPU6050.h"
#include "UART.h"
#include "u_iic.h"
float var_MPU6050[7]={0};                //存放要发送至上位机以波形显示数据的数组     陀螺仪
float MPU_error=0;                     //静态的偏差
float HBAngle=0;                       //互补滤波后的角度
uint8_t state=0;
uint8_t last_state=5;
uint8_t MPU6050_Init(void)
{
    uint8_t res;
    MPU6050_IIC_Init();     ////MPU6050 支持400K I2C
    res=MPU_Read_Byte(MPU6050_ADDR,WHO_AM_I);           //读取MPU6050的ID
//    if(res!=MPU6050_ID) //器件ID正确
//    {
//        printf("ID=%#X\r\n",res);
//        printf("MPU6050 is fail!\n");
//    }
//    else  printf("MPU6050 is OK!\n");

    res = 0;
    res += MPU_Write_Byte(MPU6050_ADDR,MPU_PWR_MGMT1_REG,0X80);//复位MPU6050
    HAL_Delay(100);  //延时100ms
    res += MPU_Write_Byte(MPU6050_ADDR,MPU_PWR_MGMT1_REG,0X00);//唤醒MPU6050
    res += MPU_Set_Gyro_Fsr(3);					        	//陀螺仪传感器,±2000dps
    res += MPU_Set_Accel_Fsr(1);					       	 	//加速度传感器,±4g
    res += MPU_Set_Rate(50);						       	 	//设置采样率50Hz
    res += MPU_Write_Byte(MPU6050_ADDR,MPU_CFG_REG,0x02);      //设置数字低通滤波器   98hz
    res += MPU_Write_Byte(MPU6050_ADDR,MPU_INT_EN_REG,0X00);   //关闭所有中断
    res += MPU_Write_Byte(MPU6050_ADDR,MPU_USER_CTRL_REG,0X00);//I2C主模式关闭
//    MPU_Write_Byte(MPU6050_ADDR,MPU_FIFO_EN_REG,0X00);	//关闭FIFO
//    MPU_Write_Byte(MPU6050_ADDR,MPU_INTBP_CFG_REG,0X80);//INT引脚低电平有效
    res += MPU_Write_Byte(MPU6050_ADDR,MPU_PWR_MGMT1_REG,0X01);  	//设置CLKSEL,PLL X轴为参考
    res += MPU_Write_Byte(MPU6050_ADDR,MPU_PWR_MGMT2_REG,0X00);  	//加速度与陀螺仪都工作

    /*if(res == 0)  //上面寄存器都写入成功
    {
        printf("MPU set is OK!\n");
    }
    else return 1;*/

    return res;
}



//设置MPU6050陀螺仪传感器满量程范围
//fsr:0,±250dps;1,±500dps;2,±1000dps;3,±2000dps
//返回值:0,设置成功
//    其他,设置失败
uint8_t MPU_Set_Gyro_Fsr(uint8_t fsr)
{
    return MPU_Write_Byte(MPU6050_ADDR,MPU_GYRO_CFG_REG,fsr<<3);//设置陀螺仪满量程范围
}
//设置MPU6050加速度传感器满量程范围
//fsr:0,±2g;1,±4g;2,±8g;3,±16g
//返回值:0,设置成功
//    其他,设置失败
uint8_t MPU_Set_Accel_Fsr(uint8_t fsr)
{
    return MPU_Write_Byte(MPU6050_ADDR,MPU_ACCEL_CFG_REG,fsr<<3);//设置加速度传感器满量程范围
}

//设置MPU6050的数字低通滤波器
//lpf:数字低通滤波频率(Hz)
//返回值:0,设置成功
//    其他,设置失败
uint8_t MPU_Set_LPF(uint16_t lpf)
{
    uint8_t data=0;
    if(lpf>=188)data=1;
    else if(lpf>=98)data=2;
    else if(lpf>=42)data=3;
    else if(lpf>=20)data=4;
    else if(lpf>=10)data=5;
    else data=6;
    return MPU_Write_Byte(MPU6050_ADDR,MPU_CFG_REG,data);//设置数字低通滤波器
}

//设置MPU6050的采样率(假定Fs=1KHz)
//rate:4~1000(Hz)
//返回值:0,设置成功
//    其他,设置失败
uint8_t MPU_Set_Rate(uint16_t rate)
{
    uint8_t data;
    if(rate>1000)rate=1000;
    if(rate<4)rate=4;
    data=1000/rate-1;
    data=MPU_Write_Byte(MPU6050_ADDR,MPU_SAMPLE_RATE_REG,data);	//设置数字低通滤波器
    return MPU_Set_LPF(rate/2);	//自动设置LPF为采样率的一半
}

//得到温度值
//返回值:温度值(扩大了100倍)
float MPU_Get_Temperature(void)
{
    uint8_t buf[2];
    short raw;
    MPU_Read_Len(MPU6050_ADDR,MPU_TEMP_OUTH_REG,2,buf);
    raw=((uint16_t)buf[0]<<8)|buf[1];
    return (((double)raw)/340.0+36.53);
}
//得到陀螺仪值(原始值)
//gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
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
//得到加速度值(原始值)
//gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
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

//得到加计值、温度值、角速度值(原始值)
//gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
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


//IIC连续写
//addr:器件地址
//reg:寄存器地址
//len:写入长度
//buf:数据区
//返回值:0,正常
//    其他,错误代码
uint8_t MPU_Write_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{
    return IIC_WriteMultByteToSlave(addr, reg, len, buf);
}

//IIC连续读
//addr:器件地址
//reg:要读取的寄存器地址
//len:要读取的长度
//buf:读取到的数据存储区
//返回值:0,正常
//    其他,错误代码
uint8_t MPU_Read_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{
    return IIC_ReadMultByteFromSlave(addr, reg, len, buf);
}


//IIC写一个字节
//devaddr:器件IIC地址
//reg:寄存器地址
//data:数据
//返回值:0,正常
//    其他,错误代码
uint8_t MPU_Write_Byte(uint8_t addr,uint8_t reg,uint8_t value)
{
    return IIC_WriteByteToSlave(addr, reg, value);
}

//IIC读一个字节
//reg:寄存器地址
//返回值:读到的数据
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
    int16_t aacx,aacy,aacz;	//加速度传感器原始数据
    int16_t gyrox,gyroy,gyroz;	//陀螺仪原始数据
    for(i=0;i<700;i++)
    {
        MPU_Get_Raw_data(&aacx,&aacy,&aacz,&gyrox,&gyroy,&gyroz);	//得到加速度传感器数据
        temp+=2000.0*gyroz/32768;
    }
    MPU_error=temp/700;


}
void Fetch_MPU6050(void)
{
    int16_t aacx,aacy,aacz;	//加速度传感器原始数据
    int16_t gyrox,gyroy,gyroz;	//陀螺仪原始数据
    MPU_Get_Raw_data(&aacx,&aacy,&aacz,&gyrox,&gyroy,&gyroz);	//得到加速度传感器数据
    var_MPU6050[0]=4*9.8*aacx/32768;
    var_MPU6050[1]=4*9.8*aacy/32768;
    var_MPU6050[2]=4*9.8*aacz/32768;
    var_MPU6050[3]=2000.0*gyrox/32768;
    var_MPU6050[4]=2000.0*gyroy/32768;
    var_MPU6050[5]=2000.0*gyroz/32768;
    var_MPU6050[6]=MPU_Get_Temperature();	//得到温度值;
    vcan_sendware((uint8_t *)var_MPU6050, sizeof(var_MPU6050));
}
void cf_angle(float A_angle, float G_angleSpeed)
{
    const float CF_K = 0.5; // 二阶互补滤波系数
    const float DT =0.005;
    static float x1 = 0, y1 = 0, z1 = 0;
    float d_a = A_angle -  HBAngle;
    x1 = d_a * CF_K * CF_K;
    y1 = x1 * DT + y1;
    z1 = y1 + d_a * 2 * CF_K + G_angleSpeed;
    HBAngle = z1 * DT +  HBAngle;
}

