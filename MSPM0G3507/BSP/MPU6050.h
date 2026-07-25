#ifndef MPU6050_H
#define MPU6050_H
#include <stdint.h>
/*
bilibili 小努班 UID:437280309
@time时间: 2025.11.2
@version版本:V1_7
@Encoding :UTF-8
@attention:
1.注意杜邦线别用劣质的
2.如果打开INT要确保连接一定要好，否则mpu6050_Get_Angle_Plus会突然失控(这也是使用其的弊端)
3.注意如果ADD接了1，iic地址是0x69,接0为0x68
4.是否可以两个滤波器都打开?(额可以)
5.如果你使用杜邦线连接，如果线接触不良，MPU6050_Get_Angle_Plus可能会崩溃

@update:
1.更新MPU6050_SoftCalibrate_Z函数确保校准正确
2.强效减小mpu6050_Get_Angle_Plus中零点漂移
3.合并MPU6050_raw与MPU6050结构体
4.取消内嵌卡尔曼滤波器
5.新增PT2二阶低通滤波器吗，PT1低通滤波器
6.新增连续读取寄存器函数
7.可以使用MPU6050的INT引脚

if you can't display Chinese correctly,please check your encoding mode(please set encoding mode to UTF-8)
*/

#define MPU6050_USE_Filter 1 // 是否使用滤波器(如果你没有我的filter.h那么你设置成1也没有用会报错)
typedef struct MPU6050 {
    // 原始数据
    int16_t AccX;      // X轴加速度原始数据
    int16_t AccY;      // Y轴加速度原始数据
    int16_t AccZ;      // Z轴加速度原始数据
    int16_t GyroX;     // X轴陀螺仪原始数据
    int16_t GyroY;     // Y轴陀螺仪原始数据
    int16_t GyroZ;     // Z轴陀螺仪原始数据
    int16_t rawTemp;   // 温度传感器原始数据
    //返回的角度(欧拉角描述法)
    float yaw;       // 偏航角
    float roll;      // 翻滚角
    float pitch;     // 俯仰角
    float temp;      //实际温度值
    //四元素描述法
    float q0;
    float q1;
    float q2;
    float q3;
} MPU6050;

/////////////////////////常用函数//////////////////////////
void MPU6050_init();                                                // 初始化姿态传感器
void MPU6050_Get_Angle_Plus(MPU6050 *this);                         // 得到角度(Madgwick+自适应+无万向锁)
void MPU6050_Get_Angle(MPU6050 *this);                              // 得到角度(正常方法+卡尔曼滤波+互补滤波)
void MPU6050_Set_Angle0(MPU6050 *this);                             // 把角度设置为0(让上电的时候角度都从0开始,前提是让传感器采集数据稳定后)

/////////////////////////扩展函数//////////////////////////
uint8_t MPU6050_ID(void);             //读出ID(用于检测设备连接,返回255错误)
float MPU6050_GetTemp(MPU6050 *this); // 读取温度(返回值是读到的实际温度)

#endif
