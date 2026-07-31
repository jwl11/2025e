#include "MPU6050.h"
#include "drv_IIC.h"
#include "math.h"
#include "stdlib.h"
#include "stdio.h"
#include "mid_delay.h" //主要装Delay_ms
#include "filter.h"  //里面主要使用了PT2滤波器和卡尔曼滤波器


/*
如果要移植的话不喜欢我的iic，可以换成你自己的只需把
MPU6050WRITE_REG等换成你自己的IIC写寄存器的函数即可然后删除掉"_MyI2C_.h"和i2cbus_struct mpu6050_i2cbus;
之后检查报错部分做对应处理,别忘记初始化
*/

i2cbus_struct mpu6050_i2cbus; // 创建了一个IIC对象
#define MPU6050_DELAY_MS(ms)                        delay_ms(ms)
#define MPU6050WRITE_REG(reg, data)                 MYI2C_Write_Reg(&mpu6050_i2cbus, reg, data)                // 可以替换为你的写寄存器函数
#define MPU6050READ_REG(reg)                        MYI2C_Read_Reg(&mpu6050_i2cbus, reg)                       // 可以替换为你的读寄存器函数
#define MPU6050_READ_REG_CONTINUE(reg, len, P_data) MYI2C_Read_Reg_Continue(&mpu6050_i2cbus, reg, len, P_data) // 可以替换为你的连续读写寄存器函数
//////////////////////////////////////以上为方便移植的接口///////////////////////////////////////////////////////

///////////////////////////下面是寄存器地址定义//////////////////////////////////
#define MPU6050_ADDRESS           0x68 // i2c address
#define MPU6050_SMPLRT_DIV        0x19
#define MPU6050_CONFIG            0x1A
#define MPU6050_GYRO_CONFIG       0x1B
#define MPU6050_ACCEL_CONFIG      0x1C
#define MPU6050_FIFO_EN           0x23

#define MPU6050_INTBP_CFG_REG     0X37
#define MPU6050_INT_ENABLE        0x38

#define MPU6050_ACCEL_XOUT_H      0x3B
#define MPU6050_ACCEL_XOUT_L      0x3C
#define MPU6050_ACCEL_YOUT_H      0x3D
#define MPU6050_ACCEL_YOUT_L      0x3E
#define MPU6050_ACCEL_ZOUT_H      0x3F
#define MPU6050_ACCEL_ZOUT_L      0x40
#define MPU6050_TEMP_OUT_H        0x41
#define MPU6050_TEMP_OUT_L        0x42
#define MPU6050_GYRO_XOUT_H       0x43
#define MPU6050_GYRO_XOUT_L       0x44
#define MPU6050_GYRO_YOUT_H       0x45
#define MPU6050_GYRO_YOUT_L       0x46
#define MPU6050_GYRO_ZOUT_H       0x47
#define MPU6050_GYRO_ZOUT_L       0x48
#define MPU6050_SIGNAL_PATH_RESET 0x68
// 校准地址不列出
#define MPU6050_USER_CTRL  0x6A
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_WHO_AM_I   0x75

//////////////////////////////////////////////////////////
static float mpu6050_dt; // 采样时间间隔(s)
static float angle_yaw   = 0;
static float angle_roll  = 0;
static float angle_pitch = 0;

static float gyroScale;  // 角速度量程250°/s→131，500°/s→65.5，1000°/s→32.8，2000°/s→16.4
static float accelScale; // 加速度计量程2g→16384，4g→8192，8g→4096，16g→2048
typedef enum {           // 传感器的滤波带宽
    Band_256Hz = 0x00,
    Band_186Hz,
    Band_96Hz,
    Band_43Hz,
    Band_21Hz,
    Band_10Hz,
    Band_5Hz
} Filter_Typedef;

typedef enum { // 传感器角速度测量范围
    gyro_250  = 0x00,
    gyro_500  = 0x08,
    gyro_1000 = 0x10,
    gyro_2000 = 0x18
} GYRO_CONFIG_Typedef;

typedef enum { // 传感器加速度测量范围
    acc_2g  = 0x00,
    acc_4g  = 0x08,
    acc_8g  = 0x10,
    acc_16g = 0x18
} ACCEL_CONFIG_Typedef;

typedef enum {
    FIFO_Disable, // 关闭FIFO
    Acc_OUT   = 0x08,
    Gyro_zOUT = 0x10,
    Gyro_yOUT = 0x20,
    Gyro_xOUT = 0x40,
    Temp_OUT  = 0x80,
} FIFO_EN_Typedef;

typedef enum {
    interrupt_Disable,        // 中断使能
    Data_Ready_EN     = 0x01, // 数据转换完成就触发(常用)
    I2C_Master_EN     = 0x08, // IIC主机模式
    FIFO_overFolow_EN = 0x10, // FIFO覆盖
    Motion_EN         = 0x40,
} INT_EN_Typedef;
///////////////////////////////////////////////////////////////////////////////////////////////////
typedef struct MPU6050_InitTypeDef {
    uint16_t SMPLRT_Rate;           // 采样率Hz
    Filter_Typedef Filter;          // 滤波器
    GYRO_CONFIG_Typedef gyro_range; // 陀螺仪测量范围
    ACCEL_CONFIG_Typedef acc_range; // 加速度计测量范围
    FIFO_EN_Typedef FIFO_EN;        // FIFO缓冲区使能
    INT_EN_Typedef INT;             // 中断使能
} MPU6050_InitTypeDef;
///////////////////////////////////////////////////////////////////////////////////////////////////
#if MPU6050_USE_Filter                                   // 如果你使用了PT2滤波器(二阶低通滤波器)
PT1Filter_t pt1_filter_x, pt1_filter_y, pt1_filter_z;    // 对加速度计的三个轴使用PT2低通滤波器
PT1Filter_t pt1_filter_gx, pt1_filter_gy, pt1_filter_gz; // 对角速度计的三个轴使用PT2低通滤波器
// KalmanFilter Kal_filter_x,Kal_filter_y,Kal_filter_z;
// KalmanFilter Kal_filter_gx,Kal_filter_gy,Kal_filter_gz;
#endif
// 传感器校准函数,减小零点漂移
static int16_t gyro_zero_x = 0, gyro_zero_y = 0, gyro_zero_z = 0; // 软件校准专用
//-1:校准失败,0校准成功
static int8_t MPU6050_SoftCalibrate_Z(uint16_t calibration_samples)
{
    int32_t gx_sum = 0, gy_sum = 0, gz_sum = 0;
    int16_t GyroX, GyroY, GyroZ;
    uint8_t temp_buffer[6];

    ///////////根据X,Y,Z轴的变换规律进行修正/////////
    for (uint16_t i = 0; i < calibration_samples; i++) {
        MPU6050_READ_REG_CONTINUE(MPU6050_GYRO_XOUT_H, 6, temp_buffer);
        GyroX = ((int16_t)(temp_buffer[0] << 8) | temp_buffer[1]);
        GyroY = ((int16_t)(temp_buffer[2] << 8) | temp_buffer[3]);
        GyroZ = ((int16_t)(temp_buffer[4] << 8) | temp_buffer[5]);

        // 以上为连续读取寄存器，以下为单个读取寄存器(慢)

        // GyroX = ((int16_t)(MPU6050READ_REG(MPU6050_GYRO_XOUT_H)) << 8) | MPU6050READ_REG(MPU6050_GYRO_XOUT_L);
        // GyroY = ((int16_t)(MPU6050READ_REG(MPU6050_GYRO_YOUT_H)) << 8) | MPU6050READ_REG(MPU6050_GYRO_YOUT_L);
        // GyroZ = ((int16_t)(MPU6050READ_REG(MPU6050_GYRO_ZOUT_H)) << 8) | MPU6050READ_REG(MPU6050_GYRO_ZOUT_L);
        // printf("%d,%d,%d\n",GyroX,GyroY,GyroZ);// 通过串口打印数据(用于检查驱动是否正常)
        gx_sum += GyroX;
        gy_sum += GyroY;
        gz_sum += GyroZ;
        MPU6050_DELAY_MS(mpu6050_dt * 1000); // 要和dt同步
    }
    gyro_zero_x = gx_sum / calibration_samples;
    gyro_zero_y = gy_sum / calibration_samples;
    gyro_zero_z = gz_sum / calibration_samples;

    // 检验检验校准是否成功
    gz_sum = gy_sum = gx_sum = 0;
    for (uint16_t i = 0; i < 100; i++) {
        MPU6050_READ_REG_CONTINUE(MPU6050_GYRO_XOUT_H, 6, temp_buffer);
        GyroX = ((int16_t)(temp_buffer[0] << 8) | temp_buffer[1]);
        GyroY = ((int16_t)(temp_buffer[2] << 8) | temp_buffer[3]);
        GyroZ = ((int16_t)(temp_buffer[4] << 8) | temp_buffer[5]);
        GyroX = GyroX - gyro_zero_x;
        GyroY = GyroY - gyro_zero_y;
        GyroZ = GyroZ - gyro_zero_z;

        // 以上为连续读取寄存器，以下为单个读取寄存器(慢)

        // GyroX = ((int16_t)(MPU6050READ_REG(MPU6050_GYRO_XOUT_H)) << 8) | MPU6050READ_REG(MPU6050_GYRO_XOUT_L);
        // GyroY = ((int16_t)(MPU6050READ_REG(MPU6050_GYRO_YOUT_H)) << 8) | MPU6050READ_REG(MPU6050_GYRO_YOUT_L);
        // GyroZ = ((int16_t)(MPU6050READ_REG(MPU6050_GYRO_ZOUT_H)) << 8) | MPU6050READ_REG(MPU6050_GYRO_ZOUT_L);
        // printf("%d,%d,%d\n",GyroX,GyroY,GyroZ);// 通过串口打印数据(用于检查驱动是否正常)
        gx_sum += GyroX;
        gy_sum += GyroY;
        gz_sum += GyroZ;
        MPU6050_DELAY_MS(mpu6050_dt * 1000); // 要和dt同步
    }
    if (abs(gx_sum / 100) >= 2 && abs(gy_sum / 100) >= 2 && abs(gz_sum / 100) >= 2) {
        return -1; // 校准失败
    }
    // printf("\n%d,%d,%d\n",gyro_zero_x,gyro_zero_y,gyro_zero_z);// 通过串口打印数据(用于检查驱动是否正常)
    // MPU6050_DELAY_MS(1000);//(用于检查驱动是否正常)
    return 0; // 校准成功
}
static void MPU6050_Register_init(MPU6050_InitTypeDef *this)
{
    float temp_acc_scale;  // 比例因子(加速度)
    float temp_groy_scale; // 比例因子(角速度)
    // 角速度量程250°/s→131，500°/s→65.5，1000°/s→32.8，2000°/s→16.4
    //  加速度计量程2g→16384，4g→8192，8g→4096，16g→2048
    switch (this->acc_range) {
        case acc_2g:
            temp_acc_scale = 16384;
            break;
        case acc_4g:
            temp_acc_scale = 8192;
            break;
        case acc_8g:
            temp_acc_scale = 4096;
            break;
        case acc_16g:
            temp_acc_scale = 2048;
            break;
    }
    switch (this->gyro_range) {
        case gyro_250:
            temp_groy_scale = 131;
            break;
        case gyro_500:
            temp_groy_scale = 65.5;
            break;
        case gyro_1000:
            temp_groy_scale = 32.8;
            break;
        case gyro_2000:
            temp_groy_scale = 16.4;
            break;
    }
    accelScale = 1.f / temp_acc_scale;
    gyroScale  = 0.0174533f / temp_groy_scale;
    MPU6050WRITE_REG(MPU6050_PWR_MGMT_1, 0x80); // 复位
    MPU6050_DELAY_MS(100);
    MPU6050WRITE_REG(MPU6050_PWR_MGMT_1, 0x00); // 唤醒
    uint8_t SMPLRT_DIV;
    if (this->SMPLRT_Rate >= 1000)
        this->SMPLRT_Rate = 1000;
    else if (this->SMPLRT_Rate < 4)
        this->SMPLRT_Rate = 4;
    SMPLRT_DIV = 1000.0f / this->SMPLRT_Rate - 1; // 由计算公式得
    MPU6050WRITE_REG(MPU6050_SMPLRT_DIV, SMPLRT_DIV);
    MPU6050WRITE_REG(MPU6050_INTBP_CFG_REG, 0x00);
    MPU6050WRITE_REG(MPU6050_INT_ENABLE, this->INT);
    MPU6050WRITE_REG(MPU6050_CONFIG, this->Filter);
    MPU6050WRITE_REG(MPU6050_GYRO_CONFIG, this->gyro_range);
    MPU6050WRITE_REG(MPU6050_ACCEL_CONFIG, this->acc_range);
    MPU6050WRITE_REG(MPU6050_FIFO_EN, this->FIFO_EN);
    uint8_t temp = 0x00;
    if (this->FIFO_EN != 0x00) // 如果打开了FIFO
        temp = 0x40;
    if ((this->INT & 0x01) == 0) // 如果打开了中断
        temp |= 0x08;
    MPU6050WRITE_REG(MPU6050_USER_CTRL, temp);
    MPU6050WRITE_REG(MPU6050_PWR_MGMT_1, 0x01); // X轴为参考
}
static void MPU6050_Get_Raw(MPU6050 *this)
{
    // 单个读取寄存器代码(慢)!!!使用了这里连续读取寄存器代码就注释掉
    // this->AccX    = ((int16_t)(MPU6050READ_REG(MPU6050_ACCEL_XOUT_H)) << 8) | MPU6050READ_REG(MPU6050_ACCEL_XOUT_L);
    // this->AccY    = ((int16_t)(MPU6050READ_REG(MPU6050_ACCEL_YOUT_H)) << 8) | MPU6050READ_REG(MPU6050_ACCEL_YOUT_L);
    // this->AccZ    = ((int16_t)(MPU6050READ_REG(MPU6050_ACCEL_ZOUT_H)) << 8) | MPU6050READ_REG(MPU6050_ACCEL_ZOUT_L);
    // this->rawTemp = ((uint16_t)(MPU6050READ_REG(MPU6050_TEMP_OUT_H)) << 8) | MPU6050READ_REG(MPU6050_TEMP_OUT_L);
    // this->GyroX   = ((int16_t)(MPU6050READ_REG(MPU6050_GYRO_XOUT_H)) << 8) | MPU6050READ_REG(MPU6050_GYRO_XOUT_L);
    // this->GyroY   = ((int16_t)(MPU6050READ_REG(MPU6050_GYRO_YOUT_H)) << 8) | MPU6050READ_REG(MPU6050_GYRO_YOUT_L);
    // this->GyroZ   = ((int16_t)(MPU6050READ_REG(MPU6050_GYRO_ZOUT_H)) << 8) | MPU6050READ_REG(MPU6050_GYRO_ZOUT_L);

    // 连续读取寄存器代码(快)!!!使用了这里单个读取寄存器代码就注释掉
    static uint8_t temp_buffer[14];
    MPU6050_READ_REG_CONTINUE(MPU6050_ACCEL_XOUT_H, 14, temp_buffer);
    this->AccX    = ((int16_t)temp_buffer[0] << 8) | temp_buffer[1];
    this->AccY    = ((int16_t)temp_buffer[2] << 8) | temp_buffer[3];
    this->AccZ    = ((int16_t)temp_buffer[4] << 8) | temp_buffer[5];
    this->rawTemp = ((int16_t)temp_buffer[6] << 8) | temp_buffer[7];
    this->GyroX   = ((int16_t)temp_buffer[8] << 8) | temp_buffer[9];
    this->GyroY   = ((int16_t)temp_buffer[10] << 8) | temp_buffer[11];
    this->GyroZ   = ((int16_t)temp_buffer[12] << 8) | temp_buffer[13];

    // 不要注释这里
    this->GyroX = this->GyroX - gyro_zero_x; // 软件校准专用(抑制效果极好)
    this->GyroY = this->GyroY - gyro_zero_y; // 软件校准专用(抑制效果极好)
    this->GyroZ = this->GyroZ - gyro_zero_z; // 软件校准专用(抑制效果极好)
#if MPU6050_USE_Filter                       // 如果使用了滤波器
    // 滤波器进行滤波
    this->AccX  = (int16_t)PT1Filter_Apply(&pt1_filter_x, this->AccX);
    this->AccY  = (int16_t)PT1Filter_Apply(&pt1_filter_y, this->AccY);
    this->AccZ  = (int16_t)PT1Filter_Apply(&pt1_filter_z, this->AccZ);
    this->GyroX = (int16_t)PT1Filter_Apply(&pt1_filter_gx, this->GyroX);
    this->GyroY = (int16_t)PT1Filter_Apply(&pt1_filter_gy, this->GyroY);
    this->GyroZ = (int16_t)PT1Filter_Apply(&pt1_filter_gz, this->GyroZ);

    // this->AccX  = (int16_t)KalmanFilter_Update(&Kal_filter_x, this->AccX);
    // this->AccY  = (int16_t)KalmanFilter_Update(&Kal_filter_y, this->AccY);
    // this->AccZ  = (int16_t)KalmanFilter_Update(&Kal_filter_z, this->AccZ);
    // this->GyroX = (int16_t)KalmanFilter_Update(&Kal_filter_gx, this->GyroX);
    // this->GyroY = (int16_t)KalmanFilter_Update(&Kal_filter_gy, this->GyroY);
    // this->GyroZ = (int16_t)KalmanFilter_Update(&Kal_filter_gz, this->GyroZ);
#endif
    // printf("%d,%d,%d\n",this->AccX,this->AccY,this->AccZ);// 通过串口打印数据(用于检查Acc滤波器是否正常)
    // printf("%d,%d,%d\n",this->GyroX,this->GyroY,this->GyroZ);// 通过串口打印数据(用于检查Groy滤波器是否正常)
}

// 快速平方根倒数算法
static inline float invSqrt(float x)
{
    float halfx = 0.5f * x;
    float y     = x;
    long i      = *(long *)&y;
    i           = 0x5f3759df - (i >> 1);
    y           = *(float *)&i;
    y           = y * (1.5f - (halfx * y * y)); // 一次牛顿迭代
    return y;
}
///////////////////////////////////////////以下为用户可以访问的代码/////////////////////////////////////////////////////

/*例子:MPU6050_init(GPIOA,GPIO_Pin_1,GPIO_Pin_2);
注意:SCL和SDA来自同一组GPIO口(notice:SCL and SDA come from a same GPIO port)*/
void MPU6050_init()
{
    // 软件iic初始化(支持两个不同的GPIO，但是你要自己改MPU6050_init函数的形参)
    MyI2C_Init(&mpu6050_i2cbus,MPU6050_ADDRESS, 5); // iic初始化(初始化函数里面已经左移地址),延时时间

    MPU6050_InitTypeDef MPU6050_init_Struct;
    MPU6050_init_Struct.SMPLRT_Rate = 200;           // 采样率Hz
    MPU6050_init_Struct.Filter      = Band_21Hz;     // 滤波器带宽
    MPU6050_init_Struct.gyro_range  = gyro_2000;     // 陀螺仪测量范围
    MPU6050_init_Struct.acc_range   = acc_4g;        // 加速度计测量范围
    MPU6050_init_Struct.FIFO_EN     = FIFO_Disable;  // FIFO
    MPU6050_init_Struct.INT         = Data_Ready_EN; // 中断配置(中断输出为,上升沿触发,电平为高)
    MPU6050_Register_init(&MPU6050_init_Struct);     // 初始化寄存器
    mpu6050_dt = 1.f / (float)MPU6050_init_Struct.SMPLRT_Rate;
    MPU6050_DELAY_MS(360);      // 开机后等待寄存器正常
    for (int i = 0; i < 3; i++) // 软件校准,减少yaw的零点漂移
        if (MPU6050_SoftCalibrate_Z(200) == 0)
            break; // 校准3次,如果校准成功就退出

#if MPU6050_USE_Filter // 如果你使用了滤波器(如果你没有我的filter文件那么你将会报错,该滤波器效果较好使用在无人机上)
    // 初始化滤波器
    PT1Filter_InitWithFreq(&pt1_filter_x, 48, MPU6050_init_Struct.SMPLRT_Rate);
    PT1Filter_InitWithFreq(&pt1_filter_y, 48, MPU6050_init_Struct.SMPLRT_Rate);
    PT1Filter_InitWithFreq(&pt1_filter_z, 48, MPU6050_init_Struct.SMPLRT_Rate);
    PT1Filter_InitWithFreq(&pt1_filter_gx, 88, MPU6050_init_Struct.SMPLRT_Rate);
    PT1Filter_InitWithFreq(&pt1_filter_gy, 88, MPU6050_init_Struct.SMPLRT_Rate);
    PT1Filter_InitWithFreq(&pt1_filter_gz, 88, MPU6050_init_Struct.SMPLRT_Rate);
    // //以下为卡尔曼
    // KalmanFilter_Init(&Kal_filter_x,1,10,0,0);
    // KalmanFilter_Init(&Kal_filter_y,1,10,0,0);
    // KalmanFilter_Init(&Kal_filter_z,1,10,0,0);
    // KalmanFilter_Init(&Kal_filter_gx,1,0.1,0,0);
    // KalmanFilter_Init(&Kal_filter_gy,1,0.1,0,0);
    // KalmanFilter_Init(&Kal_filter_gz,1,0.1,0,0);
#endif
}

float MPU6050_GetTemp(MPU6050 *this)
{ // 返回温度
    this->temp = (float)this->rawTemp / 340 + 36.53;
    return this->temp;
}

float ACC_abs = 0;
void MPU6050_Get_Angle(MPU6050 *this)
{
    float Ax, Ay, Az;
    float Gx, Gy, Gz;
    MPU6050_Get_Raw(this);
    // 读取加速度计数据
    Ax = this->AccX * accelScale; // 假设加速度计量程为±2g
    Ay = this->AccY * accelScale;
    Az = this->AccZ * accelScale;
    // 读取陀螺仪数据
    Gx = this->GyroX * gyroScale * mpu6050_dt;
    Gy = this->GyroY * gyroScale * mpu6050_dt;
    Gz = this->GyroZ * gyroScale * mpu6050_dt;

    // 计算加速度的绝对值
    float absAcc = sqrt(Ax * Ax + Ay * Ay + Az * Az);
    // ACC_abs = absAcc;
    //  动态调整权重
    float weight;
    if (absAcc > 1.2) {
        // 快速运动或剧烈振动状态，减小加速度计权重
        weight = 0.8f;
    } else {
        // 正常运动状态，强烈信任加速度计
        weight = 0.98f;
    }

    static float Gyroscope_roll  = 0.0f;
    static float Gyroscope_pitch = 0.0f;
    Gyroscope_roll += Gy;
    Gyroscope_pitch += Gx;
    this->roll  = weight * atan2(Ay, Az) / 3.1415926f * 180.f + (1 - weight) * Gyroscope_roll;
    this->pitch = -(weight * atan2(Ax, Az) / 3.1415926f * 180.f + (1 - weight) * Gyroscope_pitch);
    this->yaw += Gz * 57.2958f;
    // printf("%.3f,%.3f,%.3f,%d\n", this->roll, this->pitch, this->yaw,temp); // 通过串口打印数据(用于检查驱动是否正常)
}

// 四元素法+动态互补滤波
void MPU6050_Get_Angle_Plus(MPU6050 *this)
{
    static uint16_t times = 0;
    static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
    static float Kp, Ki;
    static float integralX = 0.0f, integralY = 0.0f, integralZ = 0.0f;

    // 读取原始数据
    MPU6050_Get_Raw(this);

    // 转换为物理量
    float ax = (float)this->AccX * accelScale;
    float ay = (float)this->AccY * accelScale;
    float az = (float)this->AccZ * accelScale;
    float gx = (float)this->GyroX * gyroScale;
    float gy = (float)this->GyroY * gyroScale;
    float gz = (float)this->GyroZ * gyroScale;

    // 加速度幅值（用于自适应增益）
    float accMag = ax * ax + ay * ay + az * az;

    // 初始化阶段参数
    if (times < 400) {
        times++;
        Kp = 8.f;
        Ki = 0.002f;
    } else {
        // 动态调整增益
        Kp = (accMag > 1.44f || accMag < 0.64f) ? 3.6f : 4.8f;
        Ki = (accMag > 1.44f || accMag < 0.64f) ? 0.001f : 0.0015f;
    }

    // 加速度计有效时校正姿态
    if (accMag > 0.01f) {
        float recipNorm = invSqrt(accMag);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // 计算重力方向误差
        float vx = 2.0f * (q1 * q3 - q0 * q2);
        float vy = 2.0f * (q0 * q1 + q2 * q3);
        float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

        float ex = ay * vz - az * vy;
        float ey = az * vx - ax * vz;
        float ez = ax * vy - ay * vx;

        // 积分项校正
        if (Ki > 0.0f) {
            integralX += ex * mpu6050_dt;
            integralY += ey * mpu6050_dt;
            integralZ += ez * mpu6050_dt;
            gx += Ki * integralX;
            gy += Ki * integralY;
            gz += Ki * integralZ;
        }

        // 比例项校正
        gx += Kp * ex;
        gy += Kp * ey;
        gz += Kp * ez;
    }

    // 四元数更新（核心计算）
    float qDot0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    float qDot1 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    float qDot2 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    float qDot3 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    q0 += qDot0 * mpu6050_dt;
    q1 += qDot1 * mpu6050_dt;
    q2 += qDot2 * mpu6050_dt;
    q3 += qDot3 * mpu6050_dt;

    // 四元数归一化
    float norm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= norm;
    q1 *= norm;
    q2 *= norm;
    q3 *= norm;

    this->q0 = q0;
    this->q1 = q1;
    this->q2 = q2;
    this->q3 = q3;

    // 计算欧拉角（四元数转欧拉角核心）
    this->roll        = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * 57.29578f;
    this->pitch       = asinf(2.0f * (q0 * q2 - q3 * q1)) * 57.29578f;
    float current_yaw = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3)) * 57.29578f;

    // 最快的yaw角解包裹处理
    static float unwrapped_yaw = 0.0f;
    static uint8_t first_run   = 1;

    if (first_run) {
        unwrapped_yaw = current_yaw; // 初始化
        first_run     = 0;
    } else {
        // 仅用一次差值判断跳变，无累计计算
        float diff = current_yaw - unwrapped_yaw;
        if (diff > 180.0f) {
            unwrapped_yaw += diff - 360.0f; // 正向跳变修正
        } else if (diff < -180.0f) {
            unwrapped_yaw += diff + 360.0f; // 负向跳变修正
        } else {
            unwrapped_yaw = current_yaw; // 无跳变直接更新
        }
    }

    // 应用校准
    this->roll -= angle_roll;
    this->pitch -= angle_pitch;
    this->yaw = unwrapped_yaw - angle_yaw; // 输出连续yaw角
}

void MPU6050_Set_Angle0(MPU6050 *this)
{
    // 记录下当前角度
    for (uint32_t i = 0; i < 1000000; i++);
    angle_yaw   = this->yaw;
    angle_roll  = this->roll;
    angle_pitch = this->pitch;
}

uint8_t MPU6050_ID()
{
    return MPU6050READ_REG(MPU6050_WHO_AM_I);
}

/**
 * @brief  读取加速度计三轴数据 (单位: g, 带 PT1 滤波)
 */
void MPU6050_Get_Accel(float *ax, float *ay, float *az)
{
    MPU6050 mpu;
    MPU6050_Get_Raw(&mpu);
    *ax = (float)mpu.AccX * accelScale;
    *ay = (float)mpu.AccY * accelScale;
    *az = (float)mpu.AccZ * accelScale;
}
