#ifndef __DRV_IIC_H
#define __DRV_IIC_H

#include "ti_msp_dl_config.h"

/**
 * @brief  向 I2C 从机指定寄存器写入数据
 * @param  devAddr  7位从机地址
 * @param  reg      寄存器地址
 * @param  data     数据缓冲区指针
 * @param  len      写入字节数
 * @return true 成功, false 失败（NACK/错误）
 */
bool drv_iic_write(uint8_t devAddr, uint8_t reg, uint8_t *data, uint8_t len);

/**
 * @brief  向 I2C 从机指定寄存器写入单字节
 * @param  devAddr  7位从机地址
 * @param  reg      寄存器地址
 * @param  data     数据
 * @return true 成功, false 失败
 */
bool drv_iic_write_byte(uint8_t devAddr, uint8_t reg, uint8_t data);

/**
 * @brief  从 I2C 从机指定寄存器读取数据
 * @param  devAddr  7位从机地址
 * @param  reg      起始寄存器地址
 * @param  data     接收缓冲区指针
 * @param  len      读取字节数
 * @return true 成功, false 失败
 */
bool drv_iic_read(uint8_t devAddr, uint8_t reg, uint8_t *data, uint8_t len);

/**
 * @brief  从 I2C 从机指定寄存器读取单字节
 * @param  devAddr  7位从机地址
 * @param  reg      寄存器地址
 * @param  data     接收数据指针
 * @return true 成功, false 失败
 */
bool drv_iic_read_byte(uint8_t devAddr, uint8_t reg, uint8_t *data);
/*
bilibili 小努班 UID:437280309
@time:2025.9.29
@verson:V1_5
@updata: 
Using object oriented programming thinking. The fastest speed of software I2C has reached 390Khz, similar to hardware IIC fast mode 400k
1.使用面向对象程序设计思维,以及大量指针。软件I2C最快速度已经达到390Khz,与硬件IIC快速模式400k类似
2.支持创建多个iic对象
3.每个iic对象的延时时间可以单独设置

注意:调用函数的时候，问为什么引脚没有波形，因为这是开漏模式，你需要上拉电阻，
不然引脚电平无法发生变化，庆幸的是很多模块会自带上拉电阻,4.7K上拉速度比10K快可以适配高速IIC*/

/*//////////////////////使用示例////////////////////////////
1.先创建一个iic对象s
2.初始化iic对象
3.调用函数(&iic对象,xx,xx)

i2cbus_struct myI2C;//创建一个对象
void main(){
	MyI2C_Init(&myI2C,     //传入你的iic对象
        sclIO口, scl引脚,       //scl引脚配置
        sdaIO口, sda引脚,       //sda引脚配置
        设备地址, 10);
    uint8_t add;
    while(1){
        add = MYI2C_Add_Scan(&myI2C);       //iic扫描地址(可以用来识别设备是否正常)
        printf("%d\n",add);                 //输出add地址
    }
}
*/

typedef struct i2cbus_struct {

    uint16_t scl_pin;
    uint16_t sda_pin;
    uint8_t mode_16bit;    // 是否使用16位操作模式(默认为8bit)
    uint8_t address;       // iic地址(请传入没有左移的地址)
    uint16_t delay_time;   // 延时时间(当使用的是for循环的时候,延时时间可以设置为10)
} i2cbus_struct;

//////////////////常用函数////////////////////
void MyI2C_Init(i2cbus_struct* i2c_bus,uint8_t Address,uint16_t delay_time);//iic初始化(初始化函数里面已经左移地址)
void MYI2C_Write_Reg(i2cbus_struct* i2c_bus,uint8_t RegAddress, uint16_t Data);//iic写寄存器
uint16_t MYI2C_Read_Reg(i2cbus_struct* i2c_bus,uint8_t RegAddress);            //iic读寄存器
void MYI2C_Write_Reg_Continue(i2cbus_struct* i2c_bus, uint8_t RegAddress, uint8_t* data_array, uint16_t array_size);//iic连续写寄存器
void MYI2C_Read_Reg_Continue(i2cbus_struct* i2c_bus, uint8_t RegAddress, uint16_t read_len, uint8_t* data_buf);//iic连续读寄存器


//////////////////扩展函数////////////////////
uint8_t MYI2C_Add_Scan(i2cbus_struct* i2c_bus);//iic扫描地址(可以用来识别设备是否正常)
void MYI2C_Set_Mode(i2cbus_struct* i2c_bus,uint8_t status);//可以用来升级为16位寄存器操作

#endif // __DRV_IIC_H
