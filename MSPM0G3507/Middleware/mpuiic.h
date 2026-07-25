#ifndef __MPUIIC_H
#define __MPUIIC_H
#include "ti_msp_dl_config.h"
#include "stdio.h"

// MPU6050 I2C 引脚操作宏 (MSPM0 DriverLib, 与 drv_IIC.c 风格一致)
#define MPU_W_SCL(x)    ((x) ? DL_GPIO_setPins(MPU6050_SCL_PORT, MPU6050_SCL_PIN) \
                             : DL_GPIO_clearPins(MPU6050_SCL_PORT, MPU6050_SCL_PIN))
#define MPU_W_SDA(x)    ((x) ? DL_GPIO_setPins(MPU6050_SDA_PORT, MPU6050_SDA_PIN) \
                             : DL_GPIO_clearPins(MPU6050_SDA_PORT, MPU6050_SDA_PIN))
#define MPU_R_SDA       DL_GPIO_readPins(MPU6050_SDA_PORT, MPU6050_SDA_PIN)

// SDA 方向切换 (推挽输出需切换方向实现双向 I2C)
#define MPU_SDA_IN()    DL_GPIO_disableOutput(MPU6050_SDA_PORT, MPU6050_SDA_PIN)
#define MPU_SDA_OUT()   DL_GPIO_enableOutput(MPU6050_SDA_PORT, MPU6050_SDA_PIN)

uint8_t MPU_Write_Len(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf);
uint8_t MPU_Read_Len(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf);

#endif
