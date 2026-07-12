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

#endif // __DRV_IIC_H
