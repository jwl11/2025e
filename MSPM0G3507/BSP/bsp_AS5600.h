#ifndef __BSP_AS5600_H
#define __BSP_AS5600_H

#include "ti_msp_dl_config.h"

/* ================================================================
 *  AS5600 常量定义
 * ================================================================ */

#define AS5600_I2C_ADDR           0x36    /* 7位 I2C 地址            */
#define AS5600_RESOLUTION         4096    /* 12位分辨率              */
#define AS5600_TWO_PI             6.283185307f  /* 2π                */

/* 寄存器 */
#define AS5600_REG_ZMCO           0x00
#define AS5600_REG_ZPOS_H         0x01
#define AS5600_REG_ZPOS_L         0x02
#define AS5600_REG_MPOS_H         0x03
#define AS5600_REG_MPOS_L         0x04
#define AS5600_REG_MANG_H         0x05
#define AS5600_REG_MANG_L         0x06
#define AS5600_REG_CONF_H         0x07
#define AS5600_REG_CONF_L         0x08
#define AS5600_REG_STATUS         0x0B
#define AS5600_REG_RAW_ANGLE_H    0x0C
#define AS5600_REG_RAW_ANGLE_L    0x0D
#define AS5600_REG_ANGLE_H        0x0E
#define AS5600_REG_ANGLE_L        0x0F
#define AS5600_REG_AGC            0x1A
#define AS5600_REG_MAGNITUDE_H    0x1B
#define AS5600_REG_MAGNITUDE_L    0x1C

/* STATUS 寄存器 bit */
#define AS5600_STATUS_MD          0x08    /* Magnet Detected     */
#define AS5600_STATUS_ML          0x10    /* Magnet too Weak     */
#define AS5600_STATUS_MH          0x20    /* Magnet too Strong   */

/* ================================================================
 *  底层 — I2C 寄存器读写
 * ================================================================ */

uint8_t as5600_read_reg(uint8_t reg_addr);
void    as5600_write_reg(uint8_t reg_addr, uint8_t data);

/* ================================================================
 *  中层 — 原始数据获取
 * ================================================================ */

uint16_t as5600_read_raw_angle(void);

/* ================================================================
 *  高层 — 角度 / 速度计算
 *
 *  时间基准由 mid_delay 的 get_system_us() 提供，
 *  SysTick 在首次 delay_ms() 调用后自动保持常开，
 *  无需额外初始化。
 * ================================================================ */

float as5600_get_angle_radians(void);
float as5600_get_angle_multiturn(void);
float as5600_get_velocity(void);
float as5600_get_speed(void);

#endif /* __BSP_AS5600_H */
