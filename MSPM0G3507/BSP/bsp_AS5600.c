#include "bsp_AS5600.h"
#include "drv_IIC.h"
#include "mid_delay.h"

static int g_as5600_dir = 1;   /* Direction: +1 or -1, applied to velocity */

void as5600_set_direction(int dir)
{
    g_as5600_dir = dir;
}

/* ================================================================
 *  底层 — I2C 寄存器读写
 * ================================================================ */

uint8_t as5600_read_reg(uint8_t reg_addr)
{
    uint8_t data = 0;
    drv_iic_read_byte(AS5600_I2C_ADDR, reg_addr, &data);
    return data;
}

void as5600_write_reg(uint8_t reg_addr, uint8_t data)
{
    drv_iic_write_byte(AS5600_I2C_ADDR, reg_addr, data);
}

/* ================================================================
 *  中层 — 原始数据获取
 * ================================================================ */

uint16_t as5600_read_raw_angle(void)
{
    uint8_t buf[2];
    drv_iic_read(AS5600_I2C_ADDR, AS5600_REG_RAW_ANGLE_H, buf, 2);
    /* RAW_ANGLE: buf[0] = angle[11:4], buf[1][7:4] = angle[3:0] */
    return ((uint16_t)buf[0] << 8) | buf[1];
}

/* ================================================================
 *  高层 — 角度计算
 * ================================================================ */

float as5600_get_angle_radians(void)
{
    uint16_t raw = as5600_read_raw_angle();
    return ((float)raw * AS5600_TWO_PI) / (float)AS5600_RESOLUTION;
}

float as5600_get_angle_multiturn(void)
{
    static int32_t  turns    = 0;
    static uint16_t last_raw = 0;
    static bool     first    = true;

    uint16_t raw = as5600_read_raw_angle();

    if (first) {
        last_raw = raw;
        first    = false;
        return 0.0f;
    }

    int16_t delta = (int16_t)(raw - last_raw);

    if (delta > 3276) {
        turns--;   /* 反转越圈 */
    } else if (delta < -3276) {
        turns++;   /* 正转越圈 */
    }

    last_raw = raw;

    return ((float)turns * AS5600_TWO_PI)
         + ((float)raw  * AS5600_TWO_PI / (float)AS5600_RESOLUTION);
}

/* ================================================================
 *  高层 — 速度计算
 *
 *  时间戳由 mid_delay 的 get_system_us() 提供。
 *  SysTick 在首次 delay_ms() 调用后自动保持运行。
 * ================================================================ */

float as5600_get_velocity(void)
{
    static int32_t  last_raw = 0;
    static uint32_t last_us  = 0;
    static bool     first    = true;

    int32_t  raw = (int32_t)as5600_read_raw_angle();
    uint32_t now = get_system_us();

    if (first) {
        last_raw = raw;
        last_us  = now;
        first    = false;
        return 0.0f;
    }

    int32_t dt_us = (int32_t)(now - last_us);
    if (dt_us < 10) {
        return 0.0f;
    }

    /* d_raw with 12-bit unwrap: if |delta| > 2048, shorter path crosses 0/4096 */
    int32_t d_raw = raw - last_raw;
    if (d_raw > 2048) {
        d_raw -= 4096;
    } else if (d_raw < -2048) {
        d_raw += 4096;
    }

    float dt_s = (float)dt_us / 1000000.0f;
    float vel  = (float)d_raw * AS5600_TWO_PI / (float)AS5600_RESOLUTION / dt_s;

    last_raw = raw;
    last_us  = now;

    return vel * (float)g_as5600_dir;
}

float as5600_get_speed(void)
{
    static float filtered = 0.0f;
    static bool  first    = true;

    float raw_vel = as5600_get_velocity();

    if (first) {
        filtered = raw_vel;
        first    = false;
    } else {
        filtered = 0.9f * filtered + 0.1f * raw_vel;
    }

    return filtered;
}
