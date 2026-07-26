#include "bsp_servo.h"

/* ================================================================
 * 舵机 BSP 层 — 实现
 *
 * 在 BSP 层调用 drv_servo 的底层函数驱动舵机。
 * ================================================================ */

/**
 * @brief  舵机初始化 (中位 90°)
 */
void bsp_servo_init(void)
{
    servo_init();
}

/**
 * @brief  设置舵机角度
 * @param  angle  0 ~ 180 (度)
 */
void bsp_servo_setAngle(uint8_t angle)
{
    servo_setAngle(angle);
}

/**
 * @brief  直接设置 CC 比较值
 * @param  cc_val  50 ~ 250
 */
void bsp_servo_setCC(uint32_t cc_val)
{
    servo_setCC(cc_val);
}
