#ifndef __BSP_BLCD_H
#define __BSP_BLCD_H

#include "ti_msp_dl_config.h"

/* ================================================================
 *  BLDC 电机 BSP 层 — 封装 FOC 初始化 / 速度环 / 位置环
 *
 *  调用顺序: init → (set_speed 或 set_angle) → stop
 *  控制周期: 1kHz (每 1ms 调用一次 set_speed 或 set_angle)
 * ================================================================ */

/**
 * @brief  初始化 BLDC 电机驱动
 *
 *         内部完成: PWM 初始化、PID 参数加载、
 *         AS5600 零电角校准（3 秒，电机必须空载可自由转动）。
 *
 * @param  vbus  直流母线电压 (V)，例如 12.6f
 * @param  pp    电机极对数，例如 7
 * @param  dir   传感器极性 +1 或 -1
 *               开环测试时看 VEL 符号：
 *                 VEL 与预期同号 → +1
 *                 VEL 与预期反号 → -1
 */
void bsp_bldc_init(float vbus, int pp, int dir);

/**
 * @brief  速度闭环控制（支持正反转）
 *
 *         以 1kHz 频率调用。
 *         正值 = 正转，负值 = 反转。
 *
 * @param  target_rad_s  目标机械角速度 (rad/s)，可正可负
 */
void bsp_bldc_set_speed(float target_rad_s);

/**
 * @brief  位置闭环控制
 *
 *         以 1kHz 频率调用。
 *
 * @param  target_rad  目标机械角度 [0, 2π) (rad)
 */
void bsp_bldc_set_angle(float target_rad);

/**
 * @brief  获取当前机械角速度 (rad/s)，已滤波
 */
float bsp_bldc_get_speed(void);

/**
 * @brief  获取当前机械角度 (rad)，单圈 [0, 2π)
 */
float bsp_bldc_get_angle(void);

/**
 * @brief  停机 — 关闭 PWM 定时器，电机断电
 */
void bsp_bldc_stop(void);

#endif /* __BSP_BLCD_H */
