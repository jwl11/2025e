#ifndef __MID_XUNJI_H
#define __MID_XUNJI_H

#include "ti_msp_dl_config.h"
#include "mid_pid.h"

/* ================================================================
 * 12路循迹模块 — 中间层
 *
 * 传感器数据采集 (UART1) → 加权偏差计算 → PID 控制 → 差速输出
 *
 * 加权算法:
 *   12 个传感器按位置赋权值 (-5500 ~ +5500, 中心为 0),
 *   加权平均得到线位置, 偏离中心即为 error。
 *
 * PID:
 *   使用独立的 PID_Controller 实例 g_pid_xunji,
 *   不占用 FOC 的 g_pid_angle / g_pid_speed。
 * ================================================================ */

#define XUNJI_SENSOR_COUNT  12

/* 循迹专用 PID 实例 */
extern PID_Controller g_pid_xunji;

/**
 * @brief  初始化循迹模块
 * @param  Kp / Ki / Kd    PID 参数
 * @param  base_speed       基础速度 (占空比 0~100)
 */
void xunji_init(float Kp, float Ki, float Kd, uint32_t base_speed);

/**
 * @brief  循迹主循环: 采集传感器 → 计算偏差 → PID → 差速输出
 *
 *         调用频率: 建议 50~200 Hz (由定时器或主循环周期调用)
 *
 *         内部完成:
 *           1. 读取 UART1 环形缓冲区
 *           2. 解析 12 路传感器数据
 *           3. 加权误差计算
 *           4. PID 控制
 *           5. 输出左右轮占空比 (0~100)
 *
 * @note   此函数只计算并存储左右占空比, 不直接控制电机。
 *         电机控制由 bsp_fishpath 层负责。
 */
void xunji_update(void);

/**
 * @brief  在线修改基础速度
 */
void xunji_set_base_speed(uint32_t speed);

/* ---- 查询接口 (调试 / 上层 BSP 使用) ---- */

int32_t xunji_get_left_duty(void);
int32_t xunji_get_right_duty(void);
int32_t xunji_get_error(void);
int32_t xunji_get_position(void);
uint8_t xunji_is_online(void);          /* 是否有传感器检测到黑线 */
void    xunji_get_sensors(uint8_t *dst); /* 复制传感器数组 (12 字节) */

#endif /* __MID_XUNJI_H */
