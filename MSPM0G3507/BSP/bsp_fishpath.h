#ifndef __BSP_FISHPATH_H
#define __BSP_FISHPATH_H

#include "ti_msp_dl_config.h"
#include "mid_xunji.h"

/* ================================================================
 * 循迹 BSP 层 — 封装循迹算法与 MG310 电机驱动
 *
 * 职责:
 *   1. 初始化循迹传感器 (UART1) 和电机 (TIMA1)
 *   2. 周期性调用 mid_xunji 计算差速占空比
 *   3. 将左右占空比写入 MG310 电机
 *   4. 提供启停 / 调速等高层接口
 *
 * 分层关系:
 *   app_fishpath_test.c           ← 应用测试
 *          ↓
 *   bsp_fishpath.c (本层)         ← BSP 层: 循迹 + 电机
 *       ↙        ↘
 *   mid_xunji   bsp_encoder      ← 中间层 / BSP
 *       ↓
 *   drv_uart                       ← 驱动层
 * ================================================================ */

/**
 * @brief  循迹系统初始化
 * @param  Kp / Ki / Kd   循迹 PID 参数
 * @param  base_speed     基础速度 (0~100%)
 *
 *         内部完成:
 *           - MG310 双电机初始化 (置停)
 *           - UART1 中断接收使能
 *           - 循迹 PID 初始化
 */
void fishpath_init(float Kp, float Ki, float Kd, uint32_t base_speed);

/**
 * @brief  循迹主更新函数 (每个控制周期调用一次)
 *
 *         流程: 采集传感器 → 计算偏差 → PID → 写入电机
 */
void fishpath_update(void);

/**
 * @brief  在线调整基础速度
 */
void fishpath_set_speed(uint32_t speed);

/**
 * @brief  循迹启停控制
 */
void fishpath_start(void);
void fishpath_stop(void);

#endif /* __BSP_FISHPATH_H */
