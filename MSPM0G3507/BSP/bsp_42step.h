#ifndef __BSP_42STEP_H
#define __BSP_42STEP_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Emm_V5.0 闭环步进驱动 — 串口控制 API
 *
 * 协议：地址 + 命令 + 数据 + 校验 (默认固定 0x6B)
 * 转速：0~5000 RPM
 * 适配 42 步进电机
 */

/* ================================================================
 * 类型定义
 * ================================================================ */

typedef enum {
    STEP42_OK = 0,
    STEP42_ERR_ARG,
    STEP42_ERR_UART,
} Step42Result;

typedef enum {
    STEP42_DIR_CW  = 0x00,   /* 顺时针 / 下降 */
    STEP42_DIR_CCW = 0x01,   /* 逆时针 / 上升 */
} Step42Dir;

typedef enum {
    STEP42_POS_RELATIVE = 0x00,
    STEP42_POS_ABSOLUTE = 0x01,
} Step42PosMode;

/* ================================================================
 * API
 * ================================================================ */

/** 初始化串口 (复用 ZDT_X35 UART2) */
void Step42_Init(void);

/** 查询固件版本: 01 1F 6B */
Step42Result Step42_QueryVersion(void);

/** 使能/失能电机 */
Step42Result Step42_Enable(bool enable);

/** 速度模式：以指定 RPM 持续运行, acc 0~255 */
Step42Result Step42_RunVelocity(Step42Dir dir, uint16_t rpm, uint8_t acc);

/** 位置模式：走指定脉冲数后停止 */
Step42Result Step42_MovePosition(Step42Dir dir, uint16_t rpm, uint8_t acc,
                                  uint32_t pulses, Step42PosMode mode);

/** 立即停止 */
Step42Result Step42_Stop(void);

#endif
