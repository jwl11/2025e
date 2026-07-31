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
    STEP42_ERR_ACK,       /* 驱动器回应错误 (E2/EE) */
    STEP42_ERR_TIMEOUT,   /* 等待 ACK 超时 */
} Step42Result;

typedef enum {
    STEP42_DIR_CW  = 0x00,
    STEP42_DIR_CCW = 0x01,
} Step42Dir;

typedef enum {
    STEP42_POS_RELATIVE = 0x00,
    STEP42_POS_ABSOLUTE = 0x01,
} Step42PosMode;

/* 电机状态 */
typedef struct {
    bool  enabled;            /* 使能中 */
    bool  position_reached;   /* 到位 */
    bool  stalled;            /* 堵转 */
    bool  homing_done;        /* 回零完成 */
    int32_t position;         /* 实时位置 (脉冲) */
    int32_t position_error;   /* 位置误差 (脉冲) */
} Step42Status;

/* ================================================================
 * 基础驱动 API
 * ================================================================ */

void Step42_Init(void);
Step42Result Step42_QueryVersion(void);
Step42Result Step42_Enable(bool enable);
Step42Result Step42_RunVelocity(Step42Dir dir, uint16_t rpm, uint8_t acc);
Step42Result Step42_MovePosition(Step42Dir dir, uint16_t rpm, uint8_t acc,
                                  uint32_t pulses, Step42PosMode mode);
Step42Result Step42_Stop(void);

/* ================================================================
 * 闭环 & 状态 API
 * ================================================================ */

/** 切闭环模式 */
Step42Result Step42_SetClosedLoop(void);

/** 清当前位置为0 */
Step42Result Step42_ClearPosition(void);

/** 设当前位置为零点 */
Step42Result Step42_SetZero(void);

/** 触发回零 */
Step42Result Step42_GoHome(void);

/** 读取实时状态 (使能/到位/堵转/回零) */
Step42Result Step42_ReadStatus(Step42Status *st);

/** 读取实时位置 (脉冲) */
Step42Result Step42_ReadPosition(int32_t *pos);

/** 读取位置误差 (脉冲) */
Step42Result Step42_ReadPositionError(int32_t *err);

/** 堵转检测 (快捷函数) */
bool Step42_IsStalled(void);

/* ================================================================
 * 角度控制 API
 * ================================================================ */

/** 设脉冲/圈 (默认 3200→16细分) */
void Step42_SetPulsesPerRev(uint32_t ppr);

/** 角度→脉冲换算，绝对定位到指定角度 */
Step42Result Step42_MoveToAngle(float degrees, uint16_t rpm, uint8_t acc);

#endif
