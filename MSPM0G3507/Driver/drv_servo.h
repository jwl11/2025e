#ifndef __DRV_SERVO_H
#define __DRV_SERVO_H

#include "ti_msp_dl_config.h"

/* ================================================================
 * 舵机 PWM 驱动 (TIMG8, PB21)
 *
 * PWM 参数 (SysConfig 已配置):
 *   时钟:  100 kHz (32MHz / 4 / 80)
 *   周期:  2000 counts → 20ms → 50Hz
 *   模式:  边沿对齐向上计数
 *
 * 脉宽 — 角度映射:
 *   0°   → 0.5ms → CC =  50
 *   90°  → 1.5ms → CC = 150
 *   180° → 2.5ms → CC = 250
 * ================================================================ */

#define SERVO_CC_MIN      50U    /* 0°   — 0.5ms */
#define SERVO_CC_MAX     250U    /* 180° — 2.5ms */
#define SERVO_CC_CENTER  150U    /* 90°  — 1.5ms */

void servo_init(void);
void servo_setCC(uint32_t cc_val);
void servo_setAngle(uint8_t angle);
void servo_setDutyPercent(uint8_t percent);

#endif /* __DRV_SERVO_H */
