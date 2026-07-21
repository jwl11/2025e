#ifndef __BSP_ENCODER_H
#define __BSP_ENCODER_H

#include "ti_msp_dl_config.h"

/* ================================================================
 * MG310 双路霍尔编码电机驱动 (TIMG0)
 *
 * 引脚映射:
 *   ┌─────────┬──────────────┬──────────────┬──────────────┐
 *   │  电机   │   PWM (TIMG0)│    IN1       │    IN2       │
 *   ├─────────┼──────────────┼──────────────┼──────────────┤
 *   │   A     │ CH0, PA12    │ AIN1, PB17   │ AIN2, PB19   │
 *   │   B     │ CH1, PA13    │ BIN1, PA16   │ BIN2, PB24   │
 *   └─────────┴──────────────┴──────────────┴──────────────┘
 *
 * 方向真值表 (IN1 IN2):
 *   00 → 停止
 *   01 → 反转
 *   10 → 正转
 *   11 → 刹车
 * ================================================================ */

/* PWM 周期 (SysConfig 配置) */
#define MG310_PWM_PERIOD_MAX        3200U

/* 电机选择 */
typedef enum {
    MG310_MOTOR_A = 0,
    MG310_MOTOR_B = 1,
} MG310_Motor;

/* 电机方向 */
typedef enum {
    MG310_DIR_STOP    = 0x00,   /* IN1=0, IN2=0 → 停止 */
    MG310_DIR_REVERSE = 0x01,   /* IN1=0, IN2=1 → 反转 */
    MG310_DIR_FORWARD = 0x02,   /* IN1=1, IN2=0 → 正转 */
    MG310_DIR_BRAKE   = 0x03,   /* IN1=1, IN2=1 → 刹车 */
} MG310_Direction;

/* ---- 单路控制 ---- */

void mg310_motorInit(MG310_Motor motor);
void mg310_motorSetSpeed(MG310_Motor motor, uint32_t duty);
void mg310_motorSetDirection(MG310_Motor motor, MG310_Direction dir);
void mg310_motorForward(MG310_Motor motor, uint32_t duty);
void mg310_motorReverse(MG310_Motor motor, uint32_t duty);
void mg310_motorStop(MG310_Motor motor);
void mg310_motorBrake(MG310_Motor motor);

/* ---- 双路同步控制 ---- */

void mg310_motorInitAll(void);
void mg310_motorStopAll(void);

#endif /* __BSP_ENCODER_H */