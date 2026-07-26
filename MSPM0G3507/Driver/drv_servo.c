#include "drv_servo.h"

/* ================================================================
 * 舵机 PWM 驱动 — 实现
 *
 * 底层直接操作 TIMG8 CC 寄存器控制脉宽。
 * SysConfig 已配置 TIMG8 的时钟/周期/启动, 此处只需设置 CC 值。
 * ================================================================ */

/**
 * @brief  舵机初始化
 * @note   SYSCFG_DL_servo_init() 已在 SYSCFG_DL_init() 中调用,
 *         此函数将 CC 值置为中心位置 (90°).
 */
void servo_init(void)
{
    /* 确保计数器在运行 */
    DL_TimerG_startCounter(servo_INST);

    /* 置为中位 (90°, 1.5ms) */
    DL_TimerG_setCaptureCompareValue(servo_INST,
                                      SERVO_CC_CENTER,
                                      GPIO_servo_C0_IDX);
}

/**
 * @brief  直接设置 CC 比较值 (控制脉宽)
 * @param  cc_val  比较值, 范围 SERVO_CC_MIN(50) ~ SERVO_CC_MAX(250)
 *                50  → 0.5ms → 0°
 *                150 → 1.5ms → 90°
 *                250 → 2.5ms → 180°
 */
void servo_setCC(uint32_t cc_val)
{
    /* 限幅 */
    if (cc_val < SERVO_CC_MIN) {
        cc_val = SERVO_CC_MIN;
    }
    if (cc_val > SERVO_CC_MAX) {
        cc_val = SERVO_CC_MAX;
    }

    DL_TimerG_setCaptureCompareValue(servo_INST,
                                      cc_val,
                                      GPIO_servo_C0_IDX);
}

/**
 * @brief  按角度设置舵机位置
 * @param  angle  0 ~ 180 (度)
 * @note   线性映射: CC = 50 + angle * 200 / 180
 */
void servo_setAngle(uint8_t angle)
{
    uint32_t cc_val;

    if (angle > 180) {
        angle = 180;
    }

    cc_val = SERVO_CC_MIN + ((uint32_t)angle * 200UL) / 180UL;

    servo_setCC(cc_val);
}

/**
 * @brief  按占空比百分比设置
 * @param  percent  0 ~ 100
 * @note   2.5% → 0°, 7.5% → 90°, 12.5% → 180°
 */
void servo_setDutyPercent(uint8_t percent)
{
    uint32_t cc_val;

    if (percent > 100) {
        percent = 100;
    }

    /* CC = percent * 2000 / 100, 然后限幅到 SERVO_CC_MIN ~ SERVO_CC_MAX */
    cc_val = ((uint32_t)percent * 2000UL) / 100UL;

    servo_setCC(cc_val);
}
