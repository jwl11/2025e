#include "drv_tim.h"

/**
 * @brief  PWM初始化
 *         配置所有通道初始占空比为0，使能时钟并启动计数器
 */
void pwm_Init(void)
{
    /* 初始化三个通道占空比为0 */
    DL_TimerA_setCaptureCompareValue(BLDC_INST, 0, DL_TIMER_CC_0_INDEX);
    DL_TimerA_setCaptureCompareValue(BLDC_INST, 0, DL_TIMER_CC_1_INDEX);
    DL_TimerA_setCaptureCompareValue(BLDC_INST, 0, DL_TIMER_CC_2_INDEX);

    /* 使能定时器时钟 */
    DL_TimerA_enableClock(BLDC_INST);

    /* 启动定时器计数器 */
    DL_Timer_startCounter(BLDC_INST);

    pwm_start();
}

/**
 * @brief  设置指定通道的PWM占空比
 * @param  ch   通道号 (PWM_CH0 / PWM_CH1 / PWM_CH2)
 * @param  duty 占空比值，范围 0 ~ PWM_PERIOD
 */
void pwm_setDuty(uint32_t ch, uint32_t duty)
{
    uint32_t cmp_val;

    /* 限幅 */
    if (duty > 100) {
        duty = 100;
    }

    /*
     * 边沿对齐向下计数模式: CC值越大占空比越小，需取反。
     * 例: duty=0  → CC=PWM_PERIOD → 输出低电平 (~0%)
     *     duty=50 → CC=PWM_PERIOD/2 → 输出 ~50%
     *     duty=100 → CC=0 → 输出高电平 (~100%)
     */
    cmp_val = PWM_PERIOD - (duty * PWM_PERIOD + 50) / 100;

    DL_TimerA_setCaptureCompareValue(BLDC_INST, cmp_val, (DL_TIMER_CC_INDEX)ch);
}

/**
 * @brief  同时设置三个通道的PWM占空比
 * @param  duty0 通道0占空比值
 * @param  duty1 通道1占空比值
 * @param  duty2 通道2占空比值
 */
void pwm_setAllDuty(uint32_t duty0, uint32_t duty1, uint32_t duty2)
{
    pwm_setDuty(PWM_CH0, duty0);
    pwm_setDuty(PWM_CH1, duty1);
    pwm_setDuty(PWM_CH2, duty2);
}

/**
 * @brief  启动PWM输出 (启动定时器计数器)
 */
void pwm_start(void)
{
    DL_Timer_startCounter(BLDC_INST);
}

/**
 * @brief  停止PWM输出 (停止定时器计数器)
 */
void pwm_stop(void)
{
    DL_Timer_stopCounter(BLDC_INST);
}
