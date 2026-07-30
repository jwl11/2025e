#include "drv_tim.h"

static volatile uint32_t g_timebase_ms = 0U;

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

void drv_timebase_start(void)
{
    /*
     * Reset the complete timebase before enabling the IRQ.  TIMG6 is
     * configured by SysConfig as PERIODIC_UP with a 500 ms load period.
     */
    NVIC_DisableIRQ(GET_MPU6050_INST_INT_IRQN);
    DL_Timer_stopCounter(GET_MPU6050_INST);
    DL_TimerG_clearInterruptStatus(
        GET_MPU6050_INST, DL_TIMERG_INTERRUPT_LOAD_EVENT);
    NVIC_ClearPendingIRQ(GET_MPU6050_INST_INT_IRQN);

    g_timebase_ms = 0U;
    DL_TimerG_setTimerCount(GET_MPU6050_INST, 0U);

    NVIC_EnableIRQ(GET_MPU6050_INST_INT_IRQN);
    DL_Timer_startCounter(GET_MPU6050_INST);
}

void drv_timebase_stop(void)
{
    DL_Timer_stopCounter(GET_MPU6050_INST);
    NVIC_DisableIRQ(GET_MPU6050_INST_INT_IRQN);
    DL_TimerG_clearInterruptStatus(
        GET_MPU6050_INST, DL_TIMERG_INTERRUPT_LOAD_EVENT);
    NVIC_ClearPendingIRQ(GET_MPU6050_INST_INT_IRQN);
}

uint32_t drv_timebase_get_ms(void)
{
    uint32_t base_ms;
    uint32_t count;
    uint32_t pending_before;
    uint32_t primask;

    /*
     * Only two interrupts per second are required.  Between interrupts,
     * convert the current PERIODIC_UP counter position into milliseconds.
     * Briefly masking interrupts gives a coherent base/count snapshot.
     */
    primask = __get_PRIMASK();
    __disable_irq();

    base_ms = g_timebase_ms;
    pending_before = DL_TimerG_getRawInterruptStatus(
        GET_MPU6050_INST, DL_TIMERG_INTERRUPT_LOAD_EVENT);
    count = DL_TimerG_getTimerCount(GET_MPU6050_INST);

    /*
     * If the timer wrapped between the first status read and the counter
     * read, take a fresh post-wrap count and include the pending period.
     */
    if ((pending_before == 0U) &&
        (DL_TimerG_getRawInterruptStatus(
             GET_MPU6050_INST, DL_TIMERG_INTERRUPT_LOAD_EVENT) != 0U)) {
        pending_before = DL_TIMERG_INTERRUPT_LOAD_EVENT;
        count = DL_TimerG_getTimerCount(GET_MPU6050_INST);
    }

    if (pending_before != 0U) {
        base_ms += DRV_TIMEBASE_PERIOD_MS;
    }

    if (primask == 0U) {
        __enable_irq();
    }

    return base_ms +
           ((count * DRV_TIMEBASE_PERIOD_MS) /
            (GET_MPU6050_INST_LOAD_VALUE + 1U));
}

void drv_mpu6050_timer_start(void)
{
    drv_timebase_start();
}

void TIMG6_IRQHandler(void)
{
    switch (DL_TimerG_getPendingInterrupt(GET_MPU6050_INST)) {
        case DL_TIMERG_IIDX_LOAD:
            DL_TimerG_clearInterruptStatus(
                GET_MPU6050_INST, DL_TIMERG_INTERRUPT_LOAD_EVENT);

            /* Two interrupts per second; ISR work is one integer addition. */
            g_timebase_ms += DRV_TIMEBASE_PERIOD_MS;
            break;
        default:
            break;
    }
}
