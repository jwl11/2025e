#ifndef __DRV_TIM_H
#define __DRV_TIM_H

#include "ti_msp_dl_config.h"

/* PWM period = timer count = 1066 */
#define PWM_PERIOD              1066
#define PWM_CH_MAX              3

/* PWM channel index mapping */
#define PWM_CH0                  DL_TIMER_CC_0_INDEX
#define PWM_CH1                  DL_TIMER_CC_1_INDEX
#define PWM_CH2                  DL_TIMER_CC_2_INDEX

void pwm_Init(void);
void pwm_setDuty(uint32_t ch, uint32_t duty);
void pwm_setAllDuty(uint32_t duty0, uint32_t duty1, uint32_t duty2);
void pwm_start(void);
void pwm_stop(void);

/* TIMG6 hardware timebase: 500 ms per interrupt. */
#define DRV_TIMEBASE_PERIOD_MS    500U

/** Start TIMG6 as a low-overhead stopwatch timebase. */
void drv_timebase_start(void);

/** Stop TIMG6 and disable its interrupt until the next start. */
void drv_timebase_stop(void);

/**
 * Return elapsed milliseconds since drv_timebase_start().
 * The current hardware counter is interpolated between 500 ms interrupts.
 */
uint32_t drv_timebase_get_ms(void);

/* Compatibility entry point retained for existing callers; IMU is not updated. */
void drv_mpu6050_timer_start(void);

#endif /* __DRV_TIM_H */
