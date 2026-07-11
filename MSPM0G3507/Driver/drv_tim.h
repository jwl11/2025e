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

#endif /* __DRV_TIM_H */
