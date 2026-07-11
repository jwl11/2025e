#include "ti_msp_dl_config.h"
#include "drv_tim.h"

void app_pwm_test()
{
    pwm_Init();

    pwm_setDuty(PWM_CH0,10);
    pwm_setDuty(PWM_CH1,50);
    pwm_setDuty(PWM_CH2,90);

    while(1)
    {
        
    }

    

}
