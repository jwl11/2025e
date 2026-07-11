#include "ti_msp_dl_config.h"
#include "mid_delay.h"
#include "bsp_led.h"


void app_delay_test(void)
{

    use_led_ON();
    while (1) {
        
        delay_ms(1000);
        use_led_TOGGLE();


    }
}