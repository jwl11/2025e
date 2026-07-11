#include "ti_msp_dl_config.h"

volatile unsigned int delay_times = 0;

void delay_us(unsigned int us)
{
    DL_SYSTICK_enable();
    delay_times = us;
    while( delay_times != 0 );
    DL_SYSTICK_disable();
}

void delay_ms(unsigned int ms)
{
    delay_us(ms*1000);
}

//滴答定时器中断服务函数
void SysTick_Handler(void)
{
    if( delay_times != 0 )
    {
        delay_times--;
    }
}

