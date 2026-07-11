#include "ti_msp_dl_config.h"

/* 32 MHz CPU 时钟下，32 个周期 = 1 μs */
#define CPU_CYCLES_PER_US  32

/* SysTick 配置为 1ms 周期，仅用于毫秒级延时 */
void delay_ms(unsigned int ms)
{
    DL_SYSTICK_enable();
    while (ms--) {
        while (!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk));
    }
    DL_SYSTICK_disable();
}

/* 微秒级延时直接用硬件周期计数，不依赖 SysTick 中断 */
void delay_us(unsigned int us)
{
    DL_Common_delayCycles(us * CPU_CYCLES_PER_US);
}
