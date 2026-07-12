#include "ti_msp_dl_config.h"
#include "mid_delay.h"

/* 32 MHz CPU 时钟下，32 个周期 = 1 μs */
#define CPU_CYCLES_PER_US  32

/*
 * SysTick 被配置为 1ms 周期（DL_SYSTICK_init(32000)）。
 * 首次调用 delay_ms() 后 SysTick 保持常开，不再关闭，
 * 以便 get_system_ms() / get_system_us() 读取时间。
 */

static volatile uint32_t g_system_ms = 0;

/**
 * @brief  毫秒级延时
 *
 *         SysTick 首次启用后常开，不会在延时结束后关闭。
 */
void delay_ms(unsigned int ms)
{
    DL_SYSTICK_enable();

    while (ms--) {
        while (!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk));
        g_system_ms++;   /* 每过 1ms，同步更新全局计数器 */
    }
}

/**
 * @brief  微秒级延时（硬件周期计数，不依赖 SysTick）
 */
void delay_us(unsigned int us)
{
    DL_Common_delayCycles(us * CPU_CYCLES_PER_US);
}

/**
 * @brief  轮询 SysTick COUNTFLAG，维持全局毫秒计数器
 */
static void tick_poll(void)
{
    if (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) {
        g_system_ms++;
    }
}

/**
 * @brief  获取系统毫秒计数值
 */
uint32_t get_system_ms(void)
{
    tick_poll();
    return g_system_ms;
}

/**
 * @brief  获取系统微秒计数值
 *
 *         毫秒部分由 COUNTFLAG 轮询累积，
 *         亚毫秒部分由 SysTick->VAL 递减计数器换算。
 *         VAL 从 31999 递减到 0，每 32 个 tick ≈ 1us (32MHz)。
 */
uint32_t get_system_us(void)
{
    uint32_t ms, val, us_in_ms;

    tick_poll();
    ms = g_system_ms;

    /*
     * 先读 VAL 再确认毫秒没变，防止读到跨秒的旧 VAL：
     *   若 VAL 刚回绕（从 0→31999），ms 可能已递增但 VAL 还是新值，
     *   此时 (32000 - VAL) 极小，us_in_ms ≈ 0，结果正确。
     *   若 ms 在 VAL 读取期间递增，说明跨秒，用新 ms。
     */
    val = SysTick->VAL & 0x00FFFFFFUL;
    tick_poll();  /* 若此期间发生了回绕，ms 会 +1 */

    us_in_ms = (32000UL - val) / 32UL;
    return g_system_ms * 1000UL + us_in_ms;
}
