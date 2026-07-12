#ifndef MID_DELAY_H
#define MID_DELAY_H

#include <stdint.h>

void delay_us(unsigned int us);
void delay_ms(unsigned int ms);

/**
 * @brief  获取系统上电以来的毫秒计数值
 *
 *         依赖 SysTick 自由运行（delay_ms 首次调用后 SysTick 常开）。
 *         需以 >= 1kHz 频率轮询以避免丢失计数。
 *
 * @return 毫秒数
 */
uint32_t get_system_ms(void);

/**
 * @brief  获取系统上电以来的微秒计数值
 *
 *         精度约 ±30us（受限于 32MHz / 32000 的整数舍入）。
 *
 * @return 微秒数
 */
uint32_t get_system_us(void);

#endif
