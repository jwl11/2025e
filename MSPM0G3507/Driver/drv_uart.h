#ifndef __DRV_UART_H
#define __DRV_UART_H

#include "ti_msp_dl_config.h"

/* ================================================================
 * UART0 (debug) — 阻塞发送 / printf 重定向
 * ================================================================ */

/** UART0 debug output: debug_INST, TX=PA10, RX=PA11, 115200-8N1. */
void drv_uart0_init(void);
void drv_uart_send_string(const char *str);
void drv_uart_print_num(unsigned long num);
void drv_uart_print_signed(long num);
void drv_uart_print_hex(uint8_t num);

/* ================================================================
 * UART1 (fishpath) — 12路循迹模块 中断接收
 * ================================================================ */

void    drv_uart1_init(void);
uint16_t drv_uart1_available(void);
int16_t drv_uart1_read(void);       /* returns byte 0~255, or -1 if empty */
void    drv_uart1_flush(void);

/* ================================================================
 * UART3 (F32C) - 二自由度云台阻塞发送
 *
 * UART3 的引脚、波特率和实例均使用 ti_msp_dl_config.h 中由
 * SysConfig 生成的 f32c_* 宏定义。
 * ================================================================ */

/** UART3 硬件已由 SYSCFG_DL_init() 初始化，此函数等待发送器空闲。 */
void drv_f32c_uart_init(void);

/** 通过 f32c_INST 阻塞发送一组 F32C 协议字节。 */
void drv_f32c_uart_write(const uint8_t *data, uint8_t length);

#endif /* __DRV_UART_H */
