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
 * UART3 (vision) — MaixCAM2 双向通信
 *
 * PB3=RX, PB2=TX, 115200-8N1。ISR 只搬运字节，协议解析在主循环。
 * ================================================================ */

void     drv_vision_uart3_init(void);
bool     drv_vision_uart3_write(const uint8_t *data, uint16_t length);
uint16_t drv_vision_uart3_available(void);
int16_t  drv_vision_uart3_read(void);
void     drv_vision_uart3_flush(void);
uint32_t drv_vision_uart3_get_overflow_count(void);

#endif /* __DRV_UART_H */
