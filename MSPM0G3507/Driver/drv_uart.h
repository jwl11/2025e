#ifndef __DRV_UART_H
#define __DRV_UART_H

#include "ti_msp_dl_config.h"

/* ================================================================
 * UART0 (debug) — 阻塞发送 / printf 重定向
 * ================================================================ */

void drv_uart_send_string(char *str);
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

#endif /* __DRV_UART_H */
