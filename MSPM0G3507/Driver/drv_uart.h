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
 * UART3 (vision) — MaixCAM2 坐标接收
 *
 * PB3=RX, PB2=TX, 115200-8N1。
 * ISR 内直接解析 @X,Y# 协议帧，主循环零延迟读取坐标。
 * ================================================================ */

void     drv_vision_uart3_init(void);
bool     drv_vision_uart3_write(const uint8_t *data, uint16_t length);
uint16_t drv_vision_get_x(void);
uint16_t drv_vision_get_y(void);
uint32_t drv_vision_get_frame_count(void);

/* 外部可直接读，ISR 自动更新 */
extern volatile uint16_t vision_x;
extern volatile uint16_t vision_y;

#endif /* __DRV_UART_H */
