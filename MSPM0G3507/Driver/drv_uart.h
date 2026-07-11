#ifndef __DRV_UART_H
#define __DRV_UART_H

#include "ti_msp_dl_config.h"

void drv_uart_send_char(char c);
void drv_uart_send_string(char *str);
void drv_uart_printf(const char *fmt, ...);

#endif // __DRV_UART_H
