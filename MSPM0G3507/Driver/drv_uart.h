#ifndef __DRV_UART_H
#define __DRV_UART_H

#include "ti_msp_dl_config.h"

void drv_uart_send_string(char *str);
void drv_uart_print_num(unsigned long num);
void drv_uart_print_signed(long num);
void drv_uart_print_hex(uint8_t num);

#endif // __DRV_UART_H
