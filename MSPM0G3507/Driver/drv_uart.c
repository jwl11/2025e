#include "drv_uart.h"
#include <stdio.h>
#include <stdarg.h>

#define DRV_UART_PRINTF_BUF_SIZE 256

/**
 * @brief  发送单个字符（阻塞）
 * @param  c 要发送的字符
 */
void drv_uart_send_char(char c)
{
    DL_UART_Main_transmitDataBlocking(debug_INST, (uint8_t)c);
}

/**
 * @brief  发送字符串（阻塞）
 * @param  str 字符串指针（以 '\0' 结尾）
 */
void drv_uart_send_string(char *str)
{
    while (*str != '\0')
    {
        DL_UART_Main_transmitDataBlocking(debug_INST, (uint8_t)(*str));
        str++;
    }
}

/**
 * @brief  格式化打印（printf风格，阻塞）
 * @param  fmt 格式化字符串
 * @param  ... 可变参数
 */
void drv_uart_printf(const char *fmt, ...)
{
    char buf[DRV_UART_PRINTF_BUF_SIZE];
    va_list args;

    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    drv_uart_send_string(buf);
}
