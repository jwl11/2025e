#include "drv_uart.h"
#include <stdio.h>

/**
 * @brief  重定向 printf 底层输出到 UART
 *
 *         标准库 printf 最终会调用 fputc 输出每个字符，
 *         重写此函数即可让 printf 直接通过串口打印，
 *         无需任何包装函数。
 */
int fputc(int ch, FILE *f)
{
    DL_UART_Main_transmitDataBlocking(debug_INST, (uint8_t)ch);
    return ch;
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
