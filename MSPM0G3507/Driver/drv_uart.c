#include "drv_uart.h"
#include <stdio.h>

/**
 * @brief  重定向 printf 底层输出到 UART
 */
int fputc(int ch, FILE *f)
{
    DL_UART_Main_transmitDataBlocking(debug_INST, (uint8_t)ch);
    return ch;
}

/**
 * @brief  发送字符串（阻塞）
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
 * @brief  打印无符号整数（绕过 printf 格式化限制）
 *
 *         嵌入式 newlib-nano 的 printf 在多参数或非 long
 *         类型时可能无法正确打印，此函数直接逐位转换发送。
 */
void drv_uart_print_num(unsigned long num)
{
    char  buf[12];  /* 0-4294967295 最多 10 位 + 符号 + null */
    uint8_t i = 0;

    if (num == 0) {
        buf[i++] = '0';
    } else {
        /* 从低位到高位填入 */
        uint8_t j = 0;
        char    tmp[12];
        while (num > 0) {
            tmp[j++] = '0' + (num % 10);
            num /= 10;
        }
        /* 反转得到高位到低位 */
        while (j > 0) {
            buf[i++] = tmp[--j];
        }
    }
    buf[i] = '\0';
    drv_uart_send_string(buf);
}

/**
 * @brief  打印有符号整数
 *
 *         负数打印 '-' 前缀，内部转 unsigned long 复用 print_num。
 */
void drv_uart_print_signed(long num)
{
    if (num < 0) {
        drv_uart_send_string("-");
        num = -num;
    }
    drv_uart_print_num((unsigned long)num);
}

/**
 * @brief  打印两位十六进制数（0x00 ~ 0xFF）
 */
void drv_uart_print_hex(uint8_t num)
{
    char hex[3];
    char nibble;

    nibble = (num >> 4) & 0x0F;
    hex[0] = (nibble < 10) ? ('0' + nibble) : ('A' + nibble - 10);
    nibble = num & 0x0F;
    hex[1] = (nibble < 10) ? ('0' + nibble) : ('A' + nibble - 10);
    hex[2] = '\0';
    drv_uart_send_string(hex);
}
