#include "ti_msp_dl_config.h"
#include "drv_uart.h"
#include "mid_delay.h"
#include "bsp_led.h"

/**
 * @brief  打印 32 位十六进制数 (8 位补零)
 *
 *         例: 0x00000000 ~ 0xFFFFFFFF
 */
static void print_hex32(uint32_t val)
{
    drv_uart_print_hex((uint8_t)(val >> 24));
    drv_uart_print_hex((uint8_t)(val >> 16));
    drv_uart_print_hex((uint8_t)(val >> 8));
    drv_uart_print_hex((uint8_t)(val));
}

void app_debug_test(void)
{
    uint32_t cnt = 0;

    drv_uart_send_string("================================\r\n");
    drv_uart_send_string("   UART Debug Test Start\r\n");
    drv_uart_send_string("   MSPM0G3507 @ 32MHz\r\n");
    drv_uart_send_string("   Baudrate: 115200\r\n");
    drv_uart_send_string("================================\r\n\r\n");

    while (1)
    {
        cnt++;
        drv_uart_send_string("Count: ");
        drv_uart_print_num(cnt);
        drv_uart_send_string("\r\n");

        drv_uart_send_string("Hex:   0x");
        print_hex32(cnt);
        drv_uart_send_string("\r\n\r\n");

        use_led_TOGGLE();
        delay_ms(1000);
    }
}
