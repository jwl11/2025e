#include "ti_msp_dl_config.h"
#include "drv_uart.h"
#include "mid_delay.h"
#include "bsp_led.h"

void app_debug_test(void)
{
    uint32_t cnt = 0;

    drv_uart_printf("================================\r\n");
    drv_uart_printf("   UART Printf Test Start\r\n");
    drv_uart_printf("   MSPM0G3507 @ 32MHz\r\n");
    drv_uart_printf("   Baudrate: 115200\r\n");
    drv_uart_printf("================================\r\n\r\n");

    while (1)
    {
        cnt++;
        drv_uart_printf("Count: %lu\r\n", cnt);
        drv_uart_printf("Hex:   0x%08lX\r\n", cnt);
        drv_uart_printf("\r\n");

        use_led_TOGGLE();
        delay_ms(1000);
    }
}
