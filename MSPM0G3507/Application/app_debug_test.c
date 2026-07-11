#include "ti_msp_dl_config.h"
#include "drv_uart.h"
#include "mid_delay.h"
#include "bsp_led.h"
#include <stdio.h>

void app_debug_test(void)
{
    uint32_t cnt = 0;

    printf("================================\r\n");
    printf("   UART Printf Test Start\r\n");
    printf("   MSPM0G3507 @ 32MHz\r\n");
    printf("   Baudrate: 115200\r\n");
    printf("================================\r\n\r\n");

    while (1)
    {
        cnt++;
        printf("Count: %lu\r\n", cnt);
        printf("Hex:   0x%08lX\r\n", cnt);
        printf("\r\n");

        use_led_TOGGLE();
        delay_ms(1000);
    }
}
