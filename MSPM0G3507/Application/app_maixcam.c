#include "ti_msp_dl_config.h"
#include "drv_uart.h"
#include "mid_delay.h"

void app_RX_maixcam_test(void)
{
    uint32_t last_frame = 0U;
    uint16_t x, y;

    drv_uart0_init();
    drv_vision_uart3_init();

    drv_uart_send_string("MaixCAM2 @X,Y# Test\r\n");

    while (1) {

            x = drv_vision_get_x();
            y = drv_vision_get_y();

            drv_uart_send_string("X=");
            drv_uart_print_num(x);
            drv_uart_send_string(" Y=");
            drv_uart_print_num(y);
            drv_uart_send_string("\r\n");

        delay_ms(50);
    }
}

