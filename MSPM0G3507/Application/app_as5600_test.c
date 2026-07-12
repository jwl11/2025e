#include "app.h"
#include "bsp_AS5600.h"
#include "bsp_led.h"
#include "drv_uart.h"
#include "mid_delay.h"

/**
 * @brief  AS5600 三层 API 测试例程
 */
void app_as5600_test(void)
{
    uint16_t raw_angle;
    uint8_t  status, agc;

    /* ---- 底层：用 read_reg 检测连接 ---- */
    drv_uart_send_string("=== AS5600 3-Layer API Test ===\r\n");

    status = as5600_read_reg(AS5600_REG_STATUS);
    agc    = as5600_read_reg(AS5600_REG_AGC);

    if (status == 0 && agc == 0) {
        drv_uart_send_string("[FAIL] AS5600 not found!\r\n");
        while (1) {
            use_led_TOGGLE();
            delay_ms(500);
        }
    }
    drv_uart_send_string("[OK] AS5600 found\r\n");

    drv_uart_send_string("STS=0x");
    drv_uart_print_hex(status);
    drv_uart_send_string("  AGC=");
    drv_uart_print_num(agc);
    drv_uart_send_string("\r\n================================\r\n\r\n");

    while (1) {
        raw_angle = as5600_read_raw_angle();

        float rad       = as5600_get_angle_radians();
        float multiturn = as5600_get_angle_multiturn();
        float speed     = as5600_get_speed();

        drv_uart_send_string("RAW=");
        drv_uart_print_num(raw_angle);

        drv_uart_send_string("  RAD=");
        drv_uart_print_num((unsigned long)(rad * 1000.0f));

        drv_uart_send_string("  MT=");
        drv_uart_print_num((unsigned long)(multiturn * 1000.0f));

        drv_uart_send_string("  SPD=");
        drv_uart_print_signed((long)(speed * 1000.0f));
        drv_uart_send_string(" mrad/s\r\n");

        {
            static uint8_t cnt = 0;
            if (++cnt >= 10) {
                cnt    = 0;
                status = as5600_read_reg(AS5600_REG_STATUS);
                agc    = as5600_read_reg(AS5600_REG_AGC);
                drv_uart_send_string("  [STS=0x");
                drv_uart_print_hex(status);
                drv_uart_send_string(" AGC=");
                drv_uart_print_num(agc);
                drv_uart_send_string("] ");
                if (status & AS5600_STATUS_MD) drv_uart_send_string("MD ");
                if (status & AS5600_STATUS_ML) drv_uart_send_string("ML ");
                if (status & AS5600_STATUS_MH) drv_uart_send_string("MH ");
                drv_uart_send_string("\r\n");
            }
        }

        use_led_TOGGLE();
        delay_ms(100);   /* 首次调用后 SysTick 保持常开，计时基准自动就绪 */
    }
}
