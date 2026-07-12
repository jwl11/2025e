#include "app.h"
#include "mid_foc.h"
#include "bsp_AS5600.h"
#include "bsp_led.h"
#include "drv_uart.h"
#include "mid_delay.h"

/*
 *  P1 = 校准
 *  P2 = 开环 2s，启动电机 + 收敛速度滤波器
 *  P3 = 闭环 5s，目标 5 rad/s
 */

void app_BLCD_test(void)
{
    drv_uart_send_string("\r\n=== FOC Closed-Loop Test ===\r\n");

    /* ---- P1 ---- */
    drv_uart_send_string("[P1] Calibrate\r\n");
    foc_init(12.6f);
    for (int i = 0; i < 3; i++) { use_led_ON(); delay_ms(150); use_led_OFF(); delay_ms(150); }
    foc_as5600_init(7, -1);   /* DIR=-1: 电机实际转向为负 */
    drv_uart_send_string("     Done\r\n");

    /* ---- P2: 开环 2 秒 ---- */
    drv_uart_send_string("[P2] Open-loop spin 2s\r\n");
    {
        uint32_t t0 = get_system_us();
        for (uint16_t i = 0; i < 2000; i++) {
            uint32_t t1   = get_system_us();
            float    dt_s = (float)(t1 - t0) / 1000000.0f;
            t0 = t1;
            foc_openloop_spin(4.0f, 50.0f, dt_s);
            use_led_TOGGLE();
            delay_ms(1);
        }
    }
    drv_uart_send_string("     Ready\r\n");

    /* ---- P3: 闭环 5 rad/s ---- */
    drv_uart_send_string("[P3] Closed-loop target=5rad/s\r\n");
    {
        uint32_t t0 = get_system_us(), last_print = t0;
        for (uint16_t i = 0; i < 5000; i++) {
            foc_set_speed(5.0f);

            uint32_t t1 = get_system_us();
            if (t1 - last_print >= 300000UL) {
                last_print = t1;
                float spd = as5600_get_speed();
                drv_uart_send_string("  SPD=");
                drv_uart_print_signed((long)(spd * 100.0f));
                drv_uart_send_string(" crad/s\r\n");
            }
            use_led_TOGGLE();
            delay_ms(1);
        }
    }
    foc_stop();

    drv_uart_send_string("=== Done ===\r\n");
    while (1) { use_led_TOGGLE(); delay_ms(500); }
}
