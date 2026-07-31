#include "app.h"
#include "bsp_42step.h"
#include "drv_uart.h"
#include "mid_delay.h"

/*
 * 42 步进电机测试函数集
 */

/* 基础测试：查询版本 + 使能 + 正反转 + 失能 */
void app_42step_test(void)
{
    drv_uart0_init();
    Step42_Init();

    drv_uart_send_string("\r\n=== 42Step Test ===\r\n");

    /* 查版本 */
    drv_uart_send_string("Query version...\r\n");
    Step42_QueryVersion();
    delay_ms(200);

    /* 使能 */
    drv_uart_send_string("Enable...\r\n");
    Step42_Enable(true);
    delay_ms(500);

    /* 正转 200 RPM，2 秒 */
    drv_uart_send_string("CW 200RPM 2s\r\n");
    Step42_RunVelocity(STEP42_DIR_CW, 200, 10);
    delay_ms(2000);

    /* 反转 200 RPM，2 秒 */
    drv_uart_send_string("CCW 200RPM 2s\r\n");
    Step42_RunVelocity(STEP42_DIR_CCW, 200, 10);
    delay_ms(2000);

    /* 停止 */
    drv_uart_send_string("Stop\r\n");
    Step42_Stop();
    delay_ms(500);

    /* 失能 */
    Step42_Enable(false);
    drv_uart_send_string("=== Done ===\r\n");
}

/* 位置模式测试：正转 10000 脉冲，再反转 10000 脉冲回原位 */
void app_42step_position_test(void)
{
    drv_uart0_init();
    Step42_Init();

    drv_uart_send_string("\r\n=== 42Step Position Test ===\r\n");

    Step42_Enable(true);
    delay_ms(500);

    /* 正转 10000 脉冲 @ 500 RPM */
    drv_uart_send_string("CW 10000 pulses @500RPM\r\n");
    Step42_MovePosition(STEP42_DIR_CW, 500, 10, 10000, STEP42_POS_RELATIVE);
    delay_ms(2000);

    /* 反转 10000 脉冲 @ 500 RPM */
    drv_uart_send_string("CCW 10000 pulses @500RPM\r\n");
    Step42_MovePosition(STEP42_DIR_CCW, 500, 10, 10000, STEP42_POS_RELATIVE);
    delay_ms(2000);

    Step42_Stop();
    delay_ms(200);
    Step42_Enable(false);
    drv_uart_send_string("=== Done ===\r\n");
}

/* 简单速度控制：持续低速运转 */
void app_42step_simple(void)
{
    drv_uart0_init();
    Step42_Init();

    Step42_Enable(true);
    delay_ms(500);

    /* 以 300 RPM 持续正转 */
    Step42_RunVelocity(STEP42_DIR_CW, 300, 20);
}
