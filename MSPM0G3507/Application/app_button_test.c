#include "ti_msp_dl_config.h"
#include "bsp_button.h"
#include "bsp_led.h"
#include "drv_uart.h"
#include "mid_delay.h"

/* ================================================================
 * 按键测试程序 (Application 层)
 *
 * 按键配置: PB15, 高电平触发
 *
 * 演示功能:
 *   1. 短按 (< 500ms)  → 翻转 LED 并打印
 *   2. 长按 (>= 500ms) → 持续打印计数器
 *   3. 释放事件       → 打印长按统计
 *
 * 分层调用链:
 *   app_button_test
 *     ├── button_init()       [BSP]  初始化按键句柄
 *     ├── button_update()     [BSP]  消抖采样 (5ms 周期)
 *     ├── button_is_clicked() [BSP]  边沿检测
 *     ├── button_is_pressed() [BSP]  电平检测
 *     ├── button_is_released()[BSP]  释放检测
 *     ├── use_led_TOGGLE()    [BSP]  LED 控制
 *     ├── drv_uart_*()        [Driver] 串口输出
 *     └── delay_ms()          [Middleware] 时基
 * ================================================================ */

/* ---- 按键 GPIO 配置 ---- */
/*
 * PB15 已在 SysConfig 中配置为 GPIO 输入 (KEY1),
 * 对应的宏定义来自 ti_msp_dl_config.h:
 *   KEY_PORT      → GPIOB
 *   KEY_KEY1_PIN  → DL_GPIO_PIN_15
 *
 * 若要将其他 GPIO 复用为按键, 仿照 SysConfig 添加新引脚,
 * 使用生成的 <NAME>_PORT / <NAME>_PIN 宏即可。
 * 参考: 给 Button 结构体传入不同的 port/pin 参数。
 */
#define BTN_USER_PORT       KEY_PORT
#define BTN_USER_PIN        KEY_KEY1_PIN
#define BTN_USER_ACTIVE     BUTTON_ACTIVE_HIGH   /* 高电平触发 */

/* ---- 应用参数 ---- */
#define BUTTON_POLL_MS       5U     /* 与 BUTTON_DEBOUNCE_MS 一致 */
#define LONG_PRESS_THRESHOLD 500U   /* 长按判定阈值 (ms) */
#define LONG_PRINT_INTERVAL  200U   /* 长按时打印间隔 (ms) */

/* ---- 全局按键句柄 ---- */
static Button g_btn_user;

/* ================================================================
 *  主测试入口
 * ================================================================ */

void app_button_test(void)
{
    uint32_t tick;
    uint32_t long_start_tick = 0;
    uint32_t long_print_tick = 0;
    uint32_t long_count      = 0;
    uint8_t  in_long_press   = 0;

    /* ---- 初始化 ---- */
    drv_uart_send_string("========================================\r\n");
    drv_uart_send_string("  Button Test — PB15, High-Level Active\r\n");
    drv_uart_send_string("  Short press: toggle LED\r\n");
    drv_uart_send_string("  Long press  (>0.5s): count\r\n");
    drv_uart_send_string("========================================\r\n\r\n");

    button_init(&g_btn_user, BTN_USER_PORT, BTN_USER_PIN, BTN_USER_ACTIVE);

    drv_uart_send_string("[INIT] Button ready on PB15.\r\n\r\n");

    /* 首次 poll 确保初始状态 */
    delay_ms(1U);

    /* ---- 主循环 ---- */
    while (1) {
        /* ① 消抖采样 (每 5ms) */
        button_update(&g_btn_user);

        /* ② 处理短按事件 (边沿触发, 单次) */
        if (button_is_clicked(&g_btn_user)) {
            use_led_TOGGLE();
            drv_uart_send_string("[BTN] Clicked! LED toggled.\r\n");
        }

        /* ③ 长按检测 — 按下并保持 */
        if (button_is_pressed(&g_btn_user) && (in_long_press == 0U)) {
            tick = get_system_ms();

            if (long_start_tick == 0U) {
                /* 第一次检测到按下, 开始计时 */
                long_start_tick = tick;
            } else if ((tick - long_start_tick) >= LONG_PRESS_THRESHOLD) {
                /* 进入长按状态 */
                in_long_press    = 1U;
                long_count       = 1U;
                long_print_tick  = tick;

                drv_uart_send_string("[BTN] Long press started! Counting...\r\n");
            }
        }

        /* ④ 长按持续动作 */
        if ((in_long_press == 1U) && button_is_pressed(&g_btn_user)) {
            tick = get_system_ms();
            if ((tick - long_print_tick) >= LONG_PRINT_INTERVAL) {
                long_print_tick = tick;
                long_count++;

                drv_uart_send_string("[BTN] Long press count = ");
                drv_uart_print_num((unsigned long)long_count);
                drv_uart_send_string("\r\n");
            }
        }

        /* ⑤ 释放事件 */
        if (button_is_released(&g_btn_user)) {
            if (in_long_press == 1U) {
                drv_uart_send_string("[BTN] Released after long press (");
                drv_uart_print_num((unsigned long)long_count);
                drv_uart_send_string(" counts).\r\n");
            } else {
                drv_uart_send_string("[BTN] Released (short press).\r\n");
            }

            /* 重置长按状态 */
            in_long_press   = 0U;
            long_start_tick = 0U;
            long_count      = 0U;
        }

        delay_ms(BUTTON_POLL_MS);
    }
}
