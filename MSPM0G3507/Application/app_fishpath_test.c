#include "ti_msp_dl_config.h"
#include "bsp_fishpath.h"
#include "bsp_encoder.h"
#include "bsp_led.h"
#include "bsp_button.h"
#include "bsp_OLED.h"
#include "drv_uart.h"
#include "mid_delay.h"
#include "mid_xunji.h"
#include "app.h"

/*
 * 2026 电子设计大赛 第(2)题 — 差速循迹
 *
 * 赛道: 椭圆 + 半圆组合，启停线为宽黑线（≥3 路传感器同时检测到）。
 * 巡线: 正常行驶时仅 1 路传感器在线上，差速 PID 控制双电机。
 *
 * PID 设计（基于单传感器场景）:
 *   - 传感器间距 1000 权值，Kp=70 时每步 ≈12.7 差速变化
 *   - Ki=0.3 微弱积分，仅消除持续弯道稳态偏差，上限 20（最多 +6 差速）
 *   - Kd=5 抑制超调
 *   - 死区 4/6/10 防止电机堵转
 */



/* ================================================================
 *  状态
 * ================================================================ */
static uint8_t  g_target_laps     = 0;   /* 0=未设置 */
static uint8_t  g_lap_count       = 0;
static uint8_t  g_start_line_active = 1U; /* 启动时站在线上，先等驶离 */
static uint8_t  g_start_line_confirm   = 0U;
static uint8_t  g_start_line_release   = 0U;
static uint8_t  g_run_started     = 0;
static uint8_t  g_run_finished    = 0;

static uint32_t g_button_press_tick = 0;
static uint8_t  g_long_press_handled = 0;

static uint32_t g_last_oled_tick  = 0;
static uint8_t  g_oled_dirty      = 1;

static Button   g_btn_user;

/* ================================================================
 *  UART 打印辅助
 * ================================================================ */
static void print_3_digits(uint32_t value)
{
    drv_uart_send_string((value < 100U) ? "0" : "");
    drv_uart_send_string((value < 10U)  ? "0" : "");
    drv_uart_print_num((unsigned long)value);
}

static void print_float_signed_2dp(float value)
{
    uint32_t scaled;
    if (value < 0.0f) { drv_uart_send_string("-"); value = -value; }
    else              { drv_uart_send_string("+"); }
    scaled = (uint32_t)(value * 100.0f + 0.5f);
    drv_uart_print_num((unsigned long)(scaled / 100U));
    drv_uart_send_string(".");
    drv_uart_print_num((unsigned long)(scaled % 100U));
}

static void print_duty_pct(long duty)
{
    if (duty < 10)  drv_uart_send_string("  ");
    else if (duty < 100) drv_uart_send_string(" ");
    drv_uart_print_num((unsigned long)duty);
    drv_uart_send_string("%");
}

/* ================================================================
 *  启停线检测
 *
 *  调试输出: '.'=黑线（内部值 0），'#'=白底（内部值 1）。
 *  启停线 = 宽黑线，至少 3 路相邻传感器同时显示为 '.'。
 *  使用施密特触发: 先驶离(连续3帧无相邻3路黑线) →
 *  再进入(1帧出现相邻≥3路黑线)算通过一次。
 *  启动时 g_start_line_active=1，必须先驶离再回来才计第一圈。
 * ================================================================ */
static uint8_t is_start_line(void)
{
    uint8_t sensors[XUNJI_SENSOR_COUNT];
    uint8_t consecutive_black = 0U;
    uint8_t i;
    xunji_get_sensors(sensors);
    for (i = 0U; i < XUNJI_SENSOR_COUNT; i++) {
        if (sensors[i] == 0U) {
            consecutive_black++;
            if (consecutive_black >= START_LINE_SENSOR_MIN) {
                return 1U;
            }
        } else {
            consecutive_black = 0U;
        }
    }
    return 0U;
}

static uint8_t take_start_line_event(void)
{
    uint8_t on_line = is_start_line();

    if (g_start_line_active) {
        /* 站在线上: 等驶离 */
        if (!on_line) {
            if (g_start_line_release < START_LINE_RELEASE_CYCLES)
                g_start_line_release++;
            if (g_start_line_release >= START_LINE_RELEASE_CYCLES) {
                g_start_line_active = 0U;
                g_start_line_release = 0U;
            }
        } else {
            g_start_line_release = 0U;
        }
        return 0U;
    }

    /* 不在线上: 等再次压线 */
    if (on_line) {
        if (g_start_line_confirm < START_LINE_CONFIRM_CYCLES)
            g_start_line_confirm++;
        if (g_start_line_confirm >= START_LINE_CONFIRM_CYCLES) {
            g_start_line_active  = 1U;
            g_start_line_confirm = 0U;
            return 1U;
        }
    } else {
        g_start_line_confirm = 0U;
    }
    return 0U;
}

/* ================================================================
 *  OLED
 * ================================================================ */
static void oled_update(void)
{
    OLED_ShowString(1, 1, "Target: ");
    OLED_ShowNum(1, 9, (uint32_t)g_target_laps, 1);
    OLED_ShowString(2, 1, "Laps:   ");
    OLED_ShowNum(2, 9, (uint32_t)g_lap_count, 1);
    if (g_run_finished) {
        OLED_ShowString(3, 1, "Finished        ");
    } else if (g_run_started) {
        OLED_ShowString(3, 1, "Running         ");
    } else if (g_target_laps == 0U) {
        OLED_ShowString(3, 1, "Select laps     ");
    } else {
        OLED_ShowString(3, 1, "Hold to start   ");
    }
}

/* ================================================================
 *  UART 调试打印
 * ================================================================ */
static void print_header(void)
{
    drv_uart_send_string("Time(ms) | 012345678901 | Pos   Err  |");
    drv_uart_send_string(" Lduty Rduty | PID   Status\r\n");
    drv_uart_send_string("---------+--------------+-----------+");
    drv_uart_send_string("-------------+------+-------\r\n");
}

static void print_status(uint32_t tick_ms)
{
    uint8_t sensors[XUNJI_SENSOR_COUNT];
    int32_t pos, error, left, right;
    uint8_t i;

    xunji_get_sensors(sensors);
    pos   = xunji_get_position();
    error = xunji_get_error();
    left  = xunji_get_left_duty();
    right = xunji_get_right_duty();

    drv_uart_print_num((unsigned long)tick_ms);
    drv_uart_send_string(" |");
    for (i = 0; i < XUNJI_SENSOR_COUNT; i++)
        drv_uart_send_string(sensors[i] ? "#" : ".");
    drv_uart_send_string(" |");
    /* signed position & error */
    if (pos < 0) { drv_uart_send_string("-"); pos = -pos; }
    else         { drv_uart_send_string("+"); }
    drv_uart_send_string(pos < 1000 ? " " : "");
    drv_uart_print_num((unsigned long)pos);
    drv_uart_send_string(" ");
    if (error < 0) { drv_uart_send_string("-"); error = -error; }
    else           { drv_uart_send_string("+"); }
    drv_uart_send_string(error < 1000 ? " " : "");
    drv_uart_print_num((unsigned long)error);
    drv_uart_send_string("  |");
    print_duty_pct((long)left);
    drv_uart_send_string("   ");
    print_duty_pct((long)right);
    drv_uart_send_string("  |");
    print_float_signed_2dp(g_pid_xunji.output);
    drv_uart_send_string(" ");
    drv_uart_send_string(xunji_is_online() ? "OK" : "LOST");
    drv_uart_send_string(" L");
    drv_uart_print_num((unsigned long)g_lap_count);
    drv_uart_send_string("\r\n");
}

/* ================================================================
 *  主入口
 * ================================================================ */
void app_fishpath_test(void)
{
    uint32_t tick, last_print_ms;

    /* ---- 欢迎信息 ---- */
    drv_uart_send_string("========================================\r\n");
    drv_uart_send_string("  2026 E-Contest (2) — Diff Tracking\r\n");
    drv_uart_send_string("  Short:Set Laps  LongHold:Start\r\n");
    drv_uart_send_string("========================================\r\n");

    /* ---- OLED ---- */
    OLED_Init();
    OLED_Clear();
    OLED_ShowString(1, 1, "Fishpath Init...");
    drv_uart_send_string("[INIT] OLED ready.\r\n");

    /* ---- 按键 ---- */
    button_init(&g_btn_user, KEY_PORT, KEY_KEY1_PIN, BUTTON_ACTIVE_HIGH);
    drv_uart_send_string("[INIT] Button ready.\r\n");

    /* ---- 循迹系统 ---- */
    fishpath_init(XUNJI_DEFAULT_KP, XUNJI_DEFAULT_KI,
                  XUNJI_DEFAULT_KD, XUNJI_DEFAULT_SPEED);
    fishpath_stop();  /* 初始化后默认停车，等按键启动 */

    drv_uart_send_string("[INIT] Kp=");
    drv_uart_print_num((unsigned long)XUNJI_DEFAULT_KP);
    drv_uart_send_string(" Ki=");
    print_float_signed_2dp(XUNJI_DEFAULT_KI);
    drv_uart_send_string(" Kd=");
    drv_uart_print_num((unsigned long)XUNJI_DEFAULT_KD);
    drv_uart_send_string(" Speed=");
    drv_uart_print_num(XUNJI_DEFAULT_SPEED);
    drv_uart_send_string("%\r\n\r\n");

    delay_ms(1U);
    oled_update();
    g_last_oled_tick = get_system_ms();
    print_header();
    last_print_ms = get_system_ms();

    /* ================================================================
     *  主循环 (5ms)
     * ================================================================ */
    for (;;) {
        /* ① 循迹更新 (停车时也解析传感器，防止 UART 溢出) */
        if (g_run_started) {
            fishpath_update();

            /* 计圈 */
            if (take_start_line_event()) {
                g_lap_count++;
                g_oled_dirty = 1;
                drv_uart_send_string("[LAP] ");
                drv_uart_print_num((unsigned long)g_lap_count);
                drv_uart_send_string("\r\n");
            }
        } else {
            /* 停车时只解析，不驱动电机 */
            xunji_update();
        }

        /* ② 按键扫描 */
        button_update(&g_btn_user);
        delay_ms(XUNJI_CONTROL_PERIOD_MS);
        tick = get_system_ms();

        /* ③ 停车时: 短按选圈数 / 长按启动 */
        if (!g_run_started) {
            if (button_is_clicked(&g_btn_user)) {
                g_button_press_tick = tick;
                g_long_press_handled = 0U;
            }
            /* 长按确认 (目标圈数已设置) */
            if (button_is_pressed(&g_btn_user) &&
                !g_long_press_handled &&
                g_target_laps != 0U &&
                (tick - g_button_press_tick) >= BUTTON_START_HOLD_MS) {

                g_long_press_handled  = 1U;
                g_lap_count           = 0U;
                g_start_line_active   = 1U;  /* 站在启停线上 */
                g_start_line_confirm  = 0U;
                g_start_line_release  = 0U;
                g_run_finished        = 0U;
                g_run_started         = 1U;

                xunji_reset_tracking();
                fishpath_start();

                oled_update();
                g_oled_dirty = 0;
                g_last_oled_tick = tick;

                drv_uart_send_string("[START] ");
                drv_uart_print_num((unsigned long)g_target_laps);
                drv_uart_send_string(" laps\r\n");
            }
            /* 短按: 圈数 +1 */
            if (button_is_released(&g_btn_user)) {
                if (!g_long_press_handled) {
                    g_target_laps++;
                    if (g_target_laps > TARGET_LAPS_MAX)
                        g_target_laps = 1U;
                    g_oled_dirty = 1;
                    drv_uart_send_string("[SET] ");
                    drv_uart_print_num((unsigned long)g_target_laps);
                    drv_uart_send_string(" laps\r\n");
                }
                g_long_press_handled = 0U;
            }
        }

        /* ④ 达到目标圈数 → 停车 */
        if (g_run_started && !g_run_finished &&
            g_lap_count >= g_target_laps) {
            g_run_finished = 1U;
            g_run_started  = 0U;
            g_oled_dirty   = 1;
            fishpath_stop();
            drv_uart_send_string("[DONE]\r\n");
        }

        /* ⑤ 调试打印 (200ms) */
        if ((tick - last_print_ms) >= DEBUG_PRINT_MS) {
            last_print_ms = tick;
            print_status(tick);
            use_led_TOGGLE();
        }

        /* ⑥ OLED 刷新 (停车且内容变化时) */
        if ((tick - g_last_oled_tick) >= OLED_UPDATE_MS) {
            g_last_oled_tick = tick;
            if (g_oled_dirty && !g_run_started) {
                g_oled_dirty = 0;
                oled_update();
            }
        }
    }
}

/* ---- 电机测试 (保留，调试用) ---- */
void app_fishpath_motor_test(void)
{
    drv_uart_send_string("MG310 motor test.\r\n");
    mg310_motorInitAll();
    delay_ms(200U);
    mg310_motorForward(MG310_MOTOR_A, 30U); delay_ms(1500U);
    mg310_motorStop(MG310_MOTOR_A);         delay_ms(500U);
    mg310_motorForward(MG310_MOTOR_B, 30U); delay_ms(1500U);
    mg310_motorStop(MG310_MOTOR_B);         delay_ms(500U);

    mg310_motorForward(MG310_MOTOR_A, 40U);
    mg310_motorForward(MG310_MOTOR_B, 20U); delay_ms(2000U);
    mg310_motorForward(MG310_MOTOR_A, 20U);
    mg310_motorForward(MG310_MOTOR_B, 40U); delay_ms(2000U);
    mg310_motorStopAll();
    drv_uart_send_string("MG310 motor test done.\r\n");
}
