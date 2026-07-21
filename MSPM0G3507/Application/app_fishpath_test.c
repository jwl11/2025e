#include "ti_msp_dl_config.h"
#include "bsp_fishpath.h"
#include "bsp_encoder.h"
#include "bsp_led.h"
#include "bsp_button.h"
#include "bsp_OLED.h"
#include "drv_uart.h"
#include "mid_delay.h"
#include "mid_xunji.h"

/* Initial low-sensitivity tracking parameters. */
#define XUNJI_DEFAULT_KP          14.0f
#define XUNJI_DEFAULT_KI           0.0f
#define XUNJI_DEFAULT_KD           6.0f
#define XUNJI_DEFAULT_SPEED       25U

#define XUNJI_CONTROL_PERIOD_MS    5U
#define DEBUG_PRINT_MS           200U

/* 至少 4 路传感器同时有效确认弯角；到达第 5 个弯角才完成一圈。 */

#define TURNS_PER_LAP             4U    /* 每圈弯道数 */
#define TURN_MIN_INTERVAL_MS      1000U /* 相邻转角至少间隔 1s，防止重复计数 */
#define BUTTON_START_HOLD_MS      1000U /* 长按 1s 确认圈数并启动 */
static uint8_t   g_turn_count      = 0;   /* 当前圈内已通过弯道数 (0~3) */
static uint8_t   g_target_laps     = 0;   /* 0=未设置，有效范围 1~5 */
static uint8_t   g_lap_count       = 0;   /* 已完成圈数 */
static uint32_t  g_last_turn_tick  = 0;   /* 上次弯道完成时刻 (ms) */
static uint8_t   g_run_finished    = 0;   /* 达到目标圈数后置 1 */
static uint8_t   g_run_started     = 0;   /* 1=已确认圈数并启动 */
static uint32_t  g_button_press_tick = 0;
static uint8_t   g_long_press_handled = 0;

/* ---- OLED 显示状态 ---- */
#define OLED_UPDATE_MS           200U
static uint32_t  g_last_oled_tick  = 0;
static uint8_t   g_oled_needs_update = 1;  /* 首次强制刷新 */

/* ---- 按键 ---- */
static Button g_btn_user;

/* ---- 前向声明 ---- */
static void fishpath_print_header(void);
static void fishpath_print_status(uint32_t tick_ms);
static void fishpath_print_sensor_bar(const uint8_t *sensors);

static void print_3_digits(uint32_t value)
{
    drv_uart_send_string((value < 100U) ? "0" : "");
    drv_uart_send_string((value < 10U) ? "0" : "");
    drv_uart_print_num((unsigned long)value);
}

static void print_2_digits(uint32_t value)
{
    drv_uart_send_string((value < 10U) ? "0" : "");
    drv_uart_print_num((unsigned long)value);
}

static void print_float_3dp(float value)
{
    uint32_t scaled;

    if (value < 0.0f) {
        drv_uart_send_string("-");
        value = -value;
    }

    scaled = (uint32_t)(value * 1000.0f + 0.5f);
    drv_uart_print_num((unsigned long)(scaled / 1000U));
    drv_uart_send_string(".");
    print_3_digits(scaled % 1000U);
}

static void print_float_signed_2dp(float value)
{
    uint32_t scaled;

    if (value < 0.0f) {
        drv_uart_send_string("-");
        value = -value;
    } else {
        drv_uart_send_string("+");
    }

    scaled = (uint32_t)(value * 100.0f + 0.5f);
    drv_uart_print_num((unsigned long)(scaled / 100U));
    drv_uart_send_string(".");
    print_2_digits(scaled % 100U);
}

static void print_signed_pad(long value, uint8_t width)
{
    char buffer[12];
    char digits[10];
    uint8_t pos = 0;
    uint8_t digit_count = 0;
    unsigned long magnitude;

    if (value < 0) {
        buffer[pos++] = '-';
        magnitude = (unsigned long)(-value);
    } else {
        buffer[pos++] = '+';
        magnitude = (unsigned long)value;
    }

    if (magnitude == 0U) {
        digits[digit_count++] = '0';
    } else {
        while ((magnitude > 0U) && (digit_count < sizeof(digits))) {
            digits[digit_count++] = (char)('0' + (magnitude % 10U));
            magnitude /= 10U;
        }
    }

    while (digit_count > 0U) {
        buffer[pos++] = digits[--digit_count];
    }
    while ((pos < width) && (pos < (sizeof(buffer) - 1U))) {
        buffer[pos++] = ' ';
    }
    buffer[pos] = '\0';
    drv_uart_send_string(buffer);
}

static void print_duty_pct(long duty)
{
    if (duty < 10) {
        drv_uart_send_string("  ");
    } else if (duty < 100) {
        drv_uart_send_string(" ");
    }
    drv_uart_print_num((unsigned long)duty);
    drv_uart_send_string("%");
}

/* 消费“到达宽线弯角”事件：前四个通过，到达第五个时完成一圈。 */
static void fishpath_detect_turn(void)
{
    if (xunji_take_corner_event() != 0U) {
        uint32_t tick = get_system_ms();

        /* 同一个弯道内的再次丢线/捕线不允许重复计为下一个转角。 */
        if ((g_turn_count == 0U) ||
            ((tick - g_last_turn_tick) >= TURN_MIN_INTERVAL_MS)) {
            g_turn_count++;
            g_last_turn_tick = tick;
            g_oled_needs_update = 1U;

            if (g_turn_count > TURNS_PER_LAP) {
                /* 第 5 个弯角也是下一圈的第 1 个弯角。 */
                g_turn_count = 1U;
                g_lap_count++;
            }
        }
    }
}

/* ================================================================
 *  OLED 显示更新
 * ================================================================ */
static void fishpath_oled_update(void)
{
    OLED_ShowString(1, 1, "Target: ");
    OLED_ShowNum(1, 9, (uint32_t)g_target_laps, 1);
    OLED_ShowString(2, 1, "Laps:   ");
    OLED_ShowNum(2, 9, (uint32_t)g_lap_count, 1);
    if (g_run_started != 0U) {
        OLED_ShowString(3, 1, "Running         ");
    } else if (g_target_laps == 0U) {
        OLED_ShowString(3, 1, "Select laps     ");
    } else {
        OLED_ShowString(3, 1, "Hold to start   ");
    }
}

/* ================================================================
 *  主入口
 * ================================================================ */
void app_fishpath_test(void)
{
    uint32_t last_print_ms;
    uint32_t tick;
    uint8_t button_clicked;
    uint8_t button_released;

    drv_uart_send_string("========================================\r\n");
    drv_uart_send_string("  12-Ch Xunji + MG310 Motor Debug\r\n");
    drv_uart_send_string("  UART1 PA9 RX, PA8 TX @ 115200\r\n");
    drv_uart_send_string("  Short press=Set  Long press=Start\r\n");
    drv_uart_send_string("========================================\r\n\r\n");

    /* ---- 初始化 OLED ---- */
    OLED_Init();
    OLED_Clear();
    OLED_ShowString(1, 1, "Fishpath Init...");
    drv_uart_send_string("[INIT] OLED ready.\r\n");

    /* ---- 初始化按键 (PB15, 高电平触发) ---- */
    button_init(&g_btn_user, KEY_PORT, KEY_KEY1_PIN, BUTTON_ACTIVE_HIGH);
    drv_uart_send_string("[INIT] Button ready on PB15.\r\n");

    /* ---- 初始化循迹系统 ---- */
    fishpath_init(XUNJI_DEFAULT_KP, XUNJI_DEFAULT_KI,
                  XUNJI_DEFAULT_KD, XUNJI_DEFAULT_SPEED);
    /* fishpath_init() 默认启动，未设置圈数时必须立即停车。 */
    fishpath_stop();

    drv_uart_send_string("[INIT] Fishpath system ready.\r\n");
    drv_uart_send_string("[INIT] Kp=");
    print_float_3dp(XUNJI_DEFAULT_KP);
    drv_uart_send_string(" Ki=");
    print_float_3dp(XUNJI_DEFAULT_KI);
    drv_uart_send_string(" Kd=");
    print_float_3dp(XUNJI_DEFAULT_KD);
    drv_uart_send_string(" Speed=");
    drv_uart_print_num(XUNJI_DEFAULT_SPEED);
    drv_uart_send_string("%\r\n");
    drv_uart_send_string("[INIT] Turns per lap: ");
    drv_uart_print_num(TURNS_PER_LAP);
    drv_uart_send_string("\r\n\r\n");

    /* delay_ms also starts SysTick for get_system_ms(). */
    delay_ms(1U);

    /* 初始 OLED 刷新 */
    fishpath_oled_update();
    g_last_oled_tick = get_system_ms();

    fishpath_print_header();
    last_print_ms = get_system_ms();

    /* ---- 主循环 ---- */
    while (1) {
        /*
         * ① 运行时执行循迹和计圈。
         * 停车等待按键时仍持续解析 UART1，避免接收缓冲区溢出；
         * xunji_update() 只计算占空比，不会直接驱动电机。
         */
        if (g_run_started != 0U) {
            fishpath_update();
            fishpath_detect_turn();
        } else {
            xunji_update();
        }

        /* ③ 按键消抖采样 */
        button_update(&g_btn_user);

        delay_ms(XUNJI_CONTROL_PERIOD_MS);

        tick = get_system_ms();
        button_clicked  = button_is_clicked(&g_btn_user);
        button_released = button_is_released(&g_btn_user);

        /* ④ 停车时：短按选圈数，长按确认并启动。 */
        if (g_run_started == 0U) {
            if (button_clicked != 0U) {
                g_button_press_tick = tick;
                g_long_press_handled = 0U;
            }

            if ((button_is_pressed(&g_btn_user) != 0U) &&
                (g_long_press_handled == 0U) &&
                (g_target_laps != 0U) &&
                ((tick - g_button_press_tick) >= BUTTON_START_HOLD_MS)) {
                g_long_press_handled = 1U;
                g_lap_count = 0U;
                g_turn_count = 0U;
                /* 第一个弯角必须立即允许计数，由检测函数负责后续间隔过滤。 */
                g_last_turn_tick = 0U;
                xunji_reset_tracking();
                g_run_finished = 0U;
                g_run_started = 1U;
                /* 电机启动前完成一次 OLED 更新，运行中不再阻塞刷屏。 */
                fishpath_oled_update();
                g_oled_needs_update = 0U;
                g_last_oled_tick = tick;
                fishpath_start();

                drv_uart_send_string("[START] Target laps: ");
                drv_uart_print_num((unsigned long)g_target_laps);
                drv_uart_send_string("\r\n");
            }

            if (button_released != 0U) {
                if (g_long_press_handled == 0U) {
                    g_target_laps++;
                    if (g_target_laps > 5U) {
                        g_target_laps = 1U;
                    }
                    g_oled_needs_update = 1U;

                    drv_uart_send_string("[SET] Target laps: ");
                    drv_uart_print_num((unsigned long)g_target_laps);
                    drv_uart_send_string("\r\n");
                }
                g_long_press_handled = 0U;
            }
        }

        /* ⑤ 启动后达到设定圈数，立即停车。 */
        if ((g_run_started != 0U) &&
            (g_run_finished == 0U) &&
            (g_lap_count >= g_target_laps)) {
            g_run_finished = 1U;
            g_run_started = 0U;
            g_oled_needs_update = 1U;
            fishpath_stop();
            drv_uart_send_string("[DONE] Target laps reached. Car stopped.\r\n");
        }

        /* ⑥ UART 调试打印 (200ms 周期) */
        if ((tick - last_print_ms) >= DEBUG_PRINT_MS) {
            last_print_ms = tick;
            fishpath_print_status(tick);
            use_led_TOGGLE();
        }

        /* ⑦ OLED 仅在停车且内容变化时刷新，避免阻塞循迹控制。 */
        if ((tick - g_last_oled_tick) >= OLED_UPDATE_MS) {
            g_last_oled_tick = tick;
            if ((g_oled_needs_update != 0U) &&
                (g_run_started == 0U)) {
                g_oled_needs_update = 0;
                fishpath_oled_update();
            }
        }
    }
}

static void fishpath_print_header(void)
{
    drv_uart_send_string("Time(ms) | 012345678901 | Pos   Err   |");
    drv_uart_send_string(" Lduty Rduty | Status\r\n");
    drv_uart_send_string("---------+--------------+-------------+");
    drv_uart_send_string("-------------+--------\r\n");
}

static void fishpath_print_status(uint32_t tick_ms)
{
    uint8_t sensors[XUNJI_SENSOR_COUNT];
    int32_t pos;
    int32_t error;
    int32_t left;
    int32_t right;

    xunji_get_sensors(sensors);
    pos   = xunji_get_position();
    error = xunji_get_error();
    left  = xunji_get_left_duty();
    right = xunji_get_right_duty();

    drv_uart_print_num((unsigned long)tick_ms);
    drv_uart_send_string(" |");
    fishpath_print_sensor_bar(sensors);
    drv_uart_send_string(" | ");
    print_signed_pad((long)pos, 5U);
    drv_uart_send_string(" ");
    print_signed_pad((long)error, 5U);
    drv_uart_send_string(" | ");
    print_duty_pct((long)left);
    drv_uart_send_string("   ");
    print_duty_pct((long)right);
    drv_uart_send_string("  |");

    if (xunji_is_online()) {
        drv_uart_send_string(" OK  PID=");
        print_float_signed_2dp(g_pid_xunji.output);
    } else {
        drv_uart_send_string(" LOST-LINE!");
    }

    /* 附加圈数信息 */
    drv_uart_send_string(" L");
    drv_uart_print_num((unsigned long)g_lap_count);
    drv_uart_send_string("T");
    drv_uart_print_num((unsigned long)g_turn_count);

    drv_uart_send_string("\r\n");
}

static void fishpath_print_sensor_bar(const uint8_t *sensors)
{
    uint8_t i;

    for (i = 0; i < XUNJI_SENSOR_COUNT; i++) {
        drv_uart_send_string(sensors[i] ? "#" : ".");
    }
}

void app_fishpath_motor_test(void)
{
    drv_uart_send_string("MG310 motor test start.\r\n");
    mg310_motorInitAll();
    delay_ms(200U);

    mg310_motorForward(MG310_MOTOR_A, 30U);
    delay_ms(1500U);
    mg310_motorStop(MG310_MOTOR_A);
    delay_ms(500U);

    mg310_motorForward(MG310_MOTOR_B, 30U);
    delay_ms(1500U);
    mg310_motorStop(MG310_MOTOR_B);
    delay_ms(500U);

    mg310_motorForward(MG310_MOTOR_A, 40U);
    mg310_motorForward(MG310_MOTOR_B, 20U);
    delay_ms(2000U);

    mg310_motorForward(MG310_MOTOR_A, 20U);
    mg310_motorForward(MG310_MOTOR_B, 40U);
    delay_ms(2000U);

    mg310_motorStopAll();
    drv_uart_send_string("MG310 motor test complete.\r\n");
}
