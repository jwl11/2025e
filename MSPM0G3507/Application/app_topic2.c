#include "app.h"
#include "bsp_button.h"
#include "bsp_fishpath.h"
#include "bsp_OLED.h"
#include "mid_delay.h"
#include "mid_xunji.h"

/*
 * 题目 2：差速循迹
 *
 * 短按 KEY1 选择目标圈数，长按 KEY1 启动。
 * 启停线为宽黑线，至少相邻 3 路传感器同时检测到黑线时有效。
 * 小车启动时开始计时，完成目标圈数停车并冻结时间。
 */

#define TOPIC2_TIMER_DISPLAY_UPDATE_MS  500U

/* ================================================================
 * 状态
 * ================================================================ */
static uint8_t  topic2_target_laps;
static uint8_t  topic2_lap_count;
static uint8_t  topic2_start_line_active;
static uint8_t  topic2_start_line_confirm;
static uint8_t  topic2_start_line_release;
static uint8_t  topic2_run_started;
static uint8_t  topic2_run_finished;

static uint32_t topic2_button_press_tick;
static uint8_t  topic2_long_press_handled;

static uint32_t topic2_last_oled_tick;
static uint8_t  topic2_oled_dirty;
static uint32_t topic2_last_time_display_ms;

static Button   topic2_button;

/* ================================================================
 * 启停线检测
 *
 * 传感器协议：黑线为内部值 0，白底为内部值 1。
 * 启停线：至少相邻 START_LINE_SENSOR_MIN 路传感器检测到黑线。
 * 启动时小车位于启停线上，必须先连续驶离，再次进入才计一圈。
 * ================================================================ */
static uint8_t topic2_is_start_line(void)
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

static uint8_t topic2_take_start_line_event(void)
{
    uint8_t on_line = topic2_is_start_line();

    if (topic2_start_line_active != 0U) {
        /* 当前站在线上，等待小车稳定驶离。 */
        if (on_line == 0U) {
            if (topic2_start_line_release < START_LINE_RELEASE_CYCLES) {
                topic2_start_line_release++;
            }
            if (topic2_start_line_release >= START_LINE_RELEASE_CYCLES) {
                topic2_start_line_active = 0U;
                topic2_start_line_release = 0U;
            }
        } else {
            topic2_start_line_release = 0U;
        }
        return 0U;
    }

    /* 已经驶离启停线，等待再次进入。 */
    if (on_line != 0U) {
        if (topic2_start_line_confirm < START_LINE_CONFIRM_CYCLES) {
            topic2_start_line_confirm++;
        }
        if (topic2_start_line_confirm >= START_LINE_CONFIRM_CYCLES) {
            topic2_start_line_active = 1U;
            topic2_start_line_confirm = 0U;
            return 1U;
        }
    } else {
        topic2_start_line_confirm = 0U;
    }

    return 0U;
}

/* ================================================================
 * OLED 显示
 * ================================================================ */
static void topic2_oled_update(void)
{
    OLED_ShowString(1, 1, "Target: ");
    OLED_ShowNum(1, 9, (uint32_t)topic2_target_laps, 1);
    OLED_ShowString(2, 1, "Laps:   ");
    OLED_ShowNum(2, 9, (uint32_t)topic2_lap_count, 1);

    if (topic2_run_finished != 0U) {
        OLED_ShowString(3, 1, "Finished        ");
    } else if (topic2_run_started != 0U) {
        OLED_ShowString(3, 1, "Running         ");
    } else if (topic2_target_laps == 0U) {
        OLED_ShowString(3, 1, "Select laps     ");
    } else {
        OLED_ShowString(3, 1, "Hold to start   ");
    }
}

static void topic2_oled_show_time(uint32_t elapsed_ms)
{
    uint32_t total_seconds = elapsed_ms / 1000U;
    uint32_t minutes = (total_seconds / 60U) % 100U;
    uint32_t seconds = total_seconds % 60U;
    uint32_t tenths = (elapsed_ms / 100U) % 10U;

    OLED_ShowNum(4, 6, minutes, 2);
    OLED_ShowNum(4, 9, seconds, 2);
    OLED_ShowNum(4, 12, tenths, 1);
}

/* ================================================================
 * 题目 2 入口
 * ================================================================ */
void topic2(void)
{
    uint32_t tick;
    uint32_t elapsed_ms;

    /* 初始化状态，保证每次进入题目 2 都从待设置状态开始。 */
    topic2_target_laps          = 0U;
    topic2_lap_count            = 0U;
    topic2_start_line_active    = 1U;
    topic2_start_line_confirm   = 0U;
    topic2_start_line_release   = 0U;
    topic2_run_started          = 0U;
    topic2_run_finished         = 0U;
    topic2_button_press_tick    = 0U;
    topic2_long_press_handled   = 0U;
    topic2_last_oled_tick       = 0U;
    topic2_oled_dirty           = 1U;
    topic2_last_time_display_ms = 0U;

    /** 初始化 **/
    OLED_Init();
    OLED_Clear();
    OLED_ShowString(1, 1, "Topic 2 Init...");

    button_init(&topic2_button, KEY_PORT, KEY_KEY1_PIN, BUTTON_ACTIVE_HIGH);

    fishpath_init(XUNJI_DEFAULT_KP, XUNJI_DEFAULT_KI,
                  XUNJI_DEFAULT_KD, XUNJI_DEFAULT_SPEED);
    fishpath_stop();

    topic2_oled_update();
    OLED_ShowString(4, 1, "Time 00:00.0    ");
    topic2_oled_show_time(0U);
    topic2_last_oled_tick = get_system_ms();

    while (1) {
        /******************** 控制循迹 ********************/
        if (topic2_run_started != 0U) {
            fishpath_update();

            if (topic2_take_start_line_event() != 0U) {
                topic2_lap_count++;
                topic2_oled_dirty = 1U;
            }
        } else {
            /* 停车时只更新传感器状态，不驱动电机。 */
            xunji_update();
        }

        /******************** 扫描按键 ********************/
        button_update(&topic2_button);
        delay_ms(XUNJI_CONTROL_PERIOD_MS);
        tick = get_system_ms();

        if (topic2_run_started == 0U) {
            if (button_is_clicked(&topic2_button)) {
                topic2_button_press_tick = tick;
                topic2_long_press_handled = 0U;
            }

            if ((button_is_pressed(&topic2_button) != 0U) &&
                (topic2_long_press_handled == 0U) &&
                (topic2_target_laps != 0U) &&
                ((tick - topic2_button_press_tick) >= BUTTON_START_HOLD_MS)) {

                topic2_long_press_handled   = 1U;
                topic2_lap_count            = 0U;
                topic2_start_line_active    = 1U;
                topic2_start_line_confirm   = 0U;
                topic2_start_line_release   = 0U;
                topic2_run_finished         = 0U;
                topic2_run_started          = 1U;
                topic2_last_time_display_ms = 0U;

                xunji_reset_tracking();
                mid_stopwatch_start();
                fishpath_start();

                topic2_oled_update();
                topic2_oled_show_time(0U);
                topic2_oled_dirty = 0U;
                topic2_last_oled_tick = tick;
            }

            if (button_is_released(&topic2_button)) {
                if (topic2_long_press_handled == 0U) {
                    topic2_target_laps++;
                    if (topic2_target_laps > TARGET_LAPS_MAX) {
                        topic2_target_laps = 1U;
                    }
                    topic2_oled_dirty = 1U;
                }
                topic2_long_press_handled = 0U;
            }
        }

        /******************** 到圈停车 ********************/
        if ((topic2_run_started != 0U) &&
            (topic2_run_finished == 0U) &&
            (topic2_lap_count >= topic2_target_laps)) {
            topic2_run_finished = 1U;
            topic2_run_started = 0U;
            topic2_oled_dirty = 1U;
            elapsed_ms = mid_stopwatch_stop();
            fishpath_stop();
            topic2_oled_show_time(elapsed_ms);
        }

        /******************** 计时显示 ********************/
        if (topic2_run_started != 0U) {
            elapsed_ms = mid_stopwatch_get_elapsed_ms();
            if ((elapsed_ms - topic2_last_time_display_ms) >=
                TOPIC2_TIMER_DISPLAY_UPDATE_MS) {
                topic2_last_time_display_ms = elapsed_ms;
                topic2_oled_show_time(elapsed_ms);
            }
        }

        /******************** OLED 状态更新 ********************/
        if ((tick - topic2_last_oled_tick) >= OLED_UPDATE_MS) {
            topic2_last_oled_tick = tick;
            if ((topic2_oled_dirty != 0U) &&
                (topic2_run_started == 0U)) {
                topic2_oled_dirty = 0U;
                topic2_oled_update();
            }
        }
    }
}
