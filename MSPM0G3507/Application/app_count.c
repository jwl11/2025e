#include "app.h"

#include "bsp_OLED.h"
#include "bsp_button.h"
#include "drv_tim.h"
#include "drv_uart.h"
#include "mid_delay.h"


#define APP_TIMER_POLL_MS          5U
#define APP_TIMER_OLED_PERIOD_MS 100U

typedef enum {
    APP_TIMER_READY = 0,
    APP_TIMER_RUNNING,
    APP_TIMER_PAUSED
} AppTimerState;

static Button g_app_timer_button;

static void app_timer_show_time(uint32_t elapsed_ms)
{
    uint32_t total_seconds = elapsed_ms / 1000U;
    uint32_t hours         = (total_seconds / 3600U) % 100U;
    uint32_t minutes       = (total_seconds / 60U) % 60U;
    uint32_t seconds       = total_seconds % 60U;
    uint32_t tenths        = (elapsed_ms / 100U) % 10U;

    OLED_ShowNum(2, 6, hours, 2);
    OLED_ShowNum(2, 9, minutes, 2);
    OLED_ShowNum(2, 12, seconds, 2);
    OLED_ShowNum(2, 15, tenths, 1);
}

static void app_timer_show_state(AppTimerState state)
{
    if (state == APP_TIMER_RUNNING) {
        OLED_ShowString(3, 1, "State: RUNNING  ");
        OLED_ShowString(4, 1, "KEY1: Pause     ");
    } else if (state == APP_TIMER_PAUSED) {
        OLED_ShowString(3, 1, "State: PAUSED   ");
        OLED_ShowString(4, 1, "KEY1: Resume    ");
    } else {
        OLED_ShowString(3, 1, "State: READY    ");
        OLED_ShowString(4, 1, "KEY1: Start     ");
    }
}

void app_oled_timer(void)
{
    AppTimerState state = APP_TIMER_READY;
    uint32_t start_ms = 0U;
    uint32_t elapsed_ms = 0U;
    uint32_t last_oled_ms = 0U;
    uint32_t now;

    /*
     * TIMG6 supplies a stable hardware timebase.  It interrupts only once
     * per 500 ms; drv_timebase_get_ms() interpolates the current timer count.
     */
    drv_timebase_start();

    OLED_Init();
    OLED_Clear();
    OLED_ShowString(1, 1, "OLED Stopwatch  ");
    OLED_ShowString(2, 1, "Time 00:00:00.0 ");
    app_timer_show_state(state);

    button_init(&g_app_timer_button,
                KEY_PORT,
                KEY_KEY1_PIN,
                BUTTON_ACTIVE_HIGH);

    last_oled_ms = drv_timebase_get_ms();

    while (1) {
        now = drv_timebase_get_ms();
        button_update(&g_app_timer_button);

        if (button_is_clicked(&g_app_timer_button)) {
            if (state == APP_TIMER_RUNNING) {
                elapsed_ms += now - start_ms;
                state = APP_TIMER_PAUSED;
            } else {
                start_ms = now;
                state = APP_TIMER_RUNNING;
            }

            app_timer_show_state(state);
            last_oled_ms = now - APP_TIMER_OLED_PERIOD_MS;
        }

        if ((now - last_oled_ms) >= APP_TIMER_OLED_PERIOD_MS) {
            uint32_t displayed_ms = elapsed_ms;

            last_oled_ms = now;
            if (state == APP_TIMER_RUNNING) {
                displayed_ms += now - start_ms;
            }
            app_timer_show_time(displayed_ms);
        }

        delay_ms(APP_TIMER_POLL_MS);
    }
}