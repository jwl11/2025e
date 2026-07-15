#include "ti_msp_dl_config.h"
#include "bsp_fishpath.h"
#include "bsp_encoder.h"
#include "bsp_led.h"
#include "drv_uart.h"
#include "mid_delay.h"
#include "mid_xunji.h"

/* Initial low-sensitivity tracking parameters. */
#define XUNJI_DEFAULT_KP          14.0f
#define XUNJI_DEFAULT_KI           0.0f
#define XUNJI_DEFAULT_KD           5.0f
#define XUNJI_DEFAULT_SPEED       25U

#define XUNJI_CONTROL_PERIOD_MS    5U
#define DEBUG_PRINT_MS           200U

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

void app_fishpath_test(void)
{
    uint32_t last_print_ms;
    uint32_t tick;

    drv_uart_send_string("========================================\r\n");
    drv_uart_send_string("  12-Ch Xunji + MG310 Motor Debug\r\n");
    drv_uart_send_string("  UART1 PA9 RX, PA8 TX @ 115200\r\n");
    drv_uart_send_string("========================================\r\n\r\n");

    fishpath_init(XUNJI_DEFAULT_KP, XUNJI_DEFAULT_KI,
                  XUNJI_DEFAULT_KD, XUNJI_DEFAULT_SPEED);

    drv_uart_send_string("[INIT] Fishpath system ready.\r\n");
    drv_uart_send_string("[INIT] Kp=");
    print_float_3dp(XUNJI_DEFAULT_KP);
    drv_uart_send_string(" Ki=");
    print_float_3dp(XUNJI_DEFAULT_KI);
    drv_uart_send_string(" Kd=");
    print_float_3dp(XUNJI_DEFAULT_KD);
    drv_uart_send_string(" Speed=");
    drv_uart_print_num(XUNJI_DEFAULT_SPEED);
    drv_uart_send_string("%\r\n\r\n");

    /* delay_ms also starts SysTick for get_system_ms(). */
    delay_ms(1U);
    fishpath_print_header();
    last_print_ms = get_system_ms();

    while (1) {
        fishpath_update();
        delay_ms(XUNJI_CONTROL_PERIOD_MS);

        tick = get_system_ms();
        if ((tick - last_print_ms) >= DEBUG_PRINT_MS) {
            last_print_ms = tick;
            fishpath_print_status(tick);
            use_led_TOGGLE();
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
