#include "ti_msp_dl_config.h"
#include "bsp_motor_ctrl.h"
#include "bsp_led.h"
#include "drv_uart.h"
#include "mid_delay.h"

/* ================================================================
 * MG310 编码电机闭环控制测试 (Application 层)
 *
 * Phase 1 — 速度闭环: +200 → +400 → +100 → 0 → -200 → -400 → STOP
 * Phase 2 — 位置闭环: +500 → -500 → 0 pulse (级联)
 *
 * 控制周期: 50 ms，与 MOTOR_MG310/TIMA1 测速周期一致
 * 调试打印: 每 200ms
 *
 * 分层调用:
 *   app_motor_ctrl_test
 *     ├── motor_ctrl_init/set_speed/set_position/update  [BSP]
 *     │     ├── encoder_update  [Driver]
 *     │     ├── pid_update      [Middleware]
 *     │     └── mg310_motor*    [BSP]
 *     └── drv_uart_* / delay_ms [Driver/Middleware]
 * ================================================================ */

#define CTRL_PERIOD_MS    50U
#define PHASE_HOLD_MS     3000U
#define PRINT_MS          200U
#define POWER_STAGE_TEST_DUTY    30U
#define POWER_STAGE_TEST_US      800000U
#define TIMER_CHECK_DELAY_US     200000U

static MotorController g_mc_a;
static MotorController g_mc_b;

static void print_sign(long v)
{
    if (v < 0) { drv_uart_send_string("-"); v = -v; }
    else       { drv_uart_send_string("+"); }
    drv_uart_print_num((unsigned long)v);
}

static void print_status(uint32_t tick, const char *label)
{
    drv_uart_print_num((unsigned long)tick);
    drv_uart_send_string(" | ");
    drv_uart_send_string((char *)label);
    drv_uart_send_string(" | A pos=");
    print_sign((long)motor_ctrl_get_position(&g_mc_a));
    drv_uart_send_string(" spd=");
    print_sign((long)motor_ctrl_get_speed(&g_mc_a));
    drv_uart_send_string(" duty=");
    print_sign((long)motor_ctrl_get_duty(&g_mc_a));
    drv_uart_send_string(" fault=");
    drv_uart_print_num((unsigned long)motor_ctrl_get_fault(&g_mc_a));
    drv_uart_send_string(" | B pos=");
    print_sign((long)motor_ctrl_get_position(&g_mc_b));
    drv_uart_send_string(" spd=");
    print_sign((long)motor_ctrl_get_speed(&g_mc_b));
    drv_uart_send_string(" duty=");
    print_sign((long)motor_ctrl_get_duty(&g_mc_b));
    drv_uart_send_string(" fault=");
    drv_uart_print_num((unsigned long)motor_ctrl_get_fault(&g_mc_b));
    drv_uart_send_string("\r\n");
}

static void update_both_motors(void)
{
    motor_ctrl_update(&g_mc_a);
    motor_ctrl_update(&g_mc_b);

    /* A fault on either wheel stops the complete two-wheel drive. */
    if ((motor_ctrl_get_fault(&g_mc_a) != MOTOR_CTRL_FAULT_NONE) ||
        (motor_ctrl_get_fault(&g_mc_b) != MOTOR_CTRL_FAULT_NONE)) {
        motor_ctrl_stop(&g_mc_a);
        motor_ctrl_stop(&g_mc_b);
    }
}

static void set_both_speed(int32_t speed_mm_s)
{
    motor_ctrl_set_speed(&g_mc_a, speed_mm_s);
    motor_ctrl_set_speed(&g_mc_b, speed_mm_s);
}

static void set_both_position(int32_t position_pulses)
{
    motor_ctrl_set_position(&g_mc_a, position_pulses);
    motor_ctrl_set_position(&g_mc_b, position_pulses);
}

static void stop_both_motors(void)
{
    motor_ctrl_stop(&g_mc_a);
    motor_ctrl_stop(&g_mc_b);
}

static bool speed_timer_is_running(void)
{
    return (encoder_get_speed_sample_sequence(&g_mc_a.encoder) != 0U) &&
           (encoder_get_speed_sample_sequence(&g_mc_b.encoder) != 0U);
}

/*
 * Directly verifies TIMG0 PWM and the direction pins. This test does not
 * depend on encoder feedback, TIMA1 or the closed-loop controller.
 */
static void power_stage_self_test(void)
{
    drv_uart_send_string("[SELFTEST] Both wheels: forward 30%, 0.8 second.\r\n");
    use_led_ON();
    mg310_motorForward(MG310_MOTOR_A, POWER_STAGE_TEST_DUTY);
    mg310_motorForward(MG310_MOTOR_B, POWER_STAGE_TEST_DUTY);
    delay_us(POWER_STAGE_TEST_US);
    mg310_motorStopAll();
    use_led_OFF();
    drv_uart_send_string("[SELFTEST] PWM stopped.\r\n");
}

void app_motor_ctrl_test(void)
{
    uint32_t tick, phase_tick, print_tick;
    uint8_t  phase, step;
    bool timer_ready;

    /* UART transmission has a timeout and cannot block motor startup. */
    use_led_ON();
    drv_uart0_init();
    drv_uart_send_string("\r\n[BOOT] app_motor_ctrl_test entered.\r\n");

    /*
     * Run the power-stage check before encoder/TIMA1 IRQs are enabled.
     * Therefore an encoder interrupt problem cannot leave PWM running.
     */
    mg310_motorInitAll();
    power_stage_self_test();

    drv_uart_send_string("[INIT] Enabling encoder and TIMA1 interrupts.\r\n");
    motor_ctrl_init(&g_mc_a, MG310_MOTOR_A);
    motor_ctrl_init(&g_mc_b, MG310_MOTOR_B);
    drv_uart_send_string("[INIT] Encoder and TIMA1 setup returned.\r\n");

    /* Wait four 50 ms periods without relying on SysTick. */
    delay_us(TIMER_CHECK_DELAY_US);
    timer_ready = speed_timer_is_running();

    drv_uart_send_string("========================================\r\n");
    drv_uart_send_string(" MG310 Closed-Loop Motor Test\r\n");
    drv_uart_send_string(" Speed PID + Position PID (Cascaded)\r\n");
    drv_uart_send_string(" Enc: E1A=PB13 E1B=PA26 | E2A=PB10 E2B=PB11\r\n");
    drv_uart_send_string(" Decode: GPIO A rising edge + B direction, 1x\r\n");
    drv_uart_send_string(" Speed tick: TIMA1 LOAD, 50 ms\r\n");
    drv_uart_send_string(" Wheel: D=48 mm, encoder=260 count/rev\r\n");
    drv_uart_send_string(" Debug: UART0 TX=PA10 RX=PA11, 115200-8N1\r\n");
    drv_uart_send_string("========================================\r\n\r\n");

    drv_uart_send_string("[INIT] Motors A and B ready.\r\n");
    drv_uart_send_string("[INIT] Speed PI: Kp=0.05 Ki=0.01, duty<=40%\r\n");
    drv_uart_send_string("[INIT] Start boost=20%\r\n");
    drv_uart_send_string("[INIT] Filter=4; fault 1=stall, 2=direction\r\n");
    drv_uart_send_string("[INIT] Pos   PID: Kp=2.0 Ki=0.01 Kd=0.5\r\n\r\n");
    if (!timer_ready) {
        drv_uart_send_string("[FATAL] TIMA1 50ms interrupt not running!\r\n");
        stop_both_motors();
        while (1) {
            use_led_TOGGLE();
            delay_ms(100U);
        }
    }
    use_led_OFF();
    drv_uart_send_string("[OK] TIMA1 speed timer is running.\r\n\r\n");

    /* ============================================================
     *  Phase 1 — 速度闭环
     * ============================================================ */
    drv_uart_send_string("--- Phase 1: Speed Control ---\r\n");
    drv_uart_send_string("Time(ms) | Mode   | Position  Speed   Duty\r\n");
    drv_uart_send_string("---------+--------+----------------------\r\n");

    phase     = 1;
    step      = 0;
    phase_tick = get_system_ms();
    print_tick = phase_tick;

    set_both_speed(200);
    drv_uart_send_string("[SPD] target = +200 mm/s\r\n");

    while (phase == 1) {
        update_both_motors();
        delay_ms(CTRL_PERIOD_MS);
        tick = get_system_ms();

        if ((tick - print_tick) >= PRINT_MS) {
            print_tick = tick;
            print_status(tick, "SPD ");
            use_led_TOGGLE();
        }

        if ((tick - phase_tick) >= PHASE_HOLD_MS) {
            step++;
            phase_tick = tick;
            switch (step) {
            case 1: set_both_speed(400);
                    drv_uart_send_string("[SPD] target = +400 mm/s\r\n"); break;
            case 2: set_both_speed(100);
                    drv_uart_send_string("[SPD] target = +100 mm/s\r\n"); break;
            case 3: set_both_speed(0);
                    drv_uart_send_string("[SPD] target = 0 mm/s\r\n"); break;
            case 4: set_both_speed(-200);
                    drv_uart_send_string("[SPD] target = -200 mm/s\r\n"); break;
            case 5: set_both_speed(-400);
                    drv_uart_send_string("[SPD] target = -400 mm/s\r\n"); break;
            default:
                    stop_both_motors();
                    drv_uart_send_string("[SPD] STOP\r\n");
                    phase = 2; break;
            }
        }
    }

    delay_ms(500);

    /* ============================================================
     *  Phase 2 — 位置闭环 (级联)
     * ============================================================ */
    drv_uart_send_string("\r\n--- Phase 2: Position Control ---\r\n");
    encoder_reset_count(&g_mc_a.encoder);
    encoder_reset_count(&g_mc_b.encoder);

    phase     = 2;
    step      = 0;
    phase_tick = get_system_ms();
    print_tick = phase_tick;

    set_both_position(500);
    drv_uart_send_string("[POS] target = +500 pulse\r\n");

    while (phase == 2) {
        update_both_motors();
        delay_ms(CTRL_PERIOD_MS);
        tick = get_system_ms();

        if ((tick - print_tick) >= PRINT_MS) {
            print_tick = tick;
            print_status(tick, "POS ");
            use_led_TOGGLE();
        }

        if ((tick - phase_tick) >= PHASE_HOLD_MS) {
            step++;
            phase_tick = tick;
            switch (step) {
            case 1: set_both_position(-500);
                    drv_uart_send_string("[POS] target = -500 pulse\r\n"); break;
            case 2: set_both_position(0);
                    drv_uart_send_string("[POS] target = 0 pulse\r\n"); break;
            default:
                    stop_both_motors();
                    drv_uart_send_string("[POS] STOP\r\n");
                    phase = 3; break;
            }
        }
    }

    drv_uart_send_string("\r\n========================================\r\n");
    drv_uart_send_string(" Motor Control Test Complete!\r\n");
    drv_uart_send_string("========================================\r\n");

    while (1) { use_led_TOGGLE(); delay_ms(500); }
}

/* ================================================================
 * Position-loop-only test
 *
 * 260 encoder counts are treated as approximately one wheel revolution.
 * Both motors follow the absolute sequence +260 -> 0 -> -260 -> 0.
 * ================================================================ */
void app_motor_position_test(void)
{
    static const int32_t position_targets[] = {
        260, 0, -260, 0
    };
    const uint32_t target_hold_ms = 4000U;
    const uint32_t target_count =
        sizeof(position_targets) / sizeof(position_targets[0]);
    uint32_t target_index;
    uint32_t target_start_ms;
    uint32_t print_ms;
    uint32_t now_ms;

    use_led_ON();
    drv_uart0_init();
    drv_uart_send_string("\r\n========================================\r\n");
    drv_uart_send_string(" MG310 Position-Loop-Only Test\r\n");
    drv_uart_send_string(" Sequence: +260 -> 0 -> -260 -> 0 count\r\n");
    drv_uart_send_string(" UART0: PA10 TX, 115200-8N1\r\n");
    drv_uart_send_string("========================================\r\n");

    motor_ctrl_init(&g_mc_a, MG310_MOTOR_A);
    motor_ctrl_init(&g_mc_b, MG310_MOTOR_B);

    /* Allow at least four TIMA1 50 ms periods without depending on SysTick. */
    delay_us(TIMER_CHECK_DELAY_US);
    if (!speed_timer_is_running()) {
        stop_both_motors();
        drv_uart_send_string("[FATAL] TIMA1 50ms interrupt not running!\r\n");
        while (1) {
            use_led_TOGGLE();
            delay_ms(100U);
        }
    }

    encoder_reset_count(&g_mc_a.encoder);
    encoder_reset_count(&g_mc_b.encoder);
    delay_ms(1U); /* Start the millisecond time base. */
    use_led_OFF();

    drv_uart_send_string("[OK] Position test started; 260 count ~= 1 turn.\r\n");
    drv_uart_send_string("Time(ms) | Mode | A/B position, speed, duty, fault\r\n");

    for (target_index = 0U; target_index < target_count; target_index++) {
        set_both_position(position_targets[target_index]);
        drv_uart_send_string("\r\n[POS] target = ");
        print_sign((long)position_targets[target_index]);
        drv_uart_send_string(" count\r\n");

        target_start_ms = get_system_ms();
        print_ms = target_start_ms;

        while ((get_system_ms() - target_start_ms) < target_hold_ms) {
            update_both_motors();
            delay_ms(CTRL_PERIOD_MS);
            now_ms = get_system_ms();

            if ((now_ms - print_ms) >= PRINT_MS) {
                print_ms = now_ms;
                print_status(now_ms, "POS ");
                use_led_TOGGLE();
            }

            if ((motor_ctrl_get_fault(&g_mc_a) != MOTOR_CTRL_FAULT_NONE) ||
                (motor_ctrl_get_fault(&g_mc_b) != MOTOR_CTRL_FAULT_NONE)) {
                stop_both_motors();
                drv_uart_send_string("[FAULT] Position test stopped.\r\n");
                while (1) {
                    use_led_TOGGLE();
                    delay_ms(100U);
                }
            }
        }
    }

    stop_both_motors();
    drv_uart_send_string("\r\n[PASS] Position-loop test complete; motors stopped.\r\n");

    while (1) {
        use_led_TOGGLE();
        delay_ms(500U);
    }
}
