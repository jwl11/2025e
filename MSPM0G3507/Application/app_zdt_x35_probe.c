#include "app.h"

#include "bsp_zdt_x35.h"
#include "drv_uart.h"
#include "drv_zdt_x35_uart.h"
#include "mid_delay.h"

/*
 * X35_V1.3 + Emm_V5.0自动运动测试程序。
 *
 * 安全策略：
 *   1. 上电先查询版本，必须收到01 1F 05 82 6B才开始运动；
 *   2. 自动测试只执行一遍，完成后失能并停在结束状态；
 *   3. 速度模式只运行1秒，之后自动发送立即停止；
 *   4. 位置测试只走100脉冲，并用反向100脉冲返回；
 *   5. 收到条件不满足回复时立即停止并失能。
 */

#define ZDT_X35_QUERY_INTERVAL_MS 1000U
#define ZDT_X35_TEST_SPEED_RPM    60U
#define ZDT_X35_TEST_ACC          10U
#define ZDT_X35_TEST_RUN_MS       1000U
#define ZDT_X35_TEST_PULSES       100U
#define ZDT_X35_TEST_GAP_MS        500U
#define ZDT_X35_TEST_POSITION_MS  1000U

typedef enum {
    ZDT_X35_TEST_WAIT_VERSION = 0,
    ZDT_X35_TEST_WAIT_AFTER_ENABLE,
    ZDT_X35_TEST_VELOCITY_RUNNING,
    ZDT_X35_TEST_WAIT_AFTER_STOP,
    ZDT_X35_TEST_WAIT_AFTER_CW_POSITION,
    ZDT_X35_TEST_WAIT_AFTER_CCW_POSITION,
    ZDT_X35_TEST_DONE,
    ZDT_X35_TEST_ERROR,
} ZdtX35TestPhase;

static void zdt_x35_print_frame(const ZdtX35ProbeState *state)
{
    uint8_t i;

    drv_uart_send_string("[ZDT_X35 RX]");
    for (i = 0U; i < state->length; i++) {
        drv_uart_send_string(" ");
        drv_uart_print_hex(state->data[i]);
    }
    drv_uart_send_string("\r\n");

    if (state->is_version_response) {
        drv_uart_send_string("[ZDT_X35] FW_CODE=0x");
        drv_uart_print_hex(state->firmware_version);
        drv_uart_send_string(" HW_CODE=0x");
        drv_uart_print_hex(state->hardware_version);

        if ((state->firmware_version == 0x05U) &&
            (state->hardware_version == 0x82U)) {
            drv_uart_send_string(
                " => Emm_V5 family + X35_V1.3 confirmed\r\n");
        } else {
            drv_uart_send_string(" => unrecognized combination\r\n");
        }
    }

    if (state->is_command_response) {
        drv_uart_send_string("[ZDT_X35 ACK] CMD=0x");
        drv_uart_print_hex(state->command);
        drv_uart_send_string(" STATUS=0x");
        drv_uart_print_hex(state->command_status);

        if (state->command_status == 0x02U) {
            drv_uart_send_string(" (accepted)\r\n");
        } else if (state->command_status == 0xE2U) {
            drv_uart_send_string(
                " (condition failed: check enable/stall)\r\n");
        } else {
            drv_uart_send_string(" (unknown)\r\n");
        }
    }

    drv_uart_send_string("[ZDT_X35] RX_OK=");
    drv_uart_print_num(state->valid_frame_count);
    drv_uart_send_string(" RX_ERR=");
    drv_uart_print_num(state->invalid_frame_count);
    drv_uart_send_string(" OVF=");
    drv_uart_print_num(drv_zdt_x35_uart_get_overflow_count());
    drv_uart_send_string("\r\n");
}

static void zdt_x35_send_version_query(void)
{
    ZdtX35Result result = zdt_x35_request_version();

    drv_uart_send_string("[ZDT_X35 TX] 01 1F 6B ");
    drv_uart_send_string(
        result == ZDT_X35_RESULT_OK ? "OK\r\n" : "UART FAIL\r\n");
}

static void zdt_x35_print_send_result(
    const char *action,
    ZdtX35Result result)
{
    drv_uart_send_string("[ZDT_X35 TEST] ");
    drv_uart_send_string(action);

    if (result == ZDT_X35_RESULT_OK) {
        drv_uart_send_string(" SENT\r\n");
    } else if (result == ZDT_X35_RESULT_BAD_ARGUMENT) {
        drv_uart_send_string(" BAD ARGUMENT\r\n");
    } else {
        drv_uart_send_string(" UART FAIL\r\n");
    }
}

static bool zdt_x35_test_send(
    const char *action,
    ZdtX35Result result)
{
    zdt_x35_print_send_result(action, result);
    return result == ZDT_X35_RESULT_OK;
}

void app_zdt_x35_motion_test(void)
{
    uint32_t last_query_ms;
    uint32_t phase_start_ms = 0U;
    ZdtX35TestPhase phase = ZDT_X35_TEST_WAIT_VERSION;

    drv_uart0_init();
    zdt_x35_init();

    drv_uart_send_string("\r\n========================================\r\n");
    drv_uart_send_string(" X35_V1.3 + Emm_V5.0 Auto Motion Test\r\n");
    drv_uart_send_string(" UART2: PA23 TX / PA24 RX / 115200-8N1\r\n");
    drv_uart_send_string(" Motor: TTL / address=1 / checksum=0x6B\r\n");
    drv_uart_send_string(" Auto: enable -> 60RPM -> stop -> +100 -> -100 -> disable\r\n");
    drv_uart_send_string("========================================\r\n");

    zdt_x35_send_version_query();
    last_query_ms = get_system_ms();

    while (1) {
        const ZdtX35ProbeState *state;
        uint32_t now_ms = get_system_ms();

        zdt_x35_poll();
        state = zdt_x35_get_probe_state();

        if (state->ready) {
            zdt_x35_print_frame(state);

            if (state->is_version_response &&
                (state->firmware_version == 0x05U) &&
                (state->hardware_version == 0x82U)) {
                if (phase == ZDT_X35_TEST_WAIT_VERSION) {
                    drv_uart_send_string(
                        "[ZDT_X35 TEST] Version confirmed, auto test starts.\r\n");

                    if (zdt_x35_test_send(
                            "enable",
                            zdt_x35_set_enable(true, false))) {
                        phase = ZDT_X35_TEST_WAIT_AFTER_ENABLE;
                        phase_start_ms = now_ms;
                    } else {
                        phase = ZDT_X35_TEST_ERROR;
                    }
                }
            }

            /*
             * 0xE2表示未使能、堵转等条件不满足。
             * 测试中收到该状态便立即发送停止和失能，不继续后续动作。
             */
            if (state->is_command_response &&
                (state->command_status != 0x02U) &&
                (phase != ZDT_X35_TEST_DONE) &&
                (phase != ZDT_X35_TEST_ERROR)) {
                drv_uart_send_string(
                    "[ZDT_X35 TEST] Command rejected, aborting test.\r\n");
                (void)zdt_x35_stop(false);
                (void)zdt_x35_set_enable(false, false);
                phase = ZDT_X35_TEST_ERROR;
            }

            zdt_x35_consume_frame();
        }

        switch (phase) {
        case ZDT_X35_TEST_WAIT_AFTER_ENABLE:
            if ((now_ms - phase_start_ms) >= ZDT_X35_TEST_GAP_MS) {
                if (zdt_x35_test_send(
                        "CW 60RPM",
                        zdt_x35_run_velocity(
                            ZDT_X35_DIRECTION_CW,
                            ZDT_X35_TEST_SPEED_RPM,
                            ZDT_X35_TEST_ACC,
                            false))) {
                    phase = ZDT_X35_TEST_VELOCITY_RUNNING;
                    phase_start_ms = now_ms;
                } else {
                    phase = ZDT_X35_TEST_ERROR;
                }
            }
            break;

        case ZDT_X35_TEST_VELOCITY_RUNNING:
            if ((now_ms - phase_start_ms) >= ZDT_X35_TEST_RUN_MS) {
                if (zdt_x35_test_send(
                        "automatic immediate stop",
                        zdt_x35_stop(false))) {
                    phase = ZDT_X35_TEST_WAIT_AFTER_STOP;
                    phase_start_ms = now_ms;
                } else {
                    phase = ZDT_X35_TEST_ERROR;
                }
            }
            break;

        case ZDT_X35_TEST_WAIT_AFTER_STOP:
            if ((now_ms - phase_start_ms) >= ZDT_X35_TEST_GAP_MS) {
                if (zdt_x35_test_send(
                        "relative CW 100 pulses",
                        zdt_x35_move_position(
                            ZDT_X35_DIRECTION_CW,
                            ZDT_X35_TEST_SPEED_RPM,
                            ZDT_X35_TEST_ACC,
                            ZDT_X35_TEST_PULSES,
                            ZDT_X35_POSITION_RELATIVE,
                            false))) {
                    phase = ZDT_X35_TEST_WAIT_AFTER_CW_POSITION;
                    phase_start_ms = now_ms;
                } else {
                    phase = ZDT_X35_TEST_ERROR;
                }
            }
            break;

        case ZDT_X35_TEST_WAIT_AFTER_CW_POSITION:
            if ((now_ms - phase_start_ms) >= ZDT_X35_TEST_POSITION_MS) {
                if (zdt_x35_test_send(
                        "relative CCW 100 pulses",
                        zdt_x35_move_position(
                            ZDT_X35_DIRECTION_CCW,
                            ZDT_X35_TEST_SPEED_RPM,
                            ZDT_X35_TEST_ACC,
                            ZDT_X35_TEST_PULSES,
                            ZDT_X35_POSITION_RELATIVE,
                            false))) {
                    phase = ZDT_X35_TEST_WAIT_AFTER_CCW_POSITION;
                    phase_start_ms = now_ms;
                } else {
                    phase = ZDT_X35_TEST_ERROR;
                }
            }
            break;

        case ZDT_X35_TEST_WAIT_AFTER_CCW_POSITION:
            if ((now_ms - phase_start_ms) >= ZDT_X35_TEST_POSITION_MS) {
                if (zdt_x35_test_send(
                        "disable",
                        zdt_x35_set_enable(false, false))) {
                    phase = ZDT_X35_TEST_DONE;
                    drv_uart_send_string(
                        "[ZDT_X35 TEST] Auto test completed.\r\n");
                } else {
                    phase = ZDT_X35_TEST_ERROR;
                }
            }
            break;

        case ZDT_X35_TEST_WAIT_VERSION:
            /* 没收到正确版本时每秒重试，电机不会执行运动命令。 */
            if ((now_ms - last_query_ms) >= ZDT_X35_QUERY_INTERVAL_MS) {
                zdt_x35_send_version_query();
                last_query_ms = now_ms;
            }
            break;

        case ZDT_X35_TEST_DONE:
        case ZDT_X35_TEST_ERROR:
        default:
            /* 完成或异常后保持静止，不自动重复测试。 */
            break;
        }

        delay_ms(5U);
    }
}

/*
 * 保留旧函数名，避免其他测试入口引用失效。
 * 当前会进入上面的自动运动测试，不再是纯只读探测。
 */
void app_zdt_x35_probe(void)
{
    app_zdt_x35_motion_test();
}
