#include "app.h"
#include "bsp_button.h"
#include "bsp_encoder.h"
#include "bsp_led.h"
#include "drv_uart.h"
#include "mid_delay.h"
#include "mid_vision_line.h"

#define VISION_CONTROL_PERIOD_MS       5U
#define VISION_LINK_TIMEOUT_MS       100U
#define VISION_DEBUG_PERIOD_MS       200U
#define VISION_MIN_CONFIDENCE        300U

#define VISION_BASE_DUTY              25
#define VISION_INTERSECTION_DUTY      15
#define VISION_MAX_DUTY               50
#define VISION_MAX_STEER              28

#define VISION_KP_NUM                 18
#define VISION_KD_NUM                  8
#define VISION_KA_NUM                  3
#define VISION_GAIN_DEN             1000

#define VISION_ACK_MSG_ID            0x81U
#define VISION_ACK_SIZE               8U

#define VISION_ACK_RUNNING            0x01U
#define VISION_ACK_LINK_OK            0x02U
#define VISION_ACK_FRAME_VALID        0x04U

static Button g_vision_button;

static int32_t clamp_i32(int32_t value, int32_t lower, int32_t upper)
{
    if (value < lower) {
        return lower;
    }
    if (value > upper) {
        return upper;
    }
    return value;
}

static void vision_stop_motors(void)
{
    mg310_motorStopAll();
}

static void vision_drive_forward(int32_t left_duty, int32_t right_duty)
{
    left_duty = clamp_i32(left_duty, 0, VISION_MAX_DUTY);
    right_duty = clamp_i32(right_duty, 0, VISION_MAX_DUTY);

    if (left_duty > 0) {
        mg310_motorForward(MG310_MOTOR_A, (uint32_t)left_duty);
    } else {
        mg310_motorStop(MG310_MOTOR_A);
    }

    if (right_duty > 0) {
        mg310_motorForward(MG310_MOTOR_B, (uint32_t)right_duty);
    } else {
        mg310_motorStop(MG310_MOTOR_B);
    }
}

static void write_u16_le(uint8_t *dst, uint16_t value)
{
    dst[0] = (uint8_t)value;
    dst[1] = (uint8_t)(value >> 8U);
}

static void vision_send_ack(uint8_t seq, uint8_t status)
{
    uint8_t ack[VISION_ACK_SIZE];
    uint16_t crc;

    ack[0] = 0xAAU;
    ack[1] = 0x55U;
    ack[2] = VISION_LINE_PROTOCOL_VERSION;
    ack[3] = VISION_ACK_MSG_ID;
    ack[4] = seq;
    ack[5] = status;
    crc = vision_line_crc16_ccitt(ack, 6U);
    write_u16_le(&ack[6], crc);
    (void)drv_vision_uart3_write(ack, VISION_ACK_SIZE);
}

void app_vision_line_test(void)
{
    VisionLineFrame frame = {0};
    uint8_t running = 0U;
    uint8_t have_frame = 0U;
    uint8_t last_seq = 0U;
    uint32_t last_frame_ms = 0U;
    uint32_t last_debug_ms = 0U;
    int16_t previous_error = 0;
    int32_t left_duty = 0;
    int32_t right_duty = 0;

    drv_uart0_init();
    drv_vision_uart3_init();
    vision_line_protocol_init();
    mg310_motorInitAll();
    button_init(&g_vision_button,
                KEY_PORT, KEY_KEY1_PIN, BUTTON_ACTIVE_HIGH);

    delay_ms(1U);

    drv_uart_send_string("\r\n========================================\r\n");
    drv_uart_send_string(" MaixCAM2 Vision Line Tracking / UART3\r\n");
    drv_uart_send_string(" UART3: PB3 RX, PB2 TX, 115200-8N1\r\n");
    drv_uart_send_string(" KEY1: start/stop; timeout: 100ms\r\n");
    drv_uart_send_string("========================================\r\n");

    while (1) {
        uint32_t now = get_system_ms();
        uint8_t status = 0U;

        button_update(&g_vision_button);
        if (button_is_clicked(&g_vision_button) != 0U) {
            running = (running == 0U) ? 1U : 0U;
            previous_error = 0;
            if (running == 0U) {
                vision_stop_motors();
                drv_uart_send_string("[STOP] KEY1\r\n");
            } else {
                drv_uart_send_string("[START] KEY1\r\n");
            }
        }

        while (vision_line_protocol_poll(&frame)) {
            have_frame = 1U;
            last_frame_ms = now;
            last_seq = frame.seq;

            if (running != 0U) {
                status |= VISION_ACK_RUNNING;
            }
            status |= VISION_ACK_LINK_OK;
            if ((frame.flags & VISION_LINE_FLAG_VALID) != 0U) {
                status |= VISION_ACK_FRAME_VALID;
            }
            vision_send_ack(frame.seq, status);
        }

        if (running == 0U) {
            vision_stop_motors();
            left_duty = 0;
            right_duty = 0;
        } else if ((have_frame == 0U) ||
                   ((now - last_frame_ms) > VISION_LINK_TIMEOUT_MS)) {
            vision_stop_motors();
            left_duty = 0;
            right_duty = 0;
        } else if (((frame.flags & VISION_LINE_FLAG_VALID) == 0U) ||
                   (frame.confidence < VISION_MIN_CONFIDENCE)) {
            vision_stop_motors();
            left_duty = 0;
            right_duty = 0;
            previous_error = frame.error_x;
        } else {
            int32_t base_duty =
                ((frame.flags & VISION_LINE_FLAG_INTERSECTION) != 0U) ?
                VISION_INTERSECTION_DUTY : VISION_BASE_DUTY;
            int32_t error_delta =
                (int32_t)frame.error_x - (int32_t)previous_error;
            int32_t steer =
                ((int32_t)frame.error_x * VISION_KP_NUM +
                 error_delta * VISION_KD_NUM +
                 (int32_t)frame.angle_cdeg * VISION_KA_NUM) /
                VISION_GAIN_DEN;

            steer = clamp_i32(steer, -VISION_MAX_STEER, VISION_MAX_STEER);
            left_duty = base_duty + steer;
            right_duty = base_duty - steer;
            vision_drive_forward(left_duty, right_duty);
            previous_error = frame.error_x;
        }

        if ((now - last_debug_ms) >= VISION_DEBUG_PERIOD_MS) {
            last_debug_ms = now;
            drv_uart_send_string("SEQ=");
            drv_uart_print_num(last_seq);
            drv_uart_send_string(" RUN=");
            drv_uart_print_num(running);
            drv_uart_send_string(" V=");
            drv_uart_print_num(
                (frame.flags & VISION_LINE_FLAG_VALID) ? 1U : 0U);
            drv_uart_send_string(" E=");
            drv_uart_print_signed(frame.error_x);
            drv_uart_send_string(" A=");
            drv_uart_print_signed(frame.angle_cdeg);
            drv_uart_send_string(" C=");
            drv_uart_print_num(frame.confidence);
            drv_uart_send_string(" L=");
            drv_uart_print_signed(left_duty);
            drv_uart_send_string(" R=");
            drv_uart_print_signed(right_duty);
            drv_uart_send_string(" CRC=");
            drv_uart_print_num(vision_line_get_crc_error_count());
            drv_uart_send_string(" FMT=");
            drv_uart_print_num(vision_line_get_format_error_count());
            drv_uart_send_string(" OVF=");
            drv_uart_print_num(drv_vision_uart3_get_overflow_count());
            drv_uart_send_string("\r\n");
            use_led_TOGGLE();
        }

        delay_ms(VISION_CONTROL_PERIOD_MS);
    }
}
