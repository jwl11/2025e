#include "app.h"
#include "bsp_42step.h"
#include "bsp_OLED.h"
#include "drv_uart.h"
#include "mid_delay.h"

#include <stdbool.h>
#include <stdint.h>

/* ================================================================
 * 第三题最终实机版：O -> +5 cm -> -5 cm
 *
 * 本文件移植自电脑端 pc_serial_topic3.py 的最终参数：
 *   1. 直接使用 MaixCAM2 原始 x 坐标，避免 0.1 cm 换算损失速度信息；
 *   2. +5 cm 到达合格区后立即反向，不在中途停留；
 *   3. -5 cm 的判题中心仍为 x=705，轨迹瞄准和终点保持均为 x=734；
 *   4. 到 x=705 后关闭位置增力，只用低通速度进行刹车；
 *   5. 连续稳定 500 ms 后回水平并停止闭环。
 *
 * MaixCAM2 串口协议仍为：@x,y#，UART3，115200-8N1。
 * ZDT 使用 UART2 的 Emm_V5.0 绝对位置模式。
 * ================================================================ */

#define BALL_O_X                         560
#define BALL_POSITIVE_TARGET_X           415
#define BALL_NEGATIVE_SCORE_X            705
#define BALL_NEGATIVE_CONTROL_X          734
#define BALL_FINAL_HOLD_X                734
#define BALL_SCORE_TOLERANCE_PX            29U
#define BALL_FINAL_TOLERANCE_PX              6U

#define START_HOLD_MS                     300U
#define TERMINAL_HOLD_MS                  500U
#define CAMERA_TIMEOUT_MS                 300U
#define MOTOR_KEEPALIVE_MS                500U
#define TOPIC_TIME_LIMIT_MS              5000U

/* 当前实机固定水平原点。驱动器重新清零或机械结构改变后需要重新标定。 */
#define LEVEL_MOTOR_POSITION               (0L)
#define POSITION_SPEED                   1500U
#define POSITION_ACC                      255U
#define CONTROL_MAX_OFFSET                500L

/* 电脑端最终控制参数。 */
#define CONTROL_KP                       12.0f
#define CONTROL_KI_NEAR                   0.1f
#define CONTROL_KD                        2.0f
#define CONTROL_GRAVITY_FF                1.0f
#define CONTROL_DAMP_FAR                  8.0f
#define CONTROL_DAMP_NEAR_POSITIVE        0.0f
#define CONTROL_DAMP_NEAR_NEGATIVE       60.0f
#define CONTROL_DAMP_LIMIT              180.0f
#define CONTROL_DEADBAND_X10                2
#define CONTROL_NEAR_POSITIVE_X10          48
#define CONTROL_NEAR_NEGATIVE_X10          65
#define CONTROL_INTEGRAL_LIMIT           150.0f
#define CONTROL_PID_LIMIT               1000.0f

#define VELOCITY_ALPHA                    0.35f
#define VELOCITY_RELEASE_PX               0.75f
#define MOMENTUM_RELEASE_PX                  3
#define MOTION_WINDOW_SIZE                   5U
#define MOTION_RESET_PX                      6

#define FAR_MIN_POSITIVE                    85L
#define FAR_MIN_NEGATIVE                    60L
#define NEAR_MIN_POSITIVE                   45L
#define NEAR_MIN_NEGATIVE                   75L
#define FAR_BOOST_STEP_MS                  300U
#define FAR_BOOST_INCREMENT                 20L
#define FAR_BOOST_MAX                      120L
#define NEAR_BOOST_STEP_MS                 100U
#define NEAR_BOOST_INCREMENT                 5L
#define NEAR_BOOST_MAX                      45L

#define TERMINAL_SPEED_THRESHOLD_PX       0.12f
#define TERMINAL_BRAKE_LIMIT                45L
#define SETTLE_SAMPLE_CAPACITY               32U

#define FINAL_CONTROL_ENTRY_X               690
#define FINAL_HOLD_KP                       1.1f
#define FINAL_HOLD_KD                      12.0f
#define FINAL_HOLD_BIAS                     5.0f
#define FINAL_HOLD_INTEGRAL_STEP            0.03f
#define FINAL_HOLD_INTEGRAL_LIMIT          22.0f
#define FINAL_HOLD_LIMIT                   52.0f
#define FINAL_HOLD_SLEW_PER_FRAME           2.0f

typedef enum {
    TOPIC3_WAIT_O = 0,
    TOPIC3_TO_POSITIVE,
    TOPIC3_TO_NEGATIVE,
    TOPIC3_DONE
} Topic3Stage;

typedef struct {
    float integral;
    float previous_pid_error;
    int16_t previous_error_x10;
    int16_t previous_x;
    float filtered_pixel_delta;
    int16_t recent_x[MOTION_WINDOW_SIZE];
    uint8_t recent_count;
    int16_t motion_anchor_x;
    uint32_t motion_stalled_since_ms;
    int32_t motion_boost;
    bool previous_x_valid;
    bool motion_anchor_valid;
} Topic3Controller;

typedef struct {
    uint32_t time_ms[SETTLE_SAMPLE_CAPACITY];
    int16_t x[SETTLE_SAMPLE_CAPACITY];
    uint8_t count;
} Topic3SettleWindow;

typedef struct {
    int16_t previous_x;
    float filtered_pixel_delta;
    float integral;
    float command;
    bool active;
} Topic3FinalHold;

static int32_t g_last_motor_position;
static uint32_t g_last_motor_send_ms;
static bool g_last_motor_position_valid;

static int32_t clamp_i32(int32_t value, int32_t minimum, int32_t maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

static float clamp_f32(float value, float minimum, float maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

static uint32_t abs_i32(int32_t value)
{
    return (uint32_t)((value < 0) ? -value : value);
}

static float abs_f32(float value)
{
    return (value < 0.0f) ? -value : value;
}

static void final_hold_reset(Topic3FinalHold *hold, int16_t x,
                             float command, float filtered_pixel_delta)
{
    hold->previous_x = x;
    hold->filtered_pixel_delta = filtered_pixel_delta;
    hold->integral = 0.0f;
    hold->command = command;
    hold->active = true;
}

static int32_t final_hold_update(Topic3FinalHold *hold, int16_t x)
{
    int16_t pixel_delta = (int16_t)(x - hold->previous_x);
    int16_t position_error = (int16_t)(BALL_FINAL_HOLD_X - x);
    bool moving_to_target;
    float desired;

    hold->previous_x = x;
    hold->filtered_pixel_delta += VELOCITY_ALPHA *
        ((float)pixel_delta - hold->filtered_pixel_delta);
    moving_to_target = ((float)position_error *
                        hold->filtered_pixel_delta > 0.0f);
    if ((abs_i32(position_error) <= BALL_FINAL_TOLERANCE_PX) ||
        moving_to_target) {
        hold->integral = 0.0f;
    } else {
        hold->integral += FINAL_HOLD_INTEGRAL_STEP *
                          (float)position_error;
        hold->integral = clamp_f32(hold->integral,
                                    -FINAL_HOLD_INTEGRAL_LIMIT,
                                    FINAL_HOLD_INTEGRAL_LIMIT);
    }
    desired = FINAL_HOLD_BIAS +
              FINAL_HOLD_KP * (float)position_error +
              hold->integral -
              FINAL_HOLD_KD * hold->filtered_pixel_delta;
    desired = clamp_f32(desired, -FINAL_HOLD_LIMIT, FINAL_HOLD_LIMIT);
    hold->command = clamp_f32(desired,
                              hold->command - FINAL_HOLD_SLEW_PER_FRAME,
                              hold->command + FINAL_HOLD_SLEW_PER_FRAME);
    hold->command = clamp_f32(hold->command,
                              -FINAL_HOLD_LIMIT, FINAL_HOLD_LIMIT);
    return (int32_t)(hold->command +
                     ((hold->command >= 0.0f) ? 0.5f : -0.5f));
}

static bool time_reached(uint32_t now_ms, uint32_t deadline_ms)
{
    return ((int32_t)(now_ms - deadline_ms) >= 0);
}

static void settle_reset(Topic3SettleWindow *window)
{
    window->count = 0U;
}

/* 整个时间窗都必须在容差内，同时限制窗口内跨度和首尾漂移。 */
static bool settle_update(Topic3SettleWindow *window,
                          uint32_t now_ms,
                          int16_t x,
                          int16_t target_x,
                          uint16_t tolerance_px,
                          uint32_t hold_ms,
                          uint16_t span_px,
                          uint16_t drift_px)
{
    uint8_t i;
    int16_t minimum_x;
    int16_t maximum_x;

    if (abs_i32((int32_t)x - target_x) > tolerance_px) {
        settle_reset(window);
        return false;
    }

    if (window->count >= SETTLE_SAMPLE_CAPACITY) {
        for (i = 1U; i < window->count; i++) {
            window->time_ms[i - 1U] = window->time_ms[i];
            window->x[i - 1U] = window->x[i];
        }
        window->count--;
    }
    window->time_ms[window->count] = now_ms;
    window->x[window->count] = x;
    window->count++;

    /* 保留恰好覆盖 hold_ms 的最早样本，与电脑端稳定窗逻辑一致。 */
    while ((window->count > 1U) &&
           ((uint32_t)(now_ms - window->time_ms[1]) >= hold_ms)) {
        for (i = 1U; i < window->count; i++) {
            window->time_ms[i - 1U] = window->time_ms[i];
            window->x[i - 1U] = window->x[i];
        }
        window->count--;
    }

    if ((window->count == 0U) ||
        ((uint32_t)(now_ms - window->time_ms[0]) < hold_ms)) {
        return false;
    }

    minimum_x = window->x[0];
    maximum_x = window->x[0];
    for (i = 1U; i < window->count; i++) {
        if (window->x[i] < minimum_x) minimum_x = window->x[i];
        if (window->x[i] > maximum_x) maximum_x = window->x[i];
    }

    return (((uint16_t)(maximum_x - minimum_x) <= span_px) &&
            (abs_i32((int32_t)window->x[window->count - 1U] -
                     window->x[0]) <= drift_px));
}

static void controller_prime(Topic3Controller *controller,
                             int16_t x,
                             int16_t target_x,
                             uint32_t now_ms)
{
    int16_t error_x10 = (int16_t)(((int32_t)x - target_x) * 10L / 29L);

    controller->integral = 0.0f;
    controller->previous_pid_error = (float)error_x10 / 10.0f;
    controller->previous_error_x10 = error_x10;
    controller->previous_x = x;
    controller->filtered_pixel_delta = 0.0f;
    controller->recent_x[0] = x;
    controller->recent_count = 1U;
    controller->motion_anchor_x = x;
    controller->motion_stalled_since_ms = now_ms;
    controller->motion_boost = 0L;
    controller->previous_x_valid = true;
    controller->motion_anchor_valid = true;
}

static int32_t controller_update(Topic3Controller *controller,
                                 int16_t x,
                                 int16_t target_x,
                                 uint32_t now_ms)
{
    int16_t previous_x = controller->previous_x_valid
                       ? controller->previous_x : x;
    int16_t pixel_delta = (int16_t)(x - previous_x);
    int16_t error_x10;
    int16_t previous_error_x10;
    int16_t error_rate_x10;
    int16_t momentum_delta;
    int16_t motion_delta;
    int32_t minimum = 0L;
    int32_t boost_max;
    int32_t boost_increment;
    uint32_t boost_step_ms;
    uint32_t stalled_ms;
    float error_cm;
    float ki;
    float pid_output;
    float offset;
    float velocity_error_rate;
    float velocity_gain;
    float velocity_feedback;
    bool crossed_target;
    bool moving_to_target;
    bool far_approaching_fast;
    bool near_approaching_with_momentum;
    bool in_scoring_zone;
    bool should_apply_minimum = false;
    uint8_t i;

    controller->previous_x = x;
    controller->previous_x_valid = true;
    controller->filtered_pixel_delta += VELOCITY_ALPHA *
        ((float)pixel_delta - controller->filtered_pixel_delta);

    if (controller->recent_count < MOTION_WINDOW_SIZE) {
        controller->recent_x[controller->recent_count++] = x;
    } else {
        for (i = 1U; i < MOTION_WINDOW_SIZE; i++) {
            controller->recent_x[i - 1U] = controller->recent_x[i];
        }
        controller->recent_x[MOTION_WINDOW_SIZE - 1U] = x;
    }
    momentum_delta = (int16_t)(x - controller->recent_x[0]);
    velocity_error_rate = controller->filtered_pixel_delta * 10.0f / 29.0f;

    error_x10 = (int16_t)(((int32_t)x - target_x) * 10L / 29L);
    previous_error_x10 = controller->previous_error_x10;
    error_rate_x10 = (int16_t)(error_x10 - previous_error_x10);
    controller->previous_error_x10 = error_x10;

    if ((error_x10 > -CONTROL_DEADBAND_X10) &&
        (error_x10 < CONTROL_DEADBAND_X10)) {
        controller->integral = 0.0f;
        controller->previous_pid_error = 0.0f;
        controller->motion_boost = 0L;
        controller->motion_anchor_valid = false;
        return 0L;
    }

    crossed_target = ((previous_error_x10 != 0) && (error_x10 != 0) &&
                      ((int32_t)previous_error_x10 * error_x10 < 0));
    moving_to_target = ((float)error_x10 *
                        controller->filtered_pixel_delta < 0.0f);
    error_cm = (float)error_x10 / 10.0f;

    if (crossed_target || moving_to_target) {
        controller->integral = 0.0f;
    } else if (error_rate_x10 == 0) {
        controller->integral += error_cm;
    }
    controller->integral = clamp_f32(controller->integral,
                                     -CONTROL_INTEGRAL_LIMIT,
                                     CONTROL_INTEGRAL_LIMIT);

    ki = (abs_i32(error_x10) < 48U) ? CONTROL_KI_NEAR : 0.0f;
    pid_output = CONTROL_KP * error_cm +
                 ki * controller->integral +
                 CONTROL_KD * (error_cm - controller->previous_pid_error);
    controller->previous_pid_error = error_cm;
    pid_output = clamp_f32(pid_output, -CONTROL_PID_LIMIT, CONTROL_PID_LIMIT);
    offset = pid_output + error_cm * CONTROL_GRAVITY_FF;

    if (error_x10 >= 0) {
        velocity_gain = (abs_i32(error_x10) < CONTROL_NEAR_POSITIVE_X10)
                      ? CONTROL_DAMP_NEAR_POSITIVE : CONTROL_DAMP_FAR;
    } else {
        velocity_gain = (abs_i32(error_x10) < CONTROL_NEAR_NEGATIVE_X10)
                      ? CONTROL_DAMP_NEAR_NEGATIVE : CONTROL_DAMP_FAR;
    }
    velocity_feedback = clamp_f32(velocity_gain * velocity_error_rate,
                                  -CONTROL_DAMP_LIMIT,
                                  CONTROL_DAMP_LIMIT);
    offset += velocity_feedback;

    if (!controller->motion_anchor_valid) {
        controller->motion_anchor_x = x;
        controller->motion_stalled_since_ms = now_ms;
        controller->motion_anchor_valid = true;
    } else if (abs_i32((int32_t)x - controller->motion_anchor_x) >=
               MOTION_RESET_PX) {
        motion_delta = (int16_t)(x - controller->motion_anchor_x);
        controller->motion_anchor_x = x;
        controller->motion_stalled_since_ms = now_ms;
        if ((int32_t)error_x10 * motion_delta < 0) {
            int32_t drop = (abs_i32(error_x10) <= 10U)
                         ? NEAR_BOOST_INCREMENT : FAR_BOOST_INCREMENT;
            controller->motion_boost -= drop;
            if (controller->motion_boost < 0L) controller->motion_boost = 0L;
        }
    }

    far_approaching_fast = moving_to_target &&
        (abs_f32(controller->filtered_pixel_delta) >= VELOCITY_RELEASE_PX);
    near_approaching_with_momentum = far_approaching_fast ||
        (((int32_t)error_x10 * momentum_delta < 0) &&
         (abs_i32(momentum_delta) >= MOMENTUM_RELEASE_PX));

    if (abs_i32(error_x10) > 2U) {
        in_scoring_zone = (abs_i32(error_x10) <= 10U);
        if (in_scoring_zone) {
            should_apply_minimum = !near_approaching_with_momentum;
            minimum = (error_x10 > 0) ? NEAR_MIN_POSITIVE
                                      : NEAR_MIN_NEGATIVE;
            boost_step_ms = NEAR_BOOST_STEP_MS;
            boost_increment = NEAR_BOOST_INCREMENT;
            boost_max = NEAR_BOOST_MAX;
        } else {
            should_apply_minimum = !far_approaching_fast;
            minimum = (error_x10 > 0) ? FAR_MIN_POSITIVE
                                      : FAR_MIN_NEGATIVE;
            boost_step_ms = FAR_BOOST_STEP_MS;
            boost_increment = FAR_BOOST_INCREMENT;
            boost_max = FAR_BOOST_MAX;
        }

        stalled_ms = now_ms - controller->motion_stalled_since_ms;
        {
            int32_t timed_boost = (int32_t)(stalled_ms / boost_step_ms) *
                                  boost_increment;
            if (timed_boost > boost_max) timed_boost = boost_max;
            if (timed_boost > controller->motion_boost) {
                controller->motion_boost = timed_boost;
            }
        }
        minimum += controller->motion_boost;
    }

    if (should_apply_minimum) {
        if ((error_x10 > 0) && (offset < (float)minimum)) {
            offset = (float)minimum;
        } else if ((error_x10 < 0) && (offset > -(float)minimum)) {
            offset = -(float)minimum;
        }
    }

    return clamp_i32((int32_t)offset,
                     -CONTROL_MAX_OFFSET,
                     CONTROL_MAX_OFFSET);
}

static void motor_send_absolute(int32_t signed_position, uint32_t now_ms,
                                bool force)
{
    Step42Dir direction;
    uint32_t pulses;

    if ((!force) && g_last_motor_position_valid &&
        (signed_position == g_last_motor_position) &&
        !time_reached(now_ms, g_last_motor_send_ms + MOTOR_KEEPALIVE_MS)) {
        return;
    }

    if (signed_position >= 0) {
        direction = STEP42_DIR_CW;
        pulses = (uint32_t)signed_position;
    } else {
        direction = STEP42_DIR_CCW;
        pulses = (uint32_t)(-signed_position);
    }

    (void)Step42_MovePosition(direction, POSITION_SPEED, POSITION_ACC,
                              pulses, STEP42_POS_ABSOLUTE);
    g_last_motor_position = signed_position;
    g_last_motor_send_ms = now_ms;
    g_last_motor_position_valid = true;
}

static void print_stage(const char *message, uint32_t elapsed_ms)
{
    drv_uart_send_string(message);
    drv_uart_send_string(" time_ms=");
    drv_uart_print_num(elapsed_ms);
    drv_uart_send_string("\r\n");
}

static void oled_show_status(Topic3Stage stage, int16_t x,
                             uint32_t elapsed_ms)
{
    OLED_ShowString(1, 1, "T3 FINAL       ");
    if (stage == TOPIC3_WAIT_O) {
        OLED_ShowString(2, 1, "WAIT O         ");
    } else if (stage == TOPIC3_TO_POSITIVE) {
        OLED_ShowString(2, 1, "GO +5cm        ");
    } else if (stage == TOPIC3_TO_NEGATIVE) {
        OLED_ShowString(2, 1, "GO -5cm        ");
    } else {
        OLED_ShowString(2, 1, "DONE           ");
    }
    OLED_ShowString(3, 1, "X:");
    OLED_ShowSignedNum(3, 3, x, 4);
    OLED_ShowString(3, 9, "A:734");
    OLED_ShowString(4, 1, "TIME:");
    OLED_ShowNum(4, 6, elapsed_ms, 5);
}

void topic3(void)
{
    Topic3Stage stage = TOPIC3_WAIT_O;
    Topic3Controller controller;
    Topic3SettleWindow settle_window;
    Topic3FinalHold final_hold = {0};
    uint32_t last_frame_count = 0U;
    uint32_t last_frame_ms;
    uint32_t sequence_start_ms = 0U;
    uint32_t leg_start_ms = 0U;
    uint32_t last_oled_ms = 0U;
    bool time_result_printed = false;

    OLED_Init();
    OLED_Clear();
    drv_uart0_init();
    drv_vision_uart3_init();
    Step42_Init();

    controller_prime(&controller, BALL_O_X, BALL_O_X, 0U);
    settle_reset(&settle_window);
    g_last_motor_position_valid = false;

    delay_ms(10U);
    (void)Step42_Enable(true);
    delay_ms(300U);
    last_frame_ms = get_system_ms();
    motor_send_absolute(LEVEL_MOTOR_POSITION, last_frame_ms, true);

    drv_uart_send_string(
        "TOPIC3 FINAL: O560 -> +5(415) -> -5 score(705), aim(734)\r\n");
    drv_uart_send_string(
        "LEVEL=0, final PID starts at x=690, output <=52\r\n");

    while (1) {
        uint32_t now_ms = get_system_ms();
        uint32_t frame_count = drv_vision_get_frame_count();

        if (frame_count != last_frame_count) {
            int16_t x = (int16_t)drv_vision_get_x();
            int32_t control_offset = 0L;

            last_frame_count = frame_count;
            last_frame_ms = now_ms;

            if (stage == TOPIC3_WAIT_O) {
                motor_send_absolute(LEVEL_MOTOR_POSITION, now_ms, false);
                if (settle_update(&settle_window, now_ms, x, BALL_O_X,
                                  BALL_SCORE_TOLERANCE_PX,
                                  START_HOLD_MS, 8U, 3U)) {
                    stage = TOPIC3_TO_POSITIVE;
                    sequence_start_ms = now_ms;
                    leg_start_ms = now_ms;
                    controller_prime(&controller, x,
                                     BALL_POSITIVE_TARGET_X, now_ms);
                    settle_reset(&settle_window);
                    print_stage("START +5cm", 0U);
                }
            } else if (stage == TOPIC3_TO_POSITIVE) {
                control_offset = controller_update(&controller, x,
                                                   BALL_POSITIVE_TARGET_X,
                                                   now_ms);
                motor_send_absolute(LEVEL_MOTOR_POSITION - control_offset,
                                    now_ms, false);

                /* +5cm 中途点只要求进入 +4~+6cm，立即反向。 */
                if (x <= BALL_POSITIVE_TARGET_X +
                         (int16_t)BALL_SCORE_TOLERANCE_PX) {
                    print_stage("REACHED +5cm", now_ms - leg_start_ms);
                    stage = TOPIC3_TO_NEGATIVE;
                    leg_start_ms = now_ms;
                    controller_prime(&controller, x,
                                     BALL_NEGATIVE_CONTROL_X, now_ms);
                    final_hold.active = false;
                    settle_reset(&settle_window);
                }
            } else if (stage == TOPIC3_TO_NEGATIVE) {
                bool terminal_position_stable;

                control_offset = controller_update(&controller, x,
                                                   BALL_NEGATIVE_CONTROL_X,
                                                   now_ms);

                /* 接近-5cm后改用小输出位置-速度PD，避免轨迹阶段的
                 * 60~120脉冲最小驱动在终点附近反复切换。 */
                if (final_hold.active || (x >= FINAL_CONTROL_ENTRY_X)) {
                    int32_t hold_offset;
                    if (!final_hold.active) {
                        final_hold_reset(&final_hold, x, -control_offset,
                                         controller.filtered_pixel_delta);
                    }
                    hold_offset = final_hold_update(&final_hold, x);
                    control_offset = -hold_offset;
                    controller.motion_boost = 0L;
                    controller.motion_anchor_x = x;
                    controller.motion_anchor_valid = true;
                    controller.motion_stalled_since_ms = now_ms;
                }

                motor_send_absolute(LEVEL_MOTOR_POSITION - control_offset,
                                    now_ms, false);

                terminal_position_stable = settle_update(
                    &settle_window, now_ms, x, BALL_FINAL_HOLD_X,
                    BALL_FINAL_TOLERANCE_PX, TERMINAL_HOLD_MS, 4U, 2U);
                if (terminal_position_stable &&
                    (abs_i32((int32_t)x - BALL_NEGATIVE_SCORE_X) <=
                     BALL_SCORE_TOLERANCE_PX) &&
                    final_hold.active &&
                    (abs_f32(final_hold.filtered_pixel_delta) <=
                     TERMINAL_SPEED_THRESHOLD_PX)) {
                    uint32_t total_ms = now_ms - sequence_start_ms;
                    print_stage("REACHED -5cm", now_ms - leg_start_ms);
                    if (total_ms <= TOPIC_TIME_LIMIT_MS) {
                        print_stage("TOPIC3 PASS", total_ms);
                    } else {
                        print_stage("TOPIC3 FAIL", total_ms);
                    }
                    stage = TOPIC3_DONE;
                    time_result_printed = true;
                    settle_reset(&settle_window);
                }
            } else {
                int32_t hold_offset;
                if (!final_hold.active) {
                    final_hold_reset(&final_hold, x, 0.0f, 0.0f);
                }
                hold_offset = final_hold_update(&final_hold, x);
                motor_send_absolute(LEVEL_MOTOR_POSITION + hold_offset,
                                    now_ms, false);
            }

            if (time_reached(now_ms, last_oled_ms + 100U)) {
                uint32_t elapsed_ms = (sequence_start_ms == 0U)
                                    ? 0U : now_ms - sequence_start_ms;
                oled_show_status(stage, x, elapsed_ms);
                last_oled_ms = now_ms;
            }
        } else if (time_reached(now_ms,
                                last_frame_ms + CAMERA_TIMEOUT_MS)) {
            /* 视觉断流后禁止使用旧坐标，立即回水平。 */
            motor_send_absolute(LEVEL_MOTOR_POSITION, now_ms, true);
            controller_prime(&controller, BALL_O_X, BALL_O_X, now_ms);
            final_hold.active = false;
            settle_reset(&settle_window);
            OLED_ShowString(2, 1, "CAM LOST        ");
            if (!time_result_printed) {
                drv_uart_send_string("WARN: camera timeout, level restored\r\n");
            }
            last_frame_ms = now_ms;
        }

        delay_ms(1U);
    }
}
