#include "app.h"
#include "bsp_42step.h"
#include "bsp_fishpath.h"
#include "bsp_OLED.h"
#include "drv_uart.h"
#include "mid_delay.h"
#include "mid_xunji.h"

#include <stdbool.h>
#include <stdint.h>

/*
 * 第四/五题共用程序：小车循迹时将钢球稳定在 O 点。
 *
 * 钢球控制完整移植自电脑端 pc_serial_topic4.py：
 *   外环：位置 PID -> 目标球速度；
 *   内环：球速度 PID -> 水管倾斜脉冲。
 *
 * 当前参数：位置 P=5, I=0.03, D=0.8；
 *                 速度 P=8, I=0, D=0.2。
 * 球进入 O 点 +/-0.15 cm 后清空位置积分，避免低速静止后还被慢慢推走。
 *
 * Topic 4：A -> B，B 点没有独立标志线，程序持续循迹并显示计时。
 * Topic 5：A 点出发，再次检测到 A 点启停线后停车并冻结计时。
 *
 * MaixCAM2：UART3，@x,y#；ZDT：UART2，绝对位置模式。
 */

#define BALL_CENTER_X                 560
#define BALL_PIXELS_PER_CM           29.0f
#define BALL_START_WINDOW_PX            60
#define LEVEL_MOTOR_POSITION          (0L)

#define POSITION_KP                   5.0f
#define POSITION_KI                   0.03f
#define POSITION_KD                   0.8f
#define POSITION_DEADBAND_CM          0.15f
#define POSITION_INTEGRAL_LIMIT       1.0f
#define TARGET_SPEED_LIMIT_CM_S      12.0f

#define SPEED_KP                      8.0f
#define SPEED_KI                      0.0f
#define SPEED_KD                      0.2f
#define SPEED_FILTER_ALPHA            0.25f
#define SPEED_INTEGRAL_LIMIT          3.0f
#define MAX_MEASURED_SPEED_CM_S      30.0f

#define CONTROL_OUTPUT_LIMIT        120.0f
#define CONTROL_OUTPUT_SLEW_S       600.0f
#define POSITION_SPEED             1000U
#define POSITION_ACC                180U
#define VISION_TIMEOUT_MS           300U
#define MOTOR_KEEPALIVE_MS          500U
#define OLED_REFRESH_MS             100U

typedef struct {
    int16_t previous_x;
    uint32_t previous_frame_ms;
    float filtered_speed;
    float position_integral;
    float speed_integral;
    float previous_speed_error;
    float previous_command;
    bool valid;
} BallCascadeController;

typedef struct {
    int16_t x;
    float error_cm;
    float speed_cm_s;
    float target_speed_cm_s;
    int32_t command;
} BallCascadeOutput;

static float clamp_f32(float value, float minimum, float maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

static int32_t round_f32_to_i32(float value)
{
    return (int32_t)(value + ((value >= 0.0f) ? 0.5f : -0.5f));
}

static float abs_f32(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static void cascade_reset(BallCascadeController *controller,
                          int16_t x, uint32_t now_ms)
{
    controller->previous_x = x;
    controller->previous_frame_ms = now_ms;
    controller->filtered_speed = 0.0f;
    controller->position_integral = 0.0f;
    controller->speed_integral = 0.0f;
    controller->previous_speed_error = 0.0f;
    controller->previous_command = 0.0f;
    controller->valid = true;
}

static BallCascadeOutput cascade_update(BallCascadeController *controller,
                                        int16_t x, uint32_t now_ms)
{
    BallCascadeOutput output;
    float dt;
    float measured_speed;
    float position_p;
    float position_i;
    float position_d;
    float speed_error;
    float speed_derivative;
    float requested;
    float max_change;
    uint32_t dt_ms;

    if (!controller->valid) {
        cascade_reset(controller, x, now_ms);
    }

    dt_ms = now_ms - controller->previous_frame_ms;
    if (dt_ms < 10U) dt_ms = 10U;
    if (dt_ms > 100U) dt_ms = 100U;
    dt = (float)dt_ms / 1000.0f;

    measured_speed = ((float)(x - controller->previous_x) /
                      BALL_PIXELS_PER_CM) / dt;
    measured_speed = clamp_f32(measured_speed,
                               -MAX_MEASURED_SPEED_CM_S,
                               MAX_MEASURED_SPEED_CM_S);
    controller->filtered_speed += SPEED_FILTER_ALPHA *
        (measured_speed - controller->filtered_speed);

    output.x = x;
    output.error_cm = (float)(x - BALL_CENTER_X) / BALL_PIXELS_PER_CM;

    if (abs_f32(output.error_cm) <= POSITION_DEADBAND_CM) {
        /* 进入实测静止区：清积分，位置 P/I 不再推球。 */
        controller->position_integral = 0.0f;
        position_p = 0.0f;
        position_i = 0.0f;
    } else {
        controller->position_integral += output.error_cm * dt;
        controller->position_integral = clamp_f32(
            controller->position_integral,
            -POSITION_INTEGRAL_LIMIT,
            POSITION_INTEGRAL_LIMIT);
        position_p = POSITION_KP * output.error_cm;
        position_i = POSITION_KI * controller->position_integral;
    }

    position_d = POSITION_KD * controller->filtered_speed;
    output.target_speed_cm_s = -(position_p + position_i + position_d);
    output.target_speed_cm_s = clamp_f32(output.target_speed_cm_s,
                                         -TARGET_SPEED_LIMIT_CM_S,
                                         TARGET_SPEED_LIMIT_CM_S);

    speed_error = controller->filtered_speed - output.target_speed_cm_s;
    controller->speed_integral += speed_error * dt;
    controller->speed_integral = clamp_f32(controller->speed_integral,
                                           -SPEED_INTEGRAL_LIMIT,
                                           SPEED_INTEGRAL_LIMIT);
    speed_derivative =
        (speed_error - controller->previous_speed_error) / dt;

    requested = SPEED_KP * speed_error
              + SPEED_KI * controller->speed_integral
              + SPEED_KD * speed_derivative;
    requested = clamp_f32(requested,
                          -CONTROL_OUTPUT_LIMIT,
                          CONTROL_OUTPUT_LIMIT);

    max_change = CONTROL_OUTPUT_SLEW_S * dt;
    requested = clamp_f32(requested,
                          controller->previous_command - max_change,
                          controller->previous_command + max_change);

    controller->previous_x = x;
    controller->previous_frame_ms = now_ms;
    controller->previous_speed_error = speed_error;
    controller->previous_command = requested;

    output.speed_cm_s = controller->filtered_speed;
    output.command = round_f32_to_i32(requested);
    return output;
}

static void move_stepper_absolute(int32_t position)
{
    Step42Dir direction =
        (position >= 0) ? STEP42_DIR_CW : STEP42_DIR_CCW;
    uint32_t pulses = (uint32_t)((position >= 0) ? position : -position);

    (void)Step42_MovePosition(direction, POSITION_SPEED, POSITION_ACC,
                              pulses, STEP42_POS_ABSOLUTE);
}

/* 启停线为连续至少3路黑线。 */
static bool is_start_line(void)
{
    uint8_t sensors[XUNJI_SENSOR_COUNT];
    uint8_t consecutive_black = 0U;
    uint8_t i;

    xunji_get_sensors(sensors);
    for (i = 0U; i < XUNJI_SENSOR_COUNT; i++) {
        if (sensors[i] == 0U) {
            consecutive_black++;
            if (consecutive_black >= START_LINE_SENSOR_MIN) return true;
        } else {
            consecutive_black = 0U;
        }
    }
    return false;
}

static bool take_start_line_event(bool *line_active,
                                  uint8_t *confirm_count,
                                  uint8_t *release_count)
{
    bool on_line = is_start_line();

    if (*line_active) {
        if (!on_line) {
            if (*release_count < START_LINE_RELEASE_CYCLES) {
                (*release_count)++;
            }
            if (*release_count >= START_LINE_RELEASE_CYCLES) {
                *line_active = false;
                *release_count = 0U;
            }
        } else {
            *release_count = 0U;
        }
        return false;
    }

    if (on_line) {
        if (*confirm_count < START_LINE_CONFIRM_CYCLES) (*confirm_count)++;
        if (*confirm_count >= START_LINE_CONFIRM_CYCLES) {
            *line_active = true;
            *confirm_count = 0U;
            return true;
        }
    } else {
        *confirm_count = 0U;
    }
    return false;
}

static void oled_show_elapsed(uint32_t elapsed_ms)
{
    uint32_t seconds = elapsed_ms / 1000U;
    uint32_t tenths = (elapsed_ms / 100U) % 10U;

    OLED_ShowString(2, 1, "Time:   . s     ");
    OLED_ShowNum(2, 7, seconds, 2);
    OLED_ShowNum(2, 10, tenths, 1);
}

static void topic45_run(bool full_lap)
{
    BallCascadeController controller = {0};
    BallCascadeOutput output = {0};
    uint32_t last_frame_count;
    uint32_t last_vision_ms;
    uint32_t last_motor_ms;
    uint32_t last_control_ms;
    uint32_t last_oled_ms;
    uint32_t start_ms;
    int32_t last_motor_position = LEVEL_MOTOR_POSITION;
    bool line_active = true;
    uint8_t line_confirm = 0U;
    uint8_t line_release = 0U;
    bool finished = false;
    bool camera_paused = false;

    OLED_Init();
    OLED_Clear();
    OLED_ShowString(1, 1, full_lap ? "Topic5 A->A" : "Topic4 A->B");
    OLED_ShowString(3, 1, "Wait camera...");

    drv_uart0_init();
    drv_vision_uart3_init();
    Step42_Init();
    fishpath_init(XUNJI_DEFAULT_KP, XUNJI_DEFAULT_KI,
                  XUNJI_DEFAULT_KD, XUNJI_DEFAULT_SPEED);
    fishpath_stop();

    /* 必须等到一个有效新坐标，不用0坐标启动。 */
    last_frame_count = drv_vision_get_frame_count();
    while ((drv_vision_get_frame_count() == last_frame_count) ||
           (drv_vision_get_x() == 0U) ||
           ((int16_t)drv_vision_get_x() <
            BALL_CENTER_X - BALL_START_WINDOW_PX) ||
           ((int16_t)drv_vision_get_x() >
            BALL_CENTER_X + BALL_START_WINDOW_PX)) {
        delay_ms(5U);
    }

    last_frame_count = drv_vision_get_frame_count();
    start_ms = get_system_ms();
    last_vision_ms = start_ms;
    last_motor_ms = start_ms;
    last_control_ms = start_ms;
    last_oled_ms = start_ms;
    cascade_reset(&controller, (int16_t)drv_vision_get_x(), start_ms);

    (void)Step42_Enable(true);
    move_stepper_absolute(LEVEL_MOTOR_POSITION);
    delay_ms(300U);
    xunji_reset_tracking();
    mid_stopwatch_start();
    fishpath_start();

    OLED_ShowString(3, 1, "Running         ");
    drv_uart_send_string(full_lap ?
        "TOPIC5 START: cascade balance + one lap\r\n" :
        "TOPIC4 START: cascade balance + A to B\r\n");

    while (1) {
        uint32_t now_ms = get_system_ms();
        uint32_t frame_count = drv_vision_get_frame_count();

        if (!finished && !camera_paused &&
            (now_ms - last_control_ms >= XUNJI_CONTROL_PERIOD_MS)) {
            last_control_ms = now_ms;
            fishpath_update();

            if (full_lap &&
                take_start_line_event(&line_active,
                                      &line_confirm,
                                      &line_release)) {
                finished = true;
                fishpath_stop();
                (void)mid_stopwatch_stop();
                move_stepper_absolute(LEVEL_MOTOR_POSITION);
                OLED_ShowString(3, 1, "Finished        ");
                drv_uart_send_string("TOPIC5 FINISHED\r\n");
            }
        }

        if (!finished && frame_count != last_frame_count) {
            int32_t motor_position;
            last_frame_count = frame_count;
            last_vision_ms = now_ms;
            if (camera_paused) {
                cascade_reset(&controller,
                              (int16_t)drv_vision_get_x(), now_ms);
                camera_paused = false;
                xunji_reset_tracking();
                fishpath_start();
                OLED_ShowString(3, 1, "Running         ");
                drv_uart_send_string(
                    "CAM RECOVERED: cascade reset, car resumed\r\n");
            } else {
                output = cascade_update(&controller,
                                        (int16_t)drv_vision_get_x(),
                                        now_ms);

                /* 与电脑端 --invert-direction 的实测方向一致。 */
                motor_position = LEVEL_MOTOR_POSITION - output.command;
                if ((motor_position != last_motor_position) ||
                    (now_ms - last_motor_ms >= MOTOR_KEEPALIVE_MS)) {
                    move_stepper_absolute(motor_position);
                    last_motor_position = motor_position;
                    last_motor_ms = now_ms;
                }
            }
        }

        if (!finished && !camera_paused &&
            (now_ms - last_vision_ms > VISION_TIMEOUT_MS)) {
            /* 与电脑端一致：断流时回水平并清空双环；
             * C端同时暂停小车，新坐标恢复后再继续。 */
            camera_paused = true;
            fishpath_stop();
            move_stepper_absolute(LEVEL_MOTOR_POSITION);
            last_motor_position = LEVEL_MOTOR_POSITION;
            last_motor_ms = now_ms;
            OLED_ShowString(3, 1, "CAM LOST        ");
            drv_uart_send_string("CAM LOST: level restored, car paused\r\n");
        }

        if (now_ms - last_oled_ms >= OLED_REFRESH_MS) {
            uint32_t elapsed_ms = mid_stopwatch_get_elapsed_ms();
            last_oled_ms = now_ms;
            oled_show_elapsed(elapsed_ms);
            OLED_ShowString(4, 1, "X:");
            OLED_ShowNum(4, 3, (uint32_t)output.x, 4);
            OLED_ShowString(4, 8, " O:560  ");
        }

        delay_ms(1U);
    }
}

void topic4(void)
{
    topic45_run(false);
}

void topic5(void)
{
    topic45_run(true);
}
