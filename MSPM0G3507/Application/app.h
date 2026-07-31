#ifndef __APP_H
#define __APP_H


#include "ti_msp_dl_config.h"


/* ================================================================
 *  循迹参数开始
 * ================================================================ */
#define XUNJI_DEFAULT_KP         250.0f
#define XUNJI_DEFAULT_KI           0.0f
#define XUNJI_DEFAULT_KD          12.0f
#define XUNJI_DEFAULT_SPEED       30U

#define XUNJI_CONTROL_PERIOD_MS    5U
#define DEBUG_PRINT_MS           200U
#define OLED_UPDATE_MS           200U

/*
 * 启停线: 宽黑线连续覆盖 ≥3 路相邻传感器。
 * 当前调试输出中 '.'=黑线、'#'=白底，因此检测连续内部值 0 的数量。
 */
#define START_LINE_SENSOR_MIN       3U
#define START_LINE_CONFIRM_CYCLES   1U   /* 高速过线单帧确认 */
#define START_LINE_RELEASE_CYCLES   3U   /* 驶离确认，防止同一条线重复触发 */

#define BUTTON_START_HOLD_MS     1000U   /* 长按 1s 确认启动 */
#define TARGET_LAPS_MAX             5U

/* ================================================================
 *  循迹参数结束
 * ================================================================ */






void app_delay_test(void);
void app_debug_test(void);
void app_pwm_test(void);
void app_as5600_test(void);
void app_BLCD_test(void);
void app_MG310_test(void);
void app_fishpath_test(void);
void app_fishpath_motor_test(void);
void app_button_test(void);
void app_vision_line_test(void);
void app_motor_ctrl_test(void);
void app_motor_position_test(void);
void app_zdt_x35_probe(void);
void app_zdt_x35_motion_test(void);
void app_zdt_x35_position_test(void);
bool app_zdt_x35_move_down(uint16_t speed_rpm, uint32_t pulse_count);
bool app_zdt_x35_move_up(uint16_t speed_rpm, uint32_t pulse_count);
void MPU6050_test(void);
void MPU6050_straight_test(void);
void app_servo_test(void);
void app_oled_timer(void);
void app_42step_test(void);
void app_42step_position_test(void);
void app_42step_simple(void);
int16_t app_ball_position_cm_x10(void);
int16_t app_ball_error_cm_x10(void);
void topic(void);

/***
 * 6道题目
 *
 * ****/
void topic1(void);
void topic2(void);
void topic3(void);
void topic4(void);
void topic5(void);
void topic6(void);






#endif // __APP_H
