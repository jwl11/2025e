#include "app.h"

#include "bsp_OLED.h"
#include "bsp_button.h"
#include "drv_tim.h"
#include "drv_uart.h"
#include "mid_delay.h"

/* ================================================================
 * MaixCAM 坐标 → 水管刻度换算
 *
 * 标定数据（X 坐标）：
 *   left:  X=209 → -11.0 cm (电机端)
 *   mid:   X=554 →   0.0 cm (中点)
 *   right: X=874 → +12.0 cm (pivot端)
 *
 * scale = (874 - 209) / (12 - (-11)) ≈ 28.9 像素/cm
 * ================================================================ */

#define X_CENTER  554
#define X_SCALE   29   /* 像素/cm */

/*
 * 返回球在管上刻度，单位 0.1cm。
 * 正 = 偏 pivot 端，负 = 偏电机端。
 */
int16_t app_ball_position_cm_x10(void)
{
    int32_t raw = (int32_t)drv_vision_get_x();
    return (int16_t)((raw - X_CENTER) * 10 / X_SCALE);
}

/*
 * 返回球与目标(0刻度)的误差，单位 0.1cm。
 * 正 = 球偏 pivot 端需要电机下降
 * 负 = 球偏电机端需要电机上升
 */
int16_t app_ball_error_cm_x10(void)
{
    return app_ball_position_cm_x10();  /* target 就是 0 */
}

