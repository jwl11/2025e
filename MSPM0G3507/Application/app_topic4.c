#include "app.h"
#include "bsp_42step.h"
#include "bsp_fishpath.h"
#include "bsp_OLED.h"
#include "drv_uart.h"
#include "mid_delay.h"
#include "mid_pid.h"

/* ================================================================
 * 小球平衡 — 位置 PID + 前馈 + 速度阻尼 (ABSOLUTE 模式)
 *
 *   pos = PID(误差) + FF(误差) + damp(速度)
 *   死区内 pos=0 回中, 积分冻结
 *   Ki=0 绕过积分问题, 前馈已够消静差
 * ================================================================ */

#define BALL_KP        25.0f   /* P */
#define BALL_KI         0.0f    /* I (动态) */
#define BALL_KD         2.0f    /* D */
#define BALL_KI_NEAR    0.1f    /* 近处慢慢攒 */
#define KI_NEAR_THRESH  48
#define BALL_KI_LIMIT 150.0f
#define GRAVITY_FF      1.0f    /* 前馈 */
#define DAMP_GAIN_FAR   6.0f    /* 远: 猛刹 */
#define DAMP_GAIN_NEAR  2.0f    /* 近: 轻刹 */
#define DAMP_LIMIT       110     /* 反向最大速度收一收 */
#define DAMP_FAR_THRESH  48     /* 4cm 分界线 */

#define DEADBAND          2     /* ±0.5cm */
#define PERIOD_MS         5
#define MAX_POS         500     /* 绝对位置限幅 */

#define POS_SPEED      1500
#define POS_ACC         255
#define TRACK_SPEED      30

static PID_Controller g_pid;
static int16_t prev_err;

void topic4(void)
{
    OLED_Init();
    OLED_Clear();
    OLED_ShowString(1, 1, "topic4: PID+Damp ABS");

    drv_uart0_init();
    drv_vision_uart3_init();
    while (drv_vision_get_x() == 0) { delay_ms(10); }
    drv_uart_send_string("CAM OK\r\n");

    pid_init(&g_pid, BALL_KP, BALL_KI, BALL_KD,
             BALL_KI_LIMIT, 1000.0f);

    Step42_Init();
    fishpath_init(XUNJI_DEFAULT_KP, XUNJI_DEFAULT_KI,
                  XUNJI_DEFAULT_KD, XUNJI_DEFAULT_SPEED);
    fishpath_set_speed(TRACK_SPEED);
    fishpath_stop();
    Step42_Enable(true);
    delay_ms(500);
    fishpath_start();

    while (1) {
        int16_t err      = app_ball_error_cm_x10();
        int16_t err_rate = err - prev_err;
        prev_err = err;

        int32_t pos;

        if (err > -DEADBAND && err < DEADBAND) {
            /* 死区: pos=0 回中, 不调 pid_update (积分自然冻结) */
            pos = 0;
        } else {
            /* 正常: PID + 前馈 + 速度阻尼 (近处开积分) */
            float err_cm  = (float)err / 10.0f;
            g_pid.Ki = (err > -KI_NEAR_THRESH && err < KI_NEAR_THRESH)
                     ? BALL_KI_NEAR : 0.0f;
            float pid_out = pid_update(&g_pid, err_cm);
            float offset  = pid_out + err_cm * GRAVITY_FF;

            /* 阻尼: 仅在跑偏时反向刹 */
            float damp = 0.0f;
            if (err * err_rate > 0) {
                float gain = (err > -DAMP_FAR_THRESH && err < DAMP_FAR_THRESH)
                           ? DAMP_GAIN_NEAR : DAMP_GAIN_FAR;
                damp = -gain * (float)err_rate;
                if (damp >  DAMP_LIMIT) damp =  DAMP_LIMIT;
                if (damp < -DAMP_LIMIT) damp = -DAMP_LIMIT;
            }
            offset += damp;
            pos = (int32_t)offset;
        }

        if (pos >  MAX_POS) pos =  MAX_POS;
        if (pos < -MAX_POS) pos = -MAX_POS;

        static uint8_t dbg;
        if (++dbg >= 40) { dbg = 0;
            drv_uart_send_string("e=");  drv_uart_print_signed(err);
            drv_uart_send_string(" v="); drv_uart_print_signed(err_rate);
            drv_uart_send_string(" p="); drv_uart_print_signed(pos);
            drv_uart_send_string("\r\n");
        }

        Step42Dir dir = (pos >= 0) ? STEP42_DIR_CW : STEP42_DIR_CCW;
        uint32_t pulses = (uint32_t)(pos >= 0 ? pos : -pos);
        Step42_MovePosition(dir, POS_SPEED, POS_ACC,
                            pulses, STEP42_POS_ABSOLUTE);

        delay_ms(PERIOD_MS);
    }
}
