#include "app.h"
#include "bsp_zdt_x35.h"
#include "drv_uart.h"
#include "drv_zdt_x35_uart.h"
#include "mid_delay.h"

#define BALL_SPEED      2999     /* 电机转速 RPM */
#define BALL_GAIN       20      /* 每 0.1cm 误差走多少脉冲 */
#define BALL_MAX_PULSES 800     /* 单次最大脉冲，防走过头 */
#define BALL_DEADBAND   10      /* ±1.0cm 死区 (单位 0.1cm) */
#define BALL_PERIOD_MS  50      /* 控制周期 */

#define TRACK_SPEED     30      /* 循迹占空比 0~100 */
int16_t err;
void ctrl_step(void);
void ctrl_xunji(void);

void topic4(void)
{
    OLED_Clear();
    OLED_ShowString(1, 1, "now topic 4!");
    OLED_ShowString(2, 1, "Time:");
    /**初始化**/
    drv_uart0_init();
    drv_vision_uart3_init();
    zdt_x35_init();
    fishpath_init(XUNJI_DEFAULT_KP, XUNJI_DEFAULT_KI,
                  XUNJI_DEFAULT_KD, XUNJI_DEFAULT_SPEED);
    fishpath_set_speed(TRACK_SPEED);
    fishpath_stop();  /* 初始化后默认停车，等按键启动 */

    /* 使能 */
    zdt_x35_set_enable(true, false);
    delay_ms(500);

    fishpath_start();

    while (1) {

/********************控制循迹**********************/
       //sctrl_xunji();
/********************控制水管**********************/
        err = app_ball_error_cm_x10();   /* 误差，单位 0.1cm */
        
        if(vision_x==0)
        {
            continue;
        }
        ctrl_step();

        delay_ms(BALL_PERIOD_MS);

    }
}



void ctrl_step(void)
{
        if (err > BALL_DEADBAND) {
            /* 球偏 pivot 端 → 电机下降，让球滚回来 */
            uint32_t pulses = (uint32_t)err * BALL_GAIN;
            if (pulses > BALL_MAX_PULSES) pulses = BALL_MAX_PULSES;
            app_zdt_x35_move_down(BALL_SPEED, pulses);
        }
        else if (err < -BALL_DEADBAND) {
            /* 球偏电机端 → 电机上升，让球滚向 pivot */
            uint32_t pulses = (uint32_t)(-err) * BALL_GAIN;
            if (pulses > BALL_MAX_PULSES) pulses = BALL_MAX_PULSES;
            app_zdt_x35_move_up(BALL_SPEED, pulses);
        }
        /* else: ±1cm 内不动 */

}
void ctrl_xunji(void)
{
    fishpath_update();
}