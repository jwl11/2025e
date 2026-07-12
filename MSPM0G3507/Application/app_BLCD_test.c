#include "app.h"
#include "bsp_BLCD.h"
#include "bsp_led.h"
#include "mid_delay.h"

/* ================================================================
 *  app_BLCD_test — 速度环驱动无刷电机（无串口打印）
 *
 *  调用顺序: bsp_bldc_init → kick → bsp_bldc_set_speed → stop
 *  kick 解决静摩擦，100ms 3V 扭矩脉冲让电机先动起来再切闭环。
 * ================================================================ */
void app_BLCD_test(void)
{
    /* ---- 1. 初始化（PWM + 编码器零角度校准，约 3s）---- */
    bsp_bldc_init(12.0f, 7, -1);

    /* ---- 2. 启动 kick（100ms 恒定扭矩，克服静摩擦）---- */
    {
        extern void foc_set_torque(float Uq, float angle_el);
        extern float foc_electrical_angle(void);
        for (int i = 0; i < 100; i++) {
            foc_set_torque(3.0f, foc_electrical_angle());
            delay_ms(1);
        }
    }

    /* ---- 3. 速度闭环 5 rad/s ---- */
    while (1) {
        bsp_bldc_set_speed(10.0f);
        use_led_TOGGLE();
        delay_ms(1);
    }
}
