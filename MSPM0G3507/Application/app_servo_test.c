#include "ti_msp_dl_config.h"
#include "bsp_servo.h"
#include "mid_delay.h"

/* ================================================================
 * 舵机测试函数
 *
 * 从 0° 缓慢扫到 180° 再扫回, 验证舵机全范围工作。
 * 每次步进 5°, 间隔 50ms, 可用示波器或肉眼观察。
 * ================================================================ */

void app_servo_test(void)
{
    uint8_t angle;

    bsp_servo_init();
    delay_ms(500);   /* 等待舵机归中 */

    while (1) {
        /* 0° → 180° */
        for (angle = 0; angle <= 180; angle += 5) {
            bsp_servo_setAngle(angle);
            delay_ms(50);
        }

        /* 180° → 0° */
        for (angle = 180; angle > 0; angle -= 5) {
            bsp_servo_setAngle(angle);
            delay_ms(50);
        }

        /* 回到 0° 后短暂停顿 */
        bsp_servo_setAngle(0);
        delay_ms(500);
    }
}
