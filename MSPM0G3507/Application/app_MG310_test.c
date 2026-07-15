#include "ti_msp_dl_config.h"
#include "drv_tim.h"
#include "bsp_encoder.h"
#include "drv_uart.h"
#include "mid_delay.h"
#include "bsp_led.h"

/**
 * @brief  MG310 双路霍尔编码电机功能测试
 *
 * 测试流程:
 *   1. 电机A 递增正转 (20% → 40% → 60% → 80%)
 *   2. 电机A 停止
 *   3. 电机A 递增反转
 *   4. 电机A 刹车制动
 *   5. 电机B 正反转测试
 *   6. 双电机同步正反转
 *   7. 双电机同步停止
 */
void app_MG310_test(void)
{
    drv_uart_send_string("================================\r\n");
    drv_uart_send_string("  MG310 Dual Motor Test Start\r\n");
    drv_uart_send_string("  TIM1, CH0(PA15)/CH1(PA16) PWM\r\n");
    drv_uart_send_string("================================\r\n\r\n");

    /* 初始化双路电机 */
    mg310_motorInitAll();
    drv_uart_send_string("[INIT] Both motors initialized (stopped).\r\n\r\n");
    delay_ms(500);

    /* ================================================
     * 阶段1: 电机A 正转速度递增测试
     * ================================================ */
    drv_uart_send_string("--- Phase 1: Motor A Forward Ramp ---\r\n");
    drv_uart_send_string("Motor A: Forward 20%\r\n");
    mg310_motorForward(MG310_MOTOR_A, 20);
    use_led_ON();
    delay_ms(2000);

    drv_uart_send_string("Motor A: Forward 40%\r\n");
    mg310_motorSetSpeed(MG310_MOTOR_A, 40);
    delay_ms(2000);

    drv_uart_send_string("Motor A: Forward 60%\r\n");
    mg310_motorSetSpeed(MG310_MOTOR_A, 60);
    delay_ms(2000);

    drv_uart_send_string("Motor A: Forward 80%\r\n");
    mg310_motorSetSpeed(MG310_MOTOR_A, 80);
    delay_ms(2000);

    /* ================================================
     * 阶段2: 电机A 停止 (惯性滑行)
     * ================================================ */
    drv_uart_send_string("\r\n--- Phase 2: Motor A Coast Stop ---\r\n");
    drv_uart_send_string("Motor A: Stop\r\n");
    mg310_motorStop(MG310_MOTOR_A);
    use_led_OFF();
    delay_ms(1500);

    /* ================================================
     * 阶段3: 电机A 反转速度递增测试
     * ================================================ */
    drv_uart_send_string("\r\n--- Phase 3: Motor A Reverse Ramp ---\r\n");
    drv_uart_send_string("Motor A: Reverse 20%\r\n");
    mg310_motorReverse(MG310_MOTOR_A, 20);
    use_led_ON();
    delay_ms(2000);

    drv_uart_send_string("Motor A: Reverse 40%\r\n");
    mg310_motorSetSpeed(MG310_MOTOR_A, 40);
    delay_ms(2000);

    drv_uart_send_string("Motor A: Reverse 60%\r\n");
    mg310_motorSetSpeed(MG310_MOTOR_A, 60);
    delay_ms(2000);

    /* ================================================
     * 阶段4: 电机A 刹车制动
     * ================================================ */
    drv_uart_send_string("\r\n--- Phase 4: Motor A Brake ---\r\n");
    drv_uart_send_string("Motor A: Brake (short brake)\r\n");
    mg310_motorBrake(MG310_MOTOR_A);
    use_led_OFF();
    delay_ms(1500);

    /* 刹车后切回停止状态 */
    mg310_motorStop(MG310_MOTOR_A);
    delay_ms(500);

    /* ================================================
     * 阶段5: 电机B 独立测试
     * ================================================ */
    drv_uart_send_string("\r\n--- Phase 5: Motor B Independent Test ---\r\n");
    drv_uart_send_string("Motor B: Forward 30%\r\n");
    mg310_motorForward(MG310_MOTOR_B, 30);
    use_led_ON();
    delay_ms(2000);

    drv_uart_send_string("Motor B: Stop\r\n");
    mg310_motorStop(MG310_MOTOR_B);
    use_led_OFF();
    delay_ms(1000);

    drv_uart_send_string("Motor B: Reverse 30%\r\n");
    mg310_motorReverse(MG310_MOTOR_B, 30);
    use_led_ON();
    delay_ms(2000);

    drv_uart_send_string("Motor B: Stop\r\n");
    mg310_motorStop(MG310_MOTOR_B);
    use_led_OFF();
    delay_ms(1000);

    /* ================================================
     * 阶段6: 双电机同步正反转 (差速测试)
     * ================================================ */
    drv_uart_send_string("\r\n--- Phase 6: Dual Motor Differential ---\r\n");
    drv_uart_send_string("Motor A: Forward 50%, Motor B: Forward 30%\r\n");
    mg310_motorForward(MG310_MOTOR_A, 50);
    mg310_motorForward(MG310_MOTOR_B, 30);
    use_led_ON();
    delay_ms(3000);

    drv_uart_send_string("Dual: Stop\r\n");
    mg310_motorStopAll();
    use_led_OFF();
    delay_ms(1000);

    drv_uart_send_string("Motor A: Reverse 50%, Motor B: Reverse 50%\r\n");
    mg310_motorReverse(MG310_MOTOR_A, 50);
    mg310_motorReverse(MG310_MOTOR_B, 50);
    use_led_ON();
    delay_ms(3000);

    /* ================================================
     * 阶段7: 双电机同步停止
     * ================================================ */
    drv_uart_send_string("\r\n--- Phase 7: Dual Motor Stop ---\r\n");
    mg310_motorStopAll();
    use_led_OFF();

    drv_uart_send_string("\r\n================================\r\n");
    drv_uart_send_string("  MG310 Motor Test Complete!\r\n");
    drv_uart_send_string("================================\r\n");

    while (1)
    {
        /* 测试结束, LED 闪烁指示 */
        use_led_TOGGLE();
        delay_ms(500);
    }
}
