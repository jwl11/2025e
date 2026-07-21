#include "app.h"
#include "bsp_f32c_gimbal.h"
#include "drv_uart.h"
#include "mid_delay.h"

/* 可按云台机械安装方向修改以下安全测试角度。 */
#define F32C_TEST_SPEED_RPM       30U
#define F32C_TEST_HOME_DEG         0.0f
#define F32C_TEST_X_DEG           30.0f
#define F32C_TEST_Y_DEG           30.0f
#define F32C_TEST_SYNC_Y_DEG      50.0f
#define F32C_POWER_ON_WAIT_MS    2000U
#define F32C_COMMAND_GAP_MS        50U
#define F32C_SYNC_FRAME_GAP_MS      2U
#define F32C_MOVE_WAIT_MS        2000U

void app_f32c_gimbal_test(void)
{
    drv_uart_send_string("================================\r\n");
    drv_uart_send_string(" F32C 2-Axis Gimbal UART3 Test\r\n");
    drv_uart_send_string(" X address=1, Y address=2\r\n");
    drv_uart_send_string(" Single-turn absolute mode\r\n");
    drv_uart_send_string("================================\r\n");

    bsp_f32c_gimbal_init();

    /* 电机驱动板上电自检慢于 MCU，必须等其启动完成后再发配置帧。 */
    drv_uart_send_string("[F32C] Waiting motor power-on...\r\n");
    delay_ms(F32C_POWER_ON_WAIT_MS);

    /*
     * 手册规定顺序：使能 -> 选择模式 -> 设置速度 -> 设置目标角度。
     * 两个地址逐轴配置，每一帧之间留出处理时间。
     */
    bsp_f32c_motor_enable(F32C_GIMBAL_X_ID);
    delay_ms(F32C_COMMAND_GAP_MS);
    bsp_f32c_motor_set_single_turn_mode(F32C_GIMBAL_X_ID);
    delay_ms(F32C_COMMAND_GAP_MS);
    if (!bsp_f32c_motor_set_speed(F32C_GIMBAL_X_ID,
                                  F32C_TEST_SPEED_RPM)) {
        drv_uart_send_string("[F32C] Invalid X speed.\r\n");
        while (1) {
        }
    }
    delay_ms(F32C_COMMAND_GAP_MS);
    drv_uart_send_string("[F32C] X configured.\r\n");

    bsp_f32c_motor_enable(F32C_GIMBAL_Y_ID);
    delay_ms(F32C_COMMAND_GAP_MS);
    bsp_f32c_motor_set_single_turn_mode(F32C_GIMBAL_Y_ID);
    delay_ms(F32C_COMMAND_GAP_MS);
    if (!bsp_f32c_motor_set_speed(F32C_GIMBAL_Y_ID,
                                  F32C_TEST_SPEED_RPM)) {
        drv_uart_send_string("[F32C] Invalid Y speed.\r\n");
        while (1) {
        }
    }
    delay_ms(F32C_COMMAND_GAP_MS);
    drv_uart_send_string("[F32C] Y configured.\r\n");

    drv_uart_send_string("[F32C] Initialized. Test loop started.\r\n");

    /* 两轴先逐个回到测试零位。 */
    (void)bsp_f32c_motor_set_single_turn_angle(F32C_GIMBAL_X_ID,
                                                F32C_TEST_HOME_DEG);
    drv_uart_send_string("[F32C] X -> 0.0\r\n");
    delay_ms(F32C_MOVE_WAIT_MS);

    (void)bsp_f32c_motor_set_single_turn_angle(F32C_GIMBAL_Y_ID,
                                                F32C_TEST_HOME_DEG);
    drv_uart_send_string("[F32C] Y -> 0.0\r\n");
    delay_ms(F32C_MOVE_WAIT_MS);

    /* X 轴单独往返。 */
    (void)bsp_f32c_motor_set_single_turn_angle(F32C_GIMBAL_X_ID,
                                                F32C_TEST_X_DEG);
    drv_uart_send_string("[F32C] X -> 30.0\r\n");
    delay_ms(F32C_MOVE_WAIT_MS);

    (void)bsp_f32c_motor_set_single_turn_angle(F32C_GIMBAL_X_ID,
                                                F32C_TEST_HOME_DEG);
    drv_uart_send_string("[F32C] X -> 0.0\r\n");
    delay_ms(F32C_MOVE_WAIT_MS);

    /* Y 轴单独往返。 */
    (void)bsp_f32c_motor_set_single_turn_angle(F32C_GIMBAL_Y_ID,
                                                F32C_TEST_Y_DEG);
    drv_uart_send_string("[F32C] Y -> 30.0\r\n");
    delay_ms(F32C_MOVE_WAIT_MS);

    (void)bsp_f32c_motor_set_single_turn_angle(F32C_GIMBAL_Y_ID,
                                                F32C_TEST_HOME_DEG);
    drv_uart_send_string("[F32C] Y -> 0.0\r\n");
    delay_ms(F32C_MOVE_WAIT_MS);

    /* 近似同步：两条地址帧之间仅留 2ms，供级联电机完成首帧解析。 */
    drv_uart_send_string("[F32C] Dual move start.\r\n");
    (void)bsp_f32c_motor_set_single_turn_angle(F32C_GIMBAL_X_ID,
                                                F32C_TEST_X_DEG);
    delay_ms(F32C_SYNC_FRAME_GAP_MS);
    (void)bsp_f32c_motor_set_single_turn_angle(F32C_GIMBAL_Y_ID,
                                                F32C_TEST_SYNC_Y_DEG);
    drv_uart_send_string("[F32C] X -> 30.0, Y -> 50.0 commands sent.\r\n");
    delay_ms(F32C_MOVE_WAIT_MS);

    /* 双电机回零也使用相同的 2ms 帧间隔。 */
    (void)bsp_f32c_motor_set_single_turn_angle(F32C_GIMBAL_X_ID,
                                                F32C_TEST_HOME_DEG);
    delay_ms(F32C_SYNC_FRAME_GAP_MS);
    (void)bsp_f32c_motor_set_single_turn_angle(F32C_GIMBAL_Y_ID,
                                                F32C_TEST_HOME_DEG);
    drv_uart_send_string("[F32C] X/Y -> 0.0 commands sent.\r\n");
    delay_ms(F32C_MOVE_WAIT_MS);

    /* 测试结束后逐轴失能，避免两个级联帧无间隔导致第二帧丢失。 */
    bsp_f32c_motor_disable(F32C_GIMBAL_X_ID);
    delay_ms(F32C_COMMAND_GAP_MS);
    bsp_f32c_motor_disable(F32C_GIMBAL_Y_ID);
    delay_ms(F32C_COMMAND_GAP_MS);
    drv_uart_send_string("[F32C] Test finished. Motors disabled.\r\n");

    while (1) {
    }
}
