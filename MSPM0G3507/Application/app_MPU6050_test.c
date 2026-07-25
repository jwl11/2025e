/*
 * MPU6050 + OLED 例程 (MSPM0G3507)
 *
 * 硬件连接:
 *   MPU6050 SCL  PA30
 *   MPU6050 SDA  PB22
 *   OLED    SCL  PB9
 *   OLED    SDA  PB8
 *   UART0   TX   PA10  (debug)
 *   UART0   RX   PA11
 *
 * 注意: OLED_ShowString 的 Line/Column 从 1 开始 (1~4, 1~16)
 */

#include "ti_msp_dl_config.h"
#include "MPU6050.h"
#include "drv_tim.h"
#include "drv_uart.h"
#include "mid_delay.h"
#include "bsp_OLED.h"
#include "stdio.h"

MPU6050 MM;

void MPU6050_test(void)
{
    char buf[17];

    /* ---- OLED 初始化 ---- */
    OLED_Init();
    OLED_Clear();
    OLED_ShowString(1, 1, "MPU6050 Demo");
    OLED_ShowString(2, 1, "Init I2C...");
    delay_ms(300);

    /* ---- MPU6050 初始化 ---- */
    MPU6050_init();
    OLED_Clear();
    OLED_ShowString(1, 1, "Init MPU6050 OK");
    delay_ms(300);

    /* ---- 检测设备 ---- */
    uint8_t id = MPU6050_ID();
    if (id == 0x68) {
        OLED_ShowString(2, 1, "ID:0x68 OK");
    } else {
        sprintf(buf, "ID:0x%02X FAIL", id);
        OLED_ShowString(2, 1, buf);
        OLED_ShowString(3, 1, "Check HW!");
        while (1);
    }
    delay_ms(300);

    /* ---- 归零 ---- */
    OLED_ShowString(3, 1, "Zero calibrate..");
    delay_ms(500);
    MPU6050_Set_Angle0(&MM);
    OLED_ShowString(3, 1, "Zero OK, start");

    /* ---- 启动定时器 ---- */
    drv_mpu6050_timer_start();

    OLED_Clear();

    /* ---- 主循环 ---- */
    while (1) {

        sprintf(buf, "Roll:%.2f", MM.roll);
        OLED_ShowString(1, 1, buf);

        sprintf(buf, "Pitch:%.2f", MM.pitch);
        OLED_ShowString(2, 1, buf);

        sprintf(buf, "Yaw:%.2f", MM.yaw);
        OLED_ShowString(3, 1, buf);

        sprintf(buf, "Temp:%.1fC", MPU6050_GetTemp(&MM));
        OLED_ShowString(4, 1, buf);

        delay_ms(20);
    }
}
