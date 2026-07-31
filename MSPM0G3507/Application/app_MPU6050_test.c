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

/* ---- 以下用于 MPU6050_straight_test ---- */
#include "bsp_encoder.h"
#include "bsp_button.h"
#include "bsp_led.h"
#include "mid_pid.h"

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



void MPU6050_start(void)
{
    MPU6050_init();
    delay_ms(300);
    /* ---- 归零 ---- */
    MPU6050_Set_Angle0(&MM);
    /* ---- 启动定时器 ---- */
    drv_mpu6050_timer_start();


}


/* ================================================================
 * MPU6050 航向 PID → 编码电机开环走直线
 *
 * 操作: KEY1 短按启动, 再次短按停止
 * 原理: 锁住启动时的 yaw 角为目标航向, PID 计算左右轮差速修正量
 *       电机侧不开编码器反馈, 仅靠陀螺仪航向闭环
 *
 * 依赖: MPU6050_start() 完成初始化 + 5ms 定时器持续解算 MM.yaw
 * ================================================================ */

/* ---- 控制参数 ---- */
#define MPUS_BASE_DUTY            15U
#define MPUS_MAX_CORRECTION       20U
#define MPUS_DEAD_ZONE            3U
#define MPUS_MAX_DUTY             60U
#define MPUS_CONTROL_MS           10U
#define MPUS_PRINT_MS             200U
#define MPUS_OLED_MS              200U

/* ---- 航向 PID ---- */
#define MPUS_KP                   0.65f
#define MPUS_KI                   0.005f
#define MPUS_KD                   0.55f
#define MPUS_INTEGRAL_LIMIT       15.0f
#define MPUS_OUTPUT_LIMIT         ((float)MPUS_MAX_CORRECTION)

static Button g_mpus_btn;

/** 归一化到 [-180, 180] */
static float mpus_norm(float a)
{
    while (a > 180.0f)  a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}

/** 安全驱动单路电机 */
static void mpus_drive(MG310_Motor m, int32_t d)
{
    uint32_t abs_d;
    if (d > (int32_t)MPUS_DEAD_ZONE) {
        abs_d = (d > (int32_t)MPUS_MAX_DUTY) ? MPUS_MAX_DUTY : (uint32_t)d;
        mg310_motorForward(m, abs_d);
    } else if (d < -(int32_t)MPUS_DEAD_ZONE) {
        abs_d = (-d > (int32_t)MPUS_MAX_DUTY) ? MPUS_MAX_DUTY : (uint32_t)(-d);
        mg310_motorReverse(m, abs_d);
    } else {
        mg310_motorStop(m);
    }
}

void MPU6050_straight_test(void)
{
    uint8_t  run = 0U;
    float    target, err, corr;
    int32_t  ld = 0, rd = 0;
    uint32_t lp, lo, tick;
    PID_Controller pid;

    /* ---- UART 调试串口 ---- */
    drv_uart0_init();

    /* ---- OLED ---- */
    OLED_Init();

    /* ---- 复用 MPU6050_start 完成传感器/Timer 初始化 ---- */
    MPU6050_start();

    drv_uart_send_string("\r\n========================================\r\n");
    drv_uart_send_string(" MPU6050 Straight-Line Test\r\n");
    drv_uart_send_string(" Yaw PID -> differential drive\r\n");
    drv_uart_send_string(" KEY1: Start/Stop\r\n");
    drv_uart_send_string("========================================\r\n");

    /* ---- MPU6050 设备检测 ---- */
    {
        uint8_t id = MPU6050_ID();
        drv_uart_send_string("[CHK] MPU6050 ID=0x");
        drv_uart_print_hex(id);
        if (id == 0x68) {
            drv_uart_send_string(" OK\r\n");
        } else {
            drv_uart_send_string(" FAIL (expected 0x68)!\r\n");
        }
    }

    /* ---- 电机 & 按键 ---- */
    mg310_motorInitAll();
    button_init(&g_mpus_btn, KEY_PORT, KEY_KEY1_PIN, BUTTON_ACTIVE_HIGH);

    drv_uart_send_string("[INIT] PID: Kp=0.65 Ki=0.005 Kd=0.45, base=15%\r\n");
    drv_uart_send_string("[INIT] Press KEY1 to start.\r\n");
    drv_uart_send_string("[DBG]  Yaw will print every 200ms.\r\n\r\n");

    OLED_Clear();
    OLED_ShowString(1, 1, "MPU6050 Straight");
    OLED_ShowString(2, 1, "KEY1: Start   ");
    OLED_ShowString(3, 1, "Yaw: ---.-    ");
    OLED_ShowString(4, 1, "L:--% R:--%   ");

    delay_ms(1U);
    lp = get_system_ms();
    lo = lp;

    while (1) {
        button_update(&g_mpus_btn);
        tick = get_system_ms();

        /* 未运行时也周期性打印 yaw，确认传感器工作 */
        if ((run == 0U) && ((tick - lp) >= MPUS_PRINT_MS)) {
            lp = tick;
            drv_uart_send_string("[IDLE] Yaw=");
            drv_uart_print_signed((long)(MM.yaw * 10.0f));
            drv_uart_send_string(" (x10 deg)\r\n");
            use_led_TOGGLE();
        }

        /* KEY1 切换 启动/停止 */
        if (button_is_clicked(&g_mpus_btn)) {
            if (run == 0U) {
                target = MM.yaw;  /* 锁住当前航向 */
                pid_init(&pid, MPUS_KP, MPUS_KI, MPUS_KD,
                         MPUS_INTEGRAL_LIMIT, MPUS_OUTPUT_LIMIT);
                run = 1U;
                lp = tick;
                use_led_ON();
                drv_uart_send_string("[RUN] Target=");
                drv_uart_print_signed((long)(target * 10.0f));
                drv_uart_send_string("\r\n");
            } else {
                mg310_motorStopAll();
                run = 0U; ld = 0; rd = 0;
                use_led_OFF();
                drv_uart_send_string("[STOP]\r\n");
            }
        }

        if (run != 0U) {
            /* 航向误差 + PID */
            err  = mpus_norm(MM.yaw - target);
            corr = -pid_update(&pid, err);

            /* 差速: corr>0→偏左, 右轮加速回正 */
            ld = (int32_t)MPUS_BASE_DUTY - (int32_t)corr;
            rd = (int32_t)MPUS_BASE_DUTY + (int32_t)corr;

            if (ld < 0) ld = 0; else if (ld > (int32_t)MPUS_MAX_DUTY) ld = MPUS_MAX_DUTY;
            if (rd < 0) rd = 0; else if (rd > (int32_t)MPUS_MAX_DUTY) rd = MPUS_MAX_DUTY;

            mpus_drive(MG310_MOTOR_A, ld);
            mpus_drive(MG310_MOTOR_B, rd);

            if ((tick - lp) >= MPUS_PRINT_MS) {
                lp = tick;
                drv_uart_print_num((unsigned long)tick);
                drv_uart_send_string(" Yaw=");
                drv_uart_print_signed((long)(MM.yaw * 10.0f));
                drv_uart_send_string(" Err=");
                drv_uart_print_signed((long)(err * 10.0f));
                drv_uart_send_string(" Corr=");
                drv_uart_print_signed((long)(corr * 10.0f));
                drv_uart_send_string(" L=");
                drv_uart_print_num((unsigned long)ld);
                drv_uart_send_string(" R=");
                drv_uart_print_num((unsigned long)rd);
                drv_uart_send_string("\r\n");
                use_led_TOGGLE();
            }
        }

        /* OLED */
        if ((tick - lo) >= MPUS_OLED_MS) {
            lo = tick;
            OLED_ShowString(2, 1, run ? "Running...    " : "KEY1: Start   ");
            {
                int32_t t = (int32_t)(MM.yaw * 10.0f);
                OLED_ShowChar(3, 5, (t >= 0) ? '+' : '-');
                if (t < 0) t = -t;
                OLED_ShowNum(3, 6, (uint32_t)(t / 10), 3);
                OLED_ShowChar(3, 9, '.');
                OLED_ShowNum(3, 10, (uint32_t)(t % 10), 1);
            }
            if (run) {
                OLED_ShowNum(4, 3, (uint32_t)ld, 2);
                OLED_ShowChar(4, 5, '%');
                OLED_ShowNum(4, 9, (uint32_t)rd, 2);
                OLED_ShowChar(4, 11, '%');
            } else {
                OLED_ShowString(4, 1, "L:--% R:--%   ");
            }
        }

        delay_ms(MPUS_CONTROL_MS);
    }
}


void app_mpu6050_accel_test(void)
{
    float ax, ay, az;

    /* ---- UART 调试串口 ---- */
    drv_uart0_init();
    MPU6050_init();
    delay_ms(300);

    drv_uart_send_string("\r\n========================================\r\n");
    drv_uart_send_string(" MPU6050 Accel Test\r\n");
    drv_uart_send_string("========================================\r\n");

    while (1) {
        MPU6050_Get_Accel(&ax, &ay, &az);
        drv_uart_send_string("Accel: X=");
        drv_uart_print_signed((long)(ax * 1000));
        drv_uart_send_string(" Y=");
        drv_uart_print_signed((long)(ay * 1000));
        drv_uart_send_string(" Z=");
        drv_uart_print_signed((long)(az * 1000));
        drv_uart_send_string(" (mg)\r\n");
        delay_ms(200U);
    }
}
