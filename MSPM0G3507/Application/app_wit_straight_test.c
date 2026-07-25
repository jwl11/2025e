#include "ti_msp_dl_config.h"
#include "bsp_wit.h"
#include "bsp_encoder.h"
#include "bsp_led.h"
#include "bsp_button.h"
#include "bsp_OLED.h"
#include "drv_uart.h"
#include "mid_delay.h"
#include "mid_pid.h"

/* ================================================================
 * WIT 陀螺仪航向开环控制 — 编码电机走直线 (Application 层)
 *
 * 原理:
 *   利用陀螺仪 yaw 角作为航向反馈, PID 计算左右轮差速修正量,
 *   补偿两轮转速不一致带来的偏航, 使小车走直线。
 *
 *   电机本身不接编码器反馈 (开环 PWM), 仅靠陀螺仪航向闭环。
 *
 * 操作:
 *   按 KEY1 (PB15) → 启动, 再次按 KEY1 → 停止
 *
 * 分层调用:
 *   app_wit_straight_test
 *     ├── WIT_Init / wit_data         [BSP]  陀螺仪数据
 *     ├── mg310_motor*                [BSP]  电机 PWM 驱动
 *     ├── pid_update                  [Middleware] 航向 PID
 *     ├── button_init / button_update [BSP]  按键
 *     ├── OLED_*                      [BSP]  显示
 *     ├── drv_uart_*                  [Driver] 串口调试
 *     └── delay_ms / get_system_ms    [Middleware] 时基
 * ================================================================ */

/* ---- 控制参数 ---- */
#define BASE_DUTY               10U    /* 基础占空比 (%)                  */
#define MAX_CORRECTION          20U    /* 最大差速修正量 (%)              */
#define MIN_DUTY_DEAD_ZONE      3U     /* 占空比死区 (低于此值停车)      */
#define MAX_SAFE_DUTY           70U    /* 安全占空比上限                  */
#define CONTROL_PERIOD_MS       10U    /* 控制循环周期 (ms)               */
#define PRINT_PERIOD_MS         200U   /* 调试打印周期 (ms)               */
#define OLED_PERIOD_MS          200U   /* OLED 刷新周期 (ms)              */

/* ---- 航向 PID 参数 ---- */
#define HEADING_KP              2.5f
#define HEADING_KI              0.08f
#define HEADING_KD              0.3f
#define HEADING_INTEGRAL_LIMIT  20.0f
#define HEADING_OUTPUT_LIMIT    ((float)MAX_CORRECTION)

/* ---- 全局按键句柄 ---- */
static Button g_btn_key1;

/* ================================================================
 *  辅助函数
 * ================================================================ */

/**
 * @brief  归一化角度到 [-180, 180] 范围
 * @note   陀螺仪 yaw 累计可能超过 ±180, PID 需要连续的误差
 */
static float normalize_angle(float angle)
{
    while (angle > 180.0f) {
        angle -= 360.0f;
    }
    while (angle < -180.0f) {
        angle += 360.0f;
    }
    return angle;
}

/**
 * @brief  安全地驱动单路电机 (带死区和限幅)
 */
static void motor_drive_safe(MG310_Motor motor, int32_t duty)
{
    uint32_t abs_duty;

    if (duty > (int32_t)MIN_DUTY_DEAD_ZONE) {
        abs_duty = (uint32_t)duty;
        if (abs_duty > MAX_SAFE_DUTY) {
            abs_duty = MAX_SAFE_DUTY;
        }
        mg310_motorForward(motor, abs_duty);
    } else if (duty < -(int32_t)MIN_DUTY_DEAD_ZONE) {
        abs_duty = (uint32_t)(-duty);
        if (abs_duty > MAX_SAFE_DUTY) {
            abs_duty = MAX_SAFE_DUTY;
        }
        mg310_motorReverse(motor, abs_duty);
    } else {
        mg310_motorStop(motor);
    }
}

/* ================================================================
 *  主入口
 * ================================================================ */

void app_wit_straight_test(void)
{
    uint8_t  running          = 0U;
    float    target_yaw       = 0.0f;
    float    heading_error;
    float    correction;
    int32_t  left_duty        = 0;
    int32_t  right_duty       = 0;
    uint32_t last_print_ms;
    uint32_t last_oled_ms;
    uint32_t tick;
    PID_Controller pid_heading;

    /* ---- 初始化 ---- */
    drv_uart0_init();
    drv_uart_send_string("\r\n========================================\r\n");
    drv_uart_send_string(" WIT Gyro Straight-Line Test\r\n");
    drv_uart_send_string(" Yaw heading PID -> differential drive\r\n");
    drv_uart_send_string(" KEY1 (PB15): Start / Stop\r\n");
    drv_uart_send_string("========================================\r\n");

    OLED_Init();
    WIT_Init();
    mg310_motorInitAll();
    button_init(&g_btn_key1, KEY_PORT, KEY_KEY1_PIN, BUTTON_ACTIVE_HIGH);

    drv_uart_send_string("[INIT] OLED, WIT, MG310, Button ready.\r\n");
    drv_uart_send_string("[INIT] Heading PID: Kp=");
    drv_uart_print_num((unsigned long)(HEADING_KP * 100.0f));
    drv_uart_send_string("/100 Ki=");
    drv_uart_print_num((unsigned long)(HEADING_KI * 1000.0f));
    drv_uart_send_string("/1000 Kd=");
    drv_uart_print_num((unsigned long)(HEADING_KD * 100.0f));
    drv_uart_send_string("/100\r\n");
    drv_uart_send_string("[INIT] Base duty=");
    drv_uart_print_num(BASE_DUTY);
    drv_uart_send_string("% Max correction=+-");
    drv_uart_print_num(MAX_CORRECTION);
    drv_uart_send_string("%\r\n");
    drv_uart_send_string("[INIT] Press KEY1 to start.\r\n\r\n");

    /* OLED 初始画面 */
    OLED_Clear();
    OLED_ShowString(1, 1, "WIT Straight");
    OLED_ShowString(2, 1, "KEY1: Start   ");
    OLED_ShowString(3, 1, "Yaw: ---.-    ");
    OLED_ShowString(4, 1, "L:--% R:--%   ");

    delay_ms(1U);  /* 启动 SysTick */
    last_print_ms = get_system_ms();
    last_oled_ms  = last_print_ms;

    /* ---- 主循环 ---- */
    while (1) {
        /* ① 按键消抖采样 */
        button_update(&g_btn_key1);

        /* ② KEY1 切换 启动 / 停止 */
        if (button_is_clicked(&g_btn_key1)) {
            if (running == 0U) {
                /* ---- 启动: 记录当前航向, 复位 PID ---- */
                target_yaw = wit_data.yaw;
                pid_init(&pid_heading,
                         HEADING_KP, HEADING_KI, HEADING_KD,
                         HEADING_INTEGRAL_LIMIT, HEADING_OUTPUT_LIMIT);
                running = 1U;

                drv_uart_send_string("[RUN]  Started. Target yaw = ");
                drv_uart_print_signed((long)(target_yaw * 10.0f));
                drv_uart_send_string(" (x10 deg)\r\n");

                drv_uart_send_string("Time(ms) | Yaw(x10)  Err(x10)");
                drv_uart_send_string("  Corr(x10) | Lduty Rduty\r\n");
                drv_uart_send_string("---------+---------------------");
                drv_uart_send_string("------------+---------------\r\n");

                last_print_ms = get_system_ms();
                use_led_ON();
            } else {
                /* ---- 停止 ---- */
                mg310_motorStopAll();
                running  = 0U;
                left_duty  = 0;
                right_duty = 0;
                use_led_OFF();

                drv_uart_send_string("[STOP] Motors stopped.\r\n\r\n");
            }
        }

        tick = get_system_ms();

        /* ③ 运行状态: 航向 PID → 差速驱动 */
        if (running != 0U) {
            /*
             * 航向误差计算 (带 wrap-around 处理):
             *   error = current_yaw - target_yaw
             *   归一化到 [-180, 180], 正 = 右偏, 负 = 左偏
             */
            heading_error = wit_data.yaw - target_yaw;
            heading_error = normalize_angle(heading_error);

            /* PID 计算修正量 (取反匹配陀螺仪安装极性) */
            correction = -pid_update(&pid_heading, heading_error);

            /*
             * 差速分配:
             *   correction > 0 → 车偏左, 右轮加速/左轮减速 → 向右回正
             *   correction < 0 → 车偏右, 左轮加速/右轮减速 → 向左回正
             */
            left_duty  = (int32_t)BASE_DUTY - (int32_t)correction;
            right_duty = (int32_t)BASE_DUTY + (int32_t)correction;

            /* 限幅到安全范围 */
            if (left_duty < 0) {
                left_duty = 0;
            } else if (left_duty > (int32_t)MAX_SAFE_DUTY) {
                left_duty = (int32_t)MAX_SAFE_DUTY;
            }
            if (right_duty < 0) {
                right_duty = 0;
            } else if (right_duty > (int32_t)MAX_SAFE_DUTY) {
                right_duty = (int32_t)MAX_SAFE_DUTY;
            }

            /* 驱动电机 */
            motor_drive_safe(MG310_MOTOR_A, left_duty);
            motor_drive_safe(MG310_MOTOR_B, right_duty);

            /* 调试打印 */
            if ((tick - last_print_ms) >= PRINT_PERIOD_MS) {
                last_print_ms = tick;

                drv_uart_print_num((unsigned long)tick);
                drv_uart_send_string(" | ");
                drv_uart_print_signed((long)(wit_data.yaw * 10.0f));
                drv_uart_send_string("    ");
                drv_uart_print_signed((long)(heading_error * 10.0f));
                drv_uart_send_string("    ");
                drv_uart_print_signed((long)(correction * 10.0f));
                drv_uart_send_string("       |  ");
                drv_uart_print_num((unsigned long)left_duty);
                drv_uart_send_string("%   ");
                drv_uart_print_num((unsigned long)right_duty);
                drv_uart_send_string("%\r\n");

                use_led_TOGGLE();
            }
        }

        /* ④ OLED 刷新 */
        if ((tick - last_oled_ms) >= OLED_PERIOD_MS) {
            last_oled_ms = tick;

            /* 行 2: 运行状态 */
            if (running != 0U) {
                OLED_ShowString(2, 1, "Running...    ");
            } else {
                OLED_ShowString(2, 1, "KEY1: Start   ");
            }

            /* 行 3: 当前 yaw 角 */
            {
                int32_t tenths = (int32_t)(wit_data.yaw * 10.0f);
                OLED_ShowChar(3, 5, (tenths >= 0) ? '+' : '-');
                if (tenths < 0) {
                    tenths = -tenths;
                }
                OLED_ShowNum(3, 6, (uint32_t)(tenths / 10), 3);
                OLED_ShowChar(3, 9, '.');
                OLED_ShowNum(3, 10, (uint32_t)(tenths % 10), 1);
                OLED_ShowString(3, 11, "   ");
            }

            /* 行 4: 左右占空比 */
            if (running != 0U) {
                OLED_ShowNum(4, 3, (uint32_t)left_duty, 2);
                OLED_ShowChar(4, 5, '%');
                OLED_ShowNum(4, 9, (uint32_t)right_duty, 2);
                OLED_ShowChar(4, 11, '%');
            } else {
                OLED_ShowString(4, 1, "L:--% R:--%   ");
            }
        }

        delay_ms(CONTROL_PERIOD_MS);
    }
}
