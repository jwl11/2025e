#include "app.h"
#include "bsp_42step.h"
#include "bsp_fishpath.h"
#include "bsp_OLED.h"
#include "MPU6050.h"
#include "drv_uart.h"
#include "mid_delay.h"
#include "mid_pid.h"
#include <stdlib.h>   /* atoi */

/* ================================================================
 * 小球平衡 — 双环 PID + 串口实时调参
 *
 *   串口命令: vkp=N vkd=N pkp=N pkd=N pki=N db=N ff=N mp=N ?
 * ================================================================ */

/* ---- 运行时参数 (串口可改) ---- */
static float   g_vel_kp = 100.0f;
static float   g_vel_ki = 0.0f;
static float   g_vel_kd = 200.0f;
static float   g_pos_kp = 0.0f;
static float   g_pos_ki = 0.0f;
static float   g_pos_kd = 0.0f;
static int16_t g_deadband = 2;
static int16_t g_max_pos = 300;
static float   g_imu_ff = 0.0f;

#define VEL_I_LIMIT    50.0f
#define VEL_OUT_LIMIT 500.0f
#define POS_I_LIMIT    50.0f
#define POS_OUT_LIMIT 200.0f

#define PERIOD_MS        5
#define POS_SPEED     2000
#define POS_ACC        255
#define TRACK_SPEED     30

static PID_Controller g_pid_vel, g_pid_pos;
static int16_t prev_err;

/* ---- 串口命令解析 ---- */
static void cmd_parse(void)
{
    char buf[32];
    uint8_t i = 0;
    int16_t ch;
    while ((ch = drv_uart0_getchar()) >= 0 && i < 31) {
        char c = (char)ch;
        if (c == '\r' || c == '\n') break;
        buf[i++] = c;
    }
    if (i == 0) return;
    buf[i] = '\0';

    float val = 0;
    char *eq = 0;
    for (uint8_t j = 0; j < i; j++) { if (buf[j] == '=') { eq = &buf[j]; break; } }
    if (eq) { *eq = '\0'; val = (float)atof(eq + 1); }

    if      (eq && !strcmp(buf, "vkp"))  { g_vel_kp = val; drv_uart_send_string("VEL_KP="); }
    else if (eq && !strcmp(buf, "vki"))  { g_vel_ki = val; drv_uart_send_string("VEL_KI="); }
    else if (eq && !strcmp(buf, "vkd"))  { g_vel_kd = val; drv_uart_send_string("VEL_KD="); }
    else if (eq && !strcmp(buf, "pkp"))  { g_pos_kp = val; drv_uart_send_string("POS_KP="); }
    else if (eq && !strcmp(buf, "pki"))  { g_pos_ki = val; drv_uart_send_string("POS_KI="); }
    else if (eq && !strcmp(buf, "pkd"))  { g_pos_kd = val; drv_uart_send_string("POS_KD="); }
    else if (eq && !strcmp(buf, "db"))   { g_deadband = (int16_t)val; drv_uart_send_string("DEADBAND="); }
    else if (eq && !strcmp(buf, "mp"))   { g_max_pos = (int16_t)val; drv_uart_send_string("MAX_POS="); }
    else if (eq && !strcmp(buf, "ff"))   { g_imu_ff = val; drv_uart_send_string("IMU_FF="); }
    else if (!strcmp(buf, "?")) {
        drv_uart_send_string("vkp=");  drv_uart_print_num((unsigned long)g_vel_kp);
        drv_uart_send_string(" vkd="); drv_uart_print_num((unsigned long)g_vel_kd);
        drv_uart_send_string("\r\npkp="); drv_uart_print_num((unsigned long)g_pos_kp);
        drv_uart_send_string(" pkd="); drv_uart_print_num((unsigned long)g_pos_kd);
        drv_uart_send_string("\r\ndb=");  drv_uart_print_num((unsigned long)g_deadband);
        drv_uart_send_string(" mp=");  drv_uart_print_num((unsigned long)g_max_pos);
        drv_uart_send_string(" ff=");  drv_uart_print_num((unsigned long)g_imu_ff);
        drv_uart_send_string("\r\n");
        return;
    } else { drv_uart_send_string("? "); return; }
    drv_uart_print_num((unsigned long)(long)val);
    drv_uart_send_string("\r\n");
}

/* ---- 重新初始化 PID 参数 ---- */
static void pid_reload(void)
{
    g_pid_vel.Kp = g_vel_kp; g_pid_vel.Ki = g_vel_ki; g_pid_vel.Kd = g_vel_kd;
    g_pid_pos.Kp = g_pos_kp; g_pid_pos.Ki = g_pos_ki; g_pid_pos.Kd = g_pos_kd;
}

void topic4(void)
{
    OLED_Init(); OLED_Clear();
    OLED_ShowString(1, 1, "topic4: Tune");

    drv_uart0_init();
    drv_vision_uart3_init();
    while (drv_vision_get_x() == 0) { delay_ms(10); }
    drv_uart_send_string("CAM OK\r\nCommands: vkp= vkd= pkp= pkd= db= mp= ff= ?\r\n");

    pid_init(&g_pid_vel, g_vel_kp, g_vel_ki, g_vel_kd, VEL_I_LIMIT, VEL_OUT_LIMIT);
    pid_init(&g_pid_pos, g_pos_kp, g_pos_ki, g_pos_kd, POS_I_LIMIT, POS_OUT_LIMIT);

    MPU6050_init();
    delay_ms(300);

    Step42_Init();
    fishpath_init(XUNJI_DEFAULT_KP, XUNJI_DEFAULT_KI,
                  XUNJI_DEFAULT_KD, XUNJI_DEFAULT_SPEED);
    fishpath_set_speed(TRACK_SPEED);
    fishpath_stop();
    Step42_Enable(true);
    delay_ms(500);
    fishpath_start();

    while (1) {
        /* 串口调参 */
        cmd_parse();
        pid_reload();

        int16_t err_pos = app_ball_error_cm_x10();
        int16_t vel_raw = err_pos - prev_err;
        prev_err = err_pos;
        float actual_vel = (float)vel_raw;

        float err_cm     = (float)err_pos / 10.0f;
        float target_vel = pid_update(&g_pid_pos, err_cm);

        float vel_err   = target_vel - actual_vel;
        float motor_out = pid_update(&g_pid_vel, vel_err);

        float ax, ay, az;
        MPU6050_Get_Accel(&ax, &ay, &az);
        motor_out += ay * g_imu_ff;

        int32_t pos;
        if (err_pos > -g_deadband && err_pos < g_deadband)
            pos = 0;
        else
            pos = (int32_t)motor_out;

        if (pos >  g_max_pos) pos =  g_max_pos;
        if (pos < -g_max_pos) pos = -g_max_pos;

        static uint8_t dbg;
        if (++dbg >= 40) { dbg = 0;
            drv_uart_send_string("e=");  drv_uart_print_signed(err_pos);
            drv_uart_send_string(" v="); drv_uart_print_signed(vel_raw);
            drv_uart_send_string(" p="); drv_uart_print_signed(pos);
            drv_uart_send_string("\r\n");
        }

        Step42Dir dir = (pos >= 0) ? STEP42_DIR_CCW : STEP42_DIR_CW;
        uint32_t pulses = (uint32_t)(pos >= 0 ? pos : -pos);
        if (pulses > 0)
            Step42_MovePosition(dir, POS_SPEED, POS_ACC,
                                pulses, STEP42_POS_RELATIVE);

        delay_ms(PERIOD_MS);
    }
}
