#include "bsp_fishpath.h"
#include "bsp_encoder.h"
#include "drv_uart.h"

/* ================================================================
 * 循迹 BSP 层 — MG310 差速驱动
 *
 * 基于 mid_xunji 计算的左右占空比, 控制 MG310 双电机实现
 * 差速转向 (左轮 vs 右轮 = 逆时针 vs 顺时针转弯)。
 * ================================================================ */

static uint8_t fishpath_running = 0;  /* 循迹启停标志 */

/**
 * @brief  循迹系统初始化
 */
void fishpath_init(float Kp, float Ki, float Kd, uint32_t base_speed)
{
    /* 1. 初始化双路电机 (置停) */
    mg310_motorInitAll();

    /* 2. 使能 UART1 中断接收 (硬件已由 SysConfig 配置) */
    drv_uart1_init();

    /* 3. 初始化循迹中间层 (PID + 传感器缓冲区) */
    xunji_init(Kp, Ki, Kd, base_speed);

    fishpath_running = 1;
}

/**
 * @brief  循迹主更新 (每个控制周期调用)
 *
 *         流程:
 *           xunji_update() → 传感器 → 偏差 → PID → 左右占空比
 *           → 限定 0~100
 *           → mg310_motorForward(L) / mg310_motorForward(R)
 */
void fishpath_update(void)
{
    int32_t left, right;

    if (!fishpath_running) {
        return;
    }

    /* 1. 循迹中间层: 传感器数据 → PID → 左右占空比 */
    xunji_update();

    /* 2. 获取计算好的占空比 */
    left  = xunji_get_left_duty();
    right = xunji_get_right_duty();

    /* 3. 安全限幅 (中间层已限幅, 此处二次确认) */
    if (left  < 0)  left  = 0;
    if (left  > 100) left = 100;
    if (right < 0)  right = 0;
    if (right > 100) right = 100;

    /* 4. 写入 MG310 双电机 (差速转向) */
    /*
     * 电机映射:
     *   MG310_MOTOR_A → 左轮 (接 TIMA1 CH0, PA15)
     *   MG310_MOTOR_B → 右轮 (接 TIMA1 CH1, PA16)
     *
     *   注意: 实际车体前进方向可能因机械安装反转,
     *         若前进变成后退, 交换电机 A/B 或使用 Reverse 即可。
     */
    if (left > 0) {
        mg310_motorForward(MG310_MOTOR_A, (uint32_t)left);
    } else {
        mg310_motorStop(MG310_MOTOR_A);
    }

    if (right > 0) {
        mg310_motorForward(MG310_MOTOR_B, (uint32_t)right);
    } else {
        mg310_motorStop(MG310_MOTOR_B);
    }
}

/**
 * @brief  在线修改基础速度
 */
void fishpath_set_speed(uint32_t speed)
{
    xunji_set_base_speed(speed);
}

/**
 * @brief  启动循迹
 */
void fishpath_start(void)
{
    fishpath_running = 1;
}

/**
 * @brief  停止循迹 (电机置停)
 */
void fishpath_stop(void)
{
    fishpath_running = 0;
    mg310_motorStopAll();
}
