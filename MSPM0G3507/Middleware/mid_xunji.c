#include "mid_xunji.h"
#include "drv_uart.h"

/* ================================================================
 * 12 路循迹传感器数据处理
 *
 * 传感器编号: 0  1  2  3  4  5  6  7  8  9  10 11
 *             ← 左                      右 →
 * 权值:    -5500  ...  -500  +500  ...  +5500
 * 中心:     传感器 5 和 6 之间 (权值=0)
 *
 * 数据协议 (UART1):
 *   循迹模块连续发送 ASCII '0' (0x30) / '1' (0x31),
 *   每帧 12 字节对应 12 路传感器状态。
 *   帧同步策略: 累计 12 个有效字节即视为一帧。
 *
 * 差速公式:
 *   left  = base_speed + pid_output
 *   right = base_speed - pid_output
 *   两端均 clamp 到 [0, 100]
 * ================================================================ */

/* 循迹专用 PID 实例 */
PID_Controller g_pid_xunji = {0};

/* Adaptive cornering speeds (PWM duty percent). */
#define XUNJI_CENTER_ERROR_MAX       500
#define XUNJI_SMALL_CURVE_ERROR_MAX  1500
#define XUNJI_SMALL_CURVE_SPEED      20U
#define XUNJI_SHARP_CURVE_SPEED      15U

/*
 * Tracking center offset in sensor-weight units (1000 per channel).
 * A positive value moves the target toward the higher sensor index.
 */
#define XUNJI_CENTER_OFFSET          1000

/* ---- 内部状态 ---- */
static uint8_t  sensor_raw[XUNJI_SENSOR_COUNT];   /* '0'=白, '1'=黑 */
static uint32_t base_speed;                        /* 基础速度 0~100 */
static int32_t  left_duty;                         /* 左轮占空比 */
static int32_t  right_duty;                        /* 右轮占空比 */
static int32_t  error_raw;                         /* 加权偏差 (-5500~+5500) */
static int32_t  position_raw;                      /* 加权位置 (0~11000) */
static uint8_t  sensor_online;                     /* 是否有线 */
static uint8_t  byte_index;                        /* 帧内字节序号 0~11 */

/**
 * @brief  传感器权值查找表
 *
 *         weight[i] = i * 1000 - 5500
 *         范围: -5500 (最左) → +5500 (最右)
 */
static const int16_t xunji_weight[XUNJI_SENSOR_COUNT] = {
    -5500, -4500, -3500, -2500, -1500, -500,
     500,  1500,  2500,  3500,  4500,  5500
};

/**
 * @brief  初始化循迹 PID 及状态变量
 */
void xunji_init(float Kp, float Ki, float Kd, uint32_t speed)
{
    /* PID 初始化 */
    pid_init(&g_pid_xunji,
             Kp, Ki, Kd,
             3000.0f,     /* integral_limit: 防止积分过深 */
             50.0f);      /* output_limit: 最大差速调整量 */

    /* 状态初始化 */
    base_speed   = speed;
    left_duty    = 0;
    right_duty   = 0;
    error_raw    = 0;
    position_raw = 0;
    sensor_online = 0;
    byte_index   = 0;

    /* 清空传感器数组 */
    uint8_t i;
    for (i = 0; i < XUNJI_SENSOR_COUNT; i++) {
        sensor_raw[i] = 0;
    }

    /* 清空 UART1 缓冲区 */
    drv_uart1_flush();
}

/**
 * @brief  从 UART1 环形缓冲区解析传感器数据
 *
 *         每次从缓冲区读取一个字节:
 *           '0' (0x30) → sensor = 0 (白)
 *           '1' (0x31) → sensor = 1 (黑)
 *           '\r' / '\n' → 帧分隔符, 重置字节序号
 *           其他字节 → 忽略, 不改变序号
 *
 *         连续 12 个有效数据字节填满一帧后自动覆盖旧帧。
 */
static void xunji_parse_uart(void)
{
    int16_t byte;

    while (drv_uart1_available() > 0) {
        byte = drv_uart1_read();
        if (byte < 0) break;

        switch (byte) {
        case '0':                         /* 白线 (或反射率低) */
            sensor_raw[byte_index] = 0;
            byte_index = (byte_index + 1) % XUNJI_SENSOR_COUNT;
            break;

        case '1':                         /* 黑线 (或反射率高) */
            sensor_raw[byte_index] = 1;
            byte_index = (byte_index + 1) % XUNJI_SENSOR_COUNT;
            break;

        case '\r':                        /* 帧分隔 — 回车 */
        case '\n':                        /* 帧分隔 — 换行 */
            byte_index = 0;
            break;

        default:                          /* 无效字节, 忽略 */
            break;
        }
    }
}

/**
 * @brief  加权误差计算
 *
 *         weighted_sum  = Σ(weight[i] × sensor[i])
 *         active_count  = Σ(sensor[i])
 *
 *         if active > 0:
 *           position = weighted_sum / active_count   (-5500~+5500)
 *           error    = position                      (中心=0)
 *         else:
 *           error = 0, online = 0 (丢线)
 */
static void xunji_calc_error(void)
{
    int32_t weighted_sum = 0;
    int32_t active_count = 0;
    uint8_t i;

    for (i = 0; i < XUNJI_SENSOR_COUNT; i++) {
        if (sensor_raw[i]) {
            weighted_sum += xunji_weight[i];
            active_count++;
        }
    }

    if (active_count > 0) {
        position_raw   = weighted_sum / active_count;
        error_raw      = position_raw - XUNJI_CENTER_OFFSET;
        sensor_online  = 1;
    } else {
        position_raw   = 0;
        error_raw      = 0;
        sensor_online  = 0;
    }
}

/**
 * @brief  循迹主更新: 传感器采集 → 加权偏差 → PID → 差速输出
 */
void xunji_update(void)
{
    float error_norm;
    float pid_out;
    int32_t diff;
    int32_t abs_error;
    uint32_t adaptive_speed;

    /* ---- 第1步: 解析 UART1 数据, 更新传感器数组 ---- */
    xunji_parse_uart();

    /* ---- 第2步: 加权误差计算 ---- */
    xunji_calc_error();

    /* ---- 第3步: PID 控制 (误差归一化到 [-1, 1]) ---- */
    if (sensor_online) {
        /* 归一化: error_raw ∈ [-5500, +5500] → [-1.0, +1.0] */
        error_norm = (float)error_raw / 5500.0f;
        pid_out    = pid_update(&g_pid_xunji, error_norm);

        /*
         * 差速混合:
         *   pid_out > 0 → 线偏右 → 右轮减速, 左轮加速
         *   pid_out < 0 → 线偏左 → 左轮减速, 右轮加速
        */
        diff = (int32_t)pid_out;

        /*
         * Keep the configured base speed on straights and reduce it as
         * the line moves farther from the sensor center.  Each curve
         * speed is capped by base_speed, so this logic never accelerates.
         */
        abs_error = (error_raw < 0) ? -error_raw : error_raw;
        adaptive_speed = base_speed;

        if (abs_error > XUNJI_SMALL_CURVE_ERROR_MAX) {
            if (adaptive_speed > XUNJI_SHARP_CURVE_SPEED) {
                adaptive_speed = XUNJI_SHARP_CURVE_SPEED;
            }
        } else if (abs_error > XUNJI_CENTER_ERROR_MAX) {
            if (adaptive_speed > XUNJI_SMALL_CURVE_SPEED) {
                adaptive_speed = XUNJI_SMALL_CURVE_SPEED;
            }
        }

        left_duty  = (int32_t)adaptive_speed + diff;
        right_duty = (int32_t)adaptive_speed - diff;
    } else {
        /* 丢线: 保持上次输出, 或可设为直行 */
        /* left_duty / right_duty 保持不变 */
    }

    /* ---- 第4步: 限幅 0~100 ---- */
    if (left_duty  < 0)   left_duty  = 0;
    if (left_duty  > 100) left_duty  = 100;
    if (right_duty < 0)   right_duty = 0;
    if (right_duty > 100) right_duty = 100;
}

/**
 * @brief  在线修改基础速度
 */
void xunji_set_base_speed(uint32_t speed)
{
    if (speed > 100) speed = 100;
    base_speed = speed;
}

/* ---- 查询接口 ---- */

int32_t xunji_get_left_duty(void)
{
    return left_duty;
}

int32_t xunji_get_right_duty(void)
{
    return right_duty;
}

int32_t xunji_get_error(void)
{
    return error_raw;
}

int32_t xunji_get_position(void)
{
    return position_raw;
}

uint8_t xunji_is_online(void)
{
    return sensor_online;
}

void xunji_get_sensors(uint8_t *dst)
{
    uint8_t i;
    for (i = 0; i < XUNJI_SENSOR_COUNT; i++) {
        dst[i] = sensor_raw[i];
    }
}
