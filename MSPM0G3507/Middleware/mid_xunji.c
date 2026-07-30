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
 *   官方帧格式为 "#xxxxxxxxxxxx!"，x 为 ASCII '0' 或 '1'。
 *   A1~A12 按模块上从左到右排列；UART 0=黑线，1=白底。
 *   内部统一转换为 1=黑线有效，0=白底。
 *
 * 差速公式:
 *   left  = base_speed - pid_output
 *   right = base_speed + pid_output
 *   两端均 clamp 到 [0, 100]
 * ================================================================ */

/* 循迹专用 PID 实例 */
PID_Controller g_pid_xunji = {0};

/* Tracking thresholds and minimum usable steering for the heavy chassis. */
#define XUNJI_CENTER_ERROR_MAX       500
#define XUNJI_SMALL_CURVE_ERROR_MAX  1500
#define XUNJI_MIN_DIFF_SMALL          4
#define XUNJI_MIN_DIFF_MEDIUM         6
#define XUNJI_MIN_DIFF_LARGE         10
#define XUNJI_CORNER_LEFT_DUTY       30
#define XUNJI_CORNER_SEARCH_DUTY     18
#define XUNJI_CORNER_CENTER_MAX      1000
#define XUNJI_CORNER_ACTIVE_MIN      8U   /* 8 路以上才算直角弯 (圆弧不会触发) */
#define XUNJI_CORNER_MARK_CYCLES     3U   /* 连续确认 15ms */
#define XUNJI_CORNER_MIN_TURN_CYCLES 12U  /* 至少左转 60ms 后才允许退出 */
#define XUNJI_LOST_CONFIRM_CYCLES    3U
#define XUNJI_CORNER_CONFIRM_CYCLES  4U
#define XUNJI_CORNER_TIMEOUT_CYCLES  300U

/*
 * Tracking center offset in sensor-weight units (1000 per channel).
 * A positive value moves the target toward the higher sensor index.
 */
#define XUNJI_CENTER_OFFSET          0

/* ---- 内部状态 ---- */
static uint8_t  sensor_raw[XUNJI_SENSOR_COUNT];   /* PID 使用的完整帧快照 */
static uint8_t  sensor_pending[XUNJI_SENSOR_COUNT]; /* UART 正在接收的临时帧 */
static uint32_t base_speed;                        /* 基础速度 0~100 */
static int32_t  left_duty;                         /* 左轮占空比 */
static int32_t  right_duty;                        /* 右轮占空比 */
static int32_t  error_raw;                         /* 加权偏差 (-5500~+5500) */
static int32_t  position_raw;                      /* 加权位置 (0~11000) */
static uint8_t  sensor_online;                     /* 是否有线 */
static uint8_t  active_sensor_count;               /* 当前帧有效传感器数量 */
static uint8_t  frame_index;                       /* 临时帧内有效数据数 */
static uint8_t  frame_receiving;                   /* 已收到官方帧头 '#' */

typedef enum {
    XUNJI_TRACK_FOLLOW = 0,
    XUNJI_TRACK_CORNER_LEFT,
} XunjiTrackState;

static XunjiTrackState tracking_state;
static uint8_t  has_seen_line;
static uint8_t  lost_line_count;
static uint8_t  corner_mark_count;
static uint8_t  corner_center_count;
static uint16_t corner_turn_cycles;
static uint8_t  corner_event;

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
             20.0f,       /* integral_limit: Ki=0.3时最多贡献6差速，防饱和 */
             50.0f);      /* output_limit: 最大差速调整量 */

    /* 状态初始化 */
    base_speed   = speed;
    left_duty    = 0;
    right_duty   = 0;
    error_raw    = 0;
    position_raw = 0;
    sensor_online = 0;
    frame_index  = 0;
    frame_receiving = 0;
    xunji_reset_tracking();

    /* 清空传感器数组 */
    uint8_t i;
    for (i = 0; i < XUNJI_SENSOR_COUNT; i++) {
        sensor_raw[i] = 0;
        sensor_pending[i] = 0;
    }

    /* 清空 UART1 缓冲区 */
    drv_uart1_flush();
}

/**
 * @brief  从 UART1 环形缓冲区解析传感器数据
 *
 *         严格按照官方 "# + 12位0/1 + !" 协议解析。
 *         只有收到完整帧头、恰好 12 路数据和帧尾才提交。
 *         PID 始终使用上一个完整帧，不会读到新旧混合的半帧。
 */
static void xunji_parse_uart(void)
{
    int16_t byte;
    uint8_t i;

    while (drv_uart1_available() > 0) {
        byte = drv_uart1_read();
        if (byte < 0) break;

        /* 帧头始终重新同步，即使上一帧因丢字节尚未结束。 */
        if (byte == '#') {
            frame_receiving = 1U;
            frame_index = 0U;
            continue;
        }

        if (frame_receiving == 0U) {
            continue;
        }

        if (byte == '!') {
            if (frame_index == XUNJI_SENSOR_COUNT) {
                for (i = 0U; i < XUNJI_SENSOR_COUNT; i++) {
                    sensor_raw[i] = sensor_pending[i];
                }
            }
            frame_receiving = 0U;
            frame_index = 0U;
            continue;
        }

        if ((byte == '0') || (byte == '1')) {
            if (frame_index < XUNJI_SENSOR_COUNT) {
                /*
                 * 模块协议：0=黑线、1=白底。
                 * 算法内部：1=黑线有效、0=白底。
                 */
                sensor_pending[frame_index++] =
                    (byte == '0') ? 1U : 0U;
            } else {
                /* 超过 12 路仍未遇到帧尾，本帧作废并等待下个 '#'. */
                frame_receiving = 0U;
                frame_index = 0U;
            }
        } else {
            /* 官方帧内部不允许出现其他字符。 */
            frame_receiving = 0U;
            frame_index = 0U;
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
        active_sensor_count = (uint8_t)active_count;
        position_raw   = weighted_sum / active_count;
        error_raw      = position_raw - XUNJI_CENTER_OFFSET;
        sensor_online  = 1;
    } else {
        active_sensor_count = 0U;
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

    /* ---- 第1步: 解析 UART1 数据, 更新传感器数组 ---- */
    xunji_parse_uart();

    /* ---- 第2步: 加权误差计算 ---- */
    xunji_calc_error();

    abs_error = (error_raw < 0) ? -error_raw : error_raw;

    /* 固定逆时针路线：丢线后主动左转，重新稳定捕获中线后恢复 PID。 */
    if (tracking_state == XUNJI_TRACK_CORNER_LEFT) {
        if (corner_turn_cycles < XUNJI_CORNER_TIMEOUT_CYCLES) {
            corner_turn_cycles++;
            left_duty  = 0;
            right_duty = XUNJI_CORNER_LEFT_DUTY;
        } else {
            /* 超时后低速继续搜索，避免停车后再也无法改变传感器位置。 */
            left_duty  = 0;
            right_duty = XUNJI_CORNER_SEARCH_DUTY;
        }

        if ((corner_turn_cycles >= XUNJI_CORNER_MIN_TURN_CYCLES) &&
            (active_sensor_count < XUNJI_CORNER_ACTIVE_MIN) &&
            sensor_online &&
            (abs_error <= XUNJI_CORNER_CENTER_MAX)) {
            corner_center_count++;
            if (corner_center_count >= XUNJI_CORNER_CONFIRM_CYCLES) {
                tracking_state = XUNJI_TRACK_FOLLOW;
                corner_center_count = 0U;
                corner_turn_cycles = 0U;
                left_duty  = (int32_t)base_speed;
                right_duty = (int32_t)base_speed;
            }
        } else {
            corner_center_count = 0U;
        }
    } else if (!sensor_online) {
        if (has_seen_line != 0U) {
            if (lost_line_count < XUNJI_LOST_CONFIRM_CYCLES) {
                lost_line_count++;
            }

            if (lost_line_count >= XUNJI_LOST_CONFIRM_CYCLES) {
                tracking_state = XUNJI_TRACK_CORNER_LEFT;
                lost_line_count = 0U;
                corner_center_count = 0U;
                corner_turn_cycles = 1U;
                left_duty  = 0;
                right_duty = XUNJI_CORNER_LEFT_DUTY;
            }
        } else {
            left_duty  = 0;
            right_duty = 0;
        }
    } else {
        has_seen_line = 1U;
        lost_line_count = 0U;

        /*
         * 椭圆赛道没有直角。宽黑线是唯一启停线，由应用层负责识别，
         * 此处不能再触发旧的强制左转逻辑。
         */
        corner_mark_count = 0U;

        /* ---- 第3步: PID 控制 (误差归一化到 [-1, 1]) ---- */
        /* 归一化: error_raw ∈ [-5500, +5500] → [-1.0, +1.0] */
        error_norm = (float)error_raw / 5500.0f;
        pid_out    = pid_update(&g_pid_xunji, error_norm);

        /*
         * 差速混合:
         *   A1 在左、A12 在右，电机 A 为左轮、B 为右轮：
         *   pid_out > 0 → 线偏右 → 左轮加速, 右轮减速
         *   pid_out < 0 → 线偏左 → 右轮加速, 左轮减速
        */
        /* Round instead of truncating small PID outputs toward zero. */
        diff = (pid_out >= 0.0f) ?
               (int32_t)(pid_out + 0.5f) :
               (int32_t)(pid_out - 0.5f);

        /*
         * A heavy car has a sizeable motor/static-friction dead zone.
         * Preserve the PID direction, but guarantee a usable differential
         * once the line has moved away from the sensor center.
         */
        if (abs_error > XUNJI_SMALL_CURVE_ERROR_MAX) {
            if ((diff > -XUNJI_MIN_DIFF_LARGE) &&
                (diff < XUNJI_MIN_DIFF_LARGE)) {
                diff = (error_raw > 0) ?
                       XUNJI_MIN_DIFF_LARGE : -XUNJI_MIN_DIFF_LARGE;
            }
        } else if (abs_error > XUNJI_CENTER_ERROR_MAX) {
            if ((diff > -XUNJI_MIN_DIFF_MEDIUM) &&
                (diff < XUNJI_MIN_DIFF_MEDIUM)) {
                diff = (error_raw > 0) ?
                       XUNJI_MIN_DIFF_MEDIUM : -XUNJI_MIN_DIFF_MEDIUM;
            }
        } else if (abs_error > 0) {
            if ((diff > -XUNJI_MIN_DIFF_SMALL) &&
                (diff < XUNJI_MIN_DIFF_SMALL)) {
                diff = (error_raw > 0) ?
                       XUNJI_MIN_DIFF_SMALL : -XUNJI_MIN_DIFF_SMALL;
            }
        }

        left_duty  = (int32_t)base_speed - diff;
        right_duty = (int32_t)base_speed + diff;
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

void xunji_reset_tracking(void)
{
    tracking_state = XUNJI_TRACK_FOLLOW;
    has_seen_line = 0U;
    lost_line_count = 0U;
    corner_mark_count = 0U;
    corner_center_count = 0U;
    corner_turn_cycles = 0U;
    corner_event = 0U;
    left_duty = 0;
    right_duty = 0;

    /*
     * 清除启停线或重新捕线期间留下的 PID 历史状态。
     * 用当前误差初始化 prev_error，可避免恢复 PID 时产生微分冲击。
     */
    g_pid_xunji.integral = 0.0f;
    g_pid_xunji.prev_error = (float)error_raw / 5500.0f;
    g_pid_xunji.output = 0.0f;
}

uint8_t xunji_take_corner_event(void)
{
    uint8_t event = corner_event;
    corner_event = 0U;
    return event;
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
