#include "bsp_encoder.h"
/* ================================================================
 * MG310 双路霍尔编码电机驱动 (TIMG0)
 *
 * 硬件连接:
 *   ┌─────────┬──────────────┬──────────────┬──────────────┐
 *   │  电机   │   PWM (TIMG0)│    IN1       │    IN2       │
 *   ├─────────┼──────────────┼──────────────┼──────────────┤
 *   │   A     │ CH0, PA12    │ AIN1, PB17   │ AIN2, PB19   │
 *   │   B     │ CH1, PA13    │ BIN1, PA16   │ BIN2, PB24   │
 *   └─────────┴──────────────┴──────────────┴──────────────┘
 *
 * PWM 模式: 边沿对齐向上计数, 周期 = 3200, 时钟 = 32MHz
 *   占空比 = CC值 / 3200
 *   duty=0   → CC=0    → 持续低电平 (0%)
 *   duty=100 → CC=3200 → 持续高电平 (100%)
 *   duty=50  → CC=1600 → 50% 方波
 *
 * 方向控制 (IN1 IN2):
 *   00 → 停止 (两路输出低电平)
 *   01 → 反转
 *   10 → 正转
 *   11 → 刹车 (两路输出高电平 → H桥下管导通制动)
 * ================================================================ */

/*
 * 电机引脚配置表
 * 每个电机对应一组 {PWM通道, IN1端口/引脚, IN2端口/引脚}
 */
typedef struct {
    DL_TIMER_CC_INDEX   pwmCh;      /* TIMA1 CC 通道索引 */
    GPIO_Regs          *in1Port;    /* IN1 GPIO 端口 */
    uint32_t            in1Pin;     /* IN1 GPIO 引脚掩码 */
    GPIO_Regs          *in2Port;    /* IN2 GPIO 端口 */
    uint32_t            in2Pin;     /* IN2 GPIO 引脚掩码 */
} MG310_MotorConfig;

static const MG310_MotorConfig gMotorCfg[] = {
    /* MG310_MOTOR_A */
    {
        .pwmCh   = DL_TIMER_CC_0_INDEX,
        .in1Port = MG310_AIN1_PORT,
        .in1Pin  = MG310_AIN1_PIN,
        .in2Port = MG310_AIN2_PORT,
        .in2Pin  = MG310_AIN2_PIN,
    },
    /* MG310_MOTOR_B */
    {
        .pwmCh   = DL_TIMER_CC_1_INDEX,
        .in1Port = MG310_BIN1_PORT,
        .in1Pin  = MG310_BIN1_PIN,
        .in2Port = MG310_BIN2_PORT,
        .in2Pin  = MG310_BIN2_PIN,
    },
};

/**
 * @brief  设置指定电机的IN1/IN2 GPIO电平
 * @param  motor 电机选择 (A/B)
 * @param  dir   方向枚举值
 */
static void mg310_setDirectionPins(MG310_Motor motor, MG310_Direction dir)
{
    const MG310_MotorConfig *cfg = &gMotorCfg[motor];

    switch (dir) {
    case MG310_DIR_STOP:
        DL_GPIO_clearPins(cfg->in1Port, cfg->in1Pin);
        DL_GPIO_clearPins(cfg->in2Port, cfg->in2Pin);
        break;
    case MG310_DIR_REVERSE:
        DL_GPIO_clearPins(cfg->in1Port, cfg->in1Pin);
        DL_GPIO_setPins(cfg->in2Port, cfg->in2Pin);
        break;
    case MG310_DIR_FORWARD:
        DL_GPIO_setPins(cfg->in1Port, cfg->in1Pin);
        DL_GPIO_clearPins(cfg->in2Port, cfg->in2Pin);
        break;
    case MG310_DIR_BRAKE:
        DL_GPIO_setPins(cfg->in1Port, cfg->in1Pin);
        DL_GPIO_setPins(cfg->in2Port, cfg->in2Pin);
        break;
    default:
        break;
    }
}

/**
 * @brief  MG310电机初始化 (单路)
 *         确保指定电机处于停止状态 (占空比=0, IN1=IN2=0)
 * @param  motor 电机选择 (MG310_MOTOR_A / MG310_MOTOR_B)
 * @note   GPIO 和 TIM1 的底层初始化已在 SYSCFG_DL_init() 中完成,
 *         此函数只需设定初始状态。调用时机: 在 SYSCFG_DL_init() 之后。
 */
void mg310_motorInit(MG310_Motor motor)
{
    const MG310_MotorConfig *cfg = &gMotorCfg[motor];

    /* 方向引脚设为停止 (IN1=0, IN2=0) */
    mg310_setDirectionPins(motor, MG310_DIR_STOP);

    /* PWM 占空比设为 0 (CC=0 → 持续低电平) */
    DL_TimerA_setCaptureCompareValue(MG310_PWM_INST, 0, cfg->pwmCh);

    /*
     * TIM1 已由 SysConfig 配置为自动启动 (DL_TIMER_START),
     * 此处再确保一次计数器在运行。
     */
    DL_Timer_startCounter(MG310_PWM_INST);
}

/**
 * @brief  设置MG310指定电机PWM占空比 (0~100)
 * @param  motor 电机选择
 * @param  duty  占空比百分比, 范围 0~100
 * @note   边沿对齐向上计数, CC值 = duty% * 周期
 *         duty=0   → 输出常低
 *         duty=100 → 输出常高
 */
void mg310_motorSetSpeed(MG310_Motor motor, uint32_t duty)
{
    const MG310_MotorConfig *cfg = &gMotorCfg[motor];
    uint32_t cmp_val;

    /* 限幅到 0~100 */
    if (duty > 100) {
        duty = 100;
    }

    /*
     * 边沿对齐向上计数模式:
     *   计数器 0→周期→0→周期...
     *   装载事件(周期匹配) → 输出 HIGH
     *   比较事件(CC匹配)    → 输出 LOW
     *   CC = duty * 周期 / 100
     */
    cmp_val = (duty * MG310_PWM_PERIOD_MAX) / 100;

    DL_TimerA_setCaptureCompareValue(MG310_PWM_INST, cmp_val, cfg->pwmCh);
}

/**
 * @brief  设置MG310指定电机方向
 * @param  motor 电机选择
 * @param  dir   方向: MG310_DIR_STOP / FORWARD / REVERSE / BRAKE
 * @note   切换方向前建议先将占空比降至0, 避免大电流冲击
 */
void mg310_motorSetDirection(MG310_Motor motor, MG310_Direction dir)
{
    mg310_setDirectionPins(motor, dir);
}

/**
 * @brief  MG310指定电机正转
 * @param  motor 电机选择
 * @param  duty  占空比 0~100
 */
void mg310_motorForward(MG310_Motor motor, uint32_t duty)
{
    mg310_setDirectionPins(motor, MG310_DIR_FORWARD);
    mg310_motorSetSpeed(motor, duty);
}

/**
 * @brief  MG310指定电机反转
 * @param  motor 电机选择
 * @param  duty  占空比 0~100
 */
void mg310_motorReverse(MG310_Motor motor, uint32_t duty)
{
    mg310_setDirectionPins(motor, MG310_DIR_REVERSE);
    mg310_motorSetSpeed(motor, duty);
}

/**
 * @brief  MG310指定电机停止 (IN1=0, IN2=0, PWM=0)
 * @param  motor 电机选择
 */
void mg310_motorStop(MG310_Motor motor)
{
    mg310_motorSetSpeed(motor, 0);
    mg310_setDirectionPins(motor, MG310_DIR_STOP);
}

/**
 * @brief  MG310指定电机制动 (IN1=1, IN2=1 → H桥下管导通制动)
 * @param  motor 电机选择
 */
void mg310_motorBrake(MG310_Motor motor)
{
    mg310_motorSetSpeed(motor, 0);
    mg310_setDirectionPins(motor, MG310_DIR_BRAKE);
}

/**
 * @brief  双电机同步初始化
 *         同时初始化电机A和B, 置停
 */
void mg310_motorInitAll(void)
{
    mg310_motorInit(MG310_MOTOR_A);
    mg310_motorInit(MG310_MOTOR_B);
}

/**
 * @brief  双电机同步停止
 */
void mg310_motorStopAll(void)
{
    mg310_motorStop(MG310_MOTOR_A);
    mg310_motorStop(MG310_MOTOR_B);
}