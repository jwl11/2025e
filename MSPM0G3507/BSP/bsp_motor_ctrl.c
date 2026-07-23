#include "bsp_motor_ctrl.h"

/* ================================================================
 * MG310 编码电机闭环控制 — 实现
 *
 * 级联 PID:
 *   位置模式:  pos_err → PosPID → speed_cmd → SpeedPID → duty
 *   速度模式:  speed_err → SpeedPID → duty
 *
 * PWM 占空比死区: |duty| < 3% → 停止 (防微动/啸叫)
 * ================================================================ */

/* ---- 默认 PID 参数 ---- */
#define SPEED_KP              0.05f
#define SPEED_KI              0.01f
#define SPEED_KD              0.0f
#define SPEED_INTEGRAL_LIMIT  50.0f
#define SPEED_OUTPUT_LIMIT    40.0f

#define POS_KP                2.0f
#define POS_KI                0.01f
#define POS_KD                0.5f
#define POS_INTEGRAL_LIMIT    200.0f
#define POS_OUTPUT_LIMIT      500.0f

#define STALL_DUTY_THRESHOLD       25
#define STALL_SPEED_THRESHOLD      12
#define STALL_SAMPLE_LIMIT         10U
#define DIRECTION_DUTY_THRESHOLD   10
#define DIRECTION_SPEED_THRESHOLD  20
#define DIRECTION_SAMPLE_LIMIT      6U
#define MOTOR_SAFE_DUTY_LIMIT       40
#define MOTOR_START_DUTY            20
#define MOTOR_START_TARGET_MIN      50

#define DEAD_ZONE             3      /* |duty| < 3 → STOP */

/* ---- 编码器引脚映射 (SysConfig 宏) ---- */

typedef struct {
    EncoderChannel channel;
    int8_t direction_sign;
} EncPinCfg;

static const EncPinCfg g_enc_pins[] = {
    /* Motor A: E1A=PB13, E1B=PA26 */
    { ENCODER_MOTOR_A, -1 },
    /* Motor B: E2A=PB10, E2B=PB11 */
    { ENCODER_MOTOR_B, 1 },
};

static void reset_pid_state(PID_Controller *pid)
{
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->output = 0.0f;
}

static void enter_fault(MotorController *mc, MotorCtrl_Fault fault)
{
    mc->fault = fault;
    mc->mode = MOTOR_CTRL_MODE_STOP;
    mc->output_duty = 0;
    reset_pid_state(&mc->pid_speed);
    reset_pid_state(&mc->pid_position);
    mg310_motorStop(mc->motor);
}

static bool check_motor_feedback(MotorController *mc)
{
    int32_t abs_duty =
        (mc->output_duty >= 0) ? mc->output_duty : -mc->output_duty;
    int32_t abs_speed =
        (mc->current_speed >= 0) ? mc->current_speed : -mc->current_speed;
    bool wrong_direction =
        ((mc->output_duty > DIRECTION_DUTY_THRESHOLD) &&
         (mc->current_speed < -DIRECTION_SPEED_THRESHOLD)) ||
        ((mc->output_duty < -DIRECTION_DUTY_THRESHOLD) &&
         (mc->current_speed > DIRECTION_SPEED_THRESHOLD));

    if ((abs_duty >= STALL_DUTY_THRESHOLD) &&
        (abs_speed <= STALL_SPEED_THRESHOLD)) {
        if (mc->stall_sample_count < STALL_SAMPLE_LIMIT) {
            mc->stall_sample_count++;
        }
    } else {
        mc->stall_sample_count = 0U;
    }

    if (wrong_direction) {
        if (mc->direction_error_count < DIRECTION_SAMPLE_LIMIT) {
            mc->direction_error_count++;
        }
    } else {
        mc->direction_error_count = 0U;
    }

    if (mc->direction_error_count >= DIRECTION_SAMPLE_LIMIT) {
        enter_fault(mc, MOTOR_CTRL_FAULT_FEEDBACK_DIRECTION);
        return true;
    }

    if (mc->stall_sample_count >= STALL_SAMPLE_LIMIT) {
        enter_fault(mc, MOTOR_CTRL_FAULT_STALL);
        return true;
    }

    return false;
}

/* ---- 将占空比写入电机硬件 ---- */
static void apply_duty(MotorController *mc, int32_t duty)
{
    uint32_t abs_d;

    if (duty > DEAD_ZONE) {
        abs_d = (uint32_t)duty;
        if (abs_d > MOTOR_SAFE_DUTY_LIMIT) {
            abs_d = MOTOR_SAFE_DUTY_LIMIT;
        }
        mg310_motorForward(mc->motor, abs_d);
    } else if (duty < -DEAD_ZONE) {
        abs_d = (uint32_t)(-duty);
        if (abs_d > MOTOR_SAFE_DUTY_LIMIT) {
            abs_d = MOTOR_SAFE_DUTY_LIMIT;
        }
        mg310_motorReverse(mc->motor, abs_d);
    } else {
        mg310_motorStop(mc->motor);
    }
}

/* ================================================================
 *  API
 * ================================================================ */

void motor_ctrl_init(MotorController *mc, MG310_Motor motor)
{
    const EncPinCfg *p = &g_enc_pins[motor];

    mc->motor = motor;
    mc->mode  = MOTOR_CTRL_MODE_STOP;

    encoder_init(&mc->encoder, p->channel, p->direction_sign);

    pid_init(&mc->pid_speed,
             SPEED_KP, SPEED_KI, SPEED_KD,
             SPEED_INTEGRAL_LIMIT, SPEED_OUTPUT_LIMIT);

    pid_init(&mc->pid_position,
             POS_KP, POS_KI, POS_KD,
             POS_INTEGRAL_LIMIT, POS_OUTPUT_LIMIT);

    mc->target_speed    = 0;
    mc->target_position = 0;
    mc->output_duty     = 0;
    mc->current_speed   = 0;
    mc->current_position = 0;
    mc->last_speed_sample_sequence =
        encoder_get_speed_sample_sequence(&mc->encoder);
    mc->stall_sample_count = 0U;
    mc->direction_error_count = 0U;
    mc->fault = MOTOR_CTRL_FAULT_NONE;

    mg310_motorInit(motor);
}

void motor_ctrl_set_speed_pid(MotorController *mc,
                              float Kp, float Ki, float Kd,
                              float ilim, float olim)
{
    pid_init(&mc->pid_speed, Kp, Ki, Kd, ilim, olim);
}

void motor_ctrl_set_position_pid(MotorController *mc,
                                 float Kp, float Ki, float Kd,
                                 float ilim, float olim)
{
    pid_init(&mc->pid_position, Kp, Ki, Kd, ilim, olim);
}

void motor_ctrl_set_speed(MotorController *mc, int32_t speed_mm_s)
{
    if (mc->fault != MOTOR_CTRL_FAULT_NONE) {
        return;
    }

    if (mc->mode != MOTOR_CTRL_MODE_SPEED) {
        reset_pid_state(&mc->pid_speed);
    }
    mc->mode         = MOTOR_CTRL_MODE_SPEED;
    mc->target_speed = speed_mm_s;
}

void motor_ctrl_set_position(MotorController *mc, int32_t pos_pulses)
{
    if (mc->fault != MOTOR_CTRL_FAULT_NONE) {
        return;
    }

    if (mc->mode != MOTOR_CTRL_MODE_POSITION) {
        reset_pid_state(&mc->pid_position);
        reset_pid_state(&mc->pid_speed);
    }
    mc->mode            = MOTOR_CTRL_MODE_POSITION;
    mc->target_position = pos_pulses;
}

void motor_ctrl_stop(MotorController *mc)
{
    mc->mode = MOTOR_CTRL_MODE_STOP;
    reset_pid_state(&mc->pid_speed);
    reset_pid_state(&mc->pid_position);
    mc->stall_sample_count = 0U;
    mc->direction_error_count = 0U;
    mc->output_duty = 0;
    mg310_motorStop(mc->motor);
}

void motor_ctrl_clear_fault(MotorController *mc)
{
    motor_ctrl_stop(mc);
    mc->fault = MOTOR_CTRL_FAULT_NONE;
}

void motor_ctrl_update(MotorController *mc)
{
    float speed_err, pos_err, speed_cmd, duty_f;
    int32_t duty;
    uint32_t sample_sequence;

    mc->current_position = encoder_get_count(&mc->encoder);
    sample_sequence =
        encoder_get_speed_sample_sequence(&mc->encoder);

    /*
     * Execute once per new 50 ms speed sample. This prevents the controller
     * from applying PWM before the first valid encoder measurement.
     */
    if (sample_sequence == mc->last_speed_sample_sequence) {
        return;
    }
    mc->last_speed_sample_sequence = sample_sequence;
    mc->current_speed = encoder_get_speed(&mc->encoder);

    if (mc->fault != MOTOR_CTRL_FAULT_NONE) {
        return;
    }

    if ((mc->mode != MOTOR_CTRL_MODE_STOP) &&
        check_motor_feedback(mc)) {
        return;
    }

    /* ② PID 控制 */
    switch (mc->mode) {

    case MOTOR_CTRL_MODE_SPEED:
        speed_err = (float)(mc->target_speed - mc->current_speed);
        duty_f    = pid_update_incremental_pi(&mc->pid_speed, speed_err);
        duty      = (int32_t)duty_f;
        break;

    case MOTOR_CTRL_MODE_POSITION:
        pos_err   = (float)(mc->target_position - mc->current_position);
        speed_cmd = pid_update(&mc->pid_position, pos_err);
        speed_err = speed_cmd - (float)mc->current_speed;
        duty_f    = pid_update_incremental_pi(&mc->pid_speed, speed_err);
        duty      = (int32_t)duty_f;
        break;

    default:
        duty = 0;
        break;
    }

    /* ③ 输出 */
    if ((mc->mode == MOTOR_CTRL_MODE_SPEED) &&
        (((mc->target_speed >= 0) ? mc->target_speed : -mc->target_speed) >=
         MOTOR_START_TARGET_MIN) &&
        (((mc->current_speed >= 0) ? mc->current_speed : -mc->current_speed) <=
         STALL_SPEED_THRESHOLD)) {
        if ((duty > DEAD_ZONE) && (duty < MOTOR_START_DUTY)) {
            duty = MOTOR_START_DUTY;
        } else if ((duty < -DEAD_ZONE) && (duty > -MOTOR_START_DUTY)) {
            duty = -MOTOR_START_DUTY;
        }
    }

    if (duty > MOTOR_SAFE_DUTY_LIMIT) {
        duty = MOTOR_SAFE_DUTY_LIMIT;
    } else if (duty < -MOTOR_SAFE_DUTY_LIMIT) {
        duty = -MOTOR_SAFE_DUTY_LIMIT;
    }
    mc->output_duty = duty;
    apply_duty(mc, duty);
}

int32_t motor_ctrl_get_speed(const MotorController *mc)    { return mc->current_speed; }
int32_t motor_ctrl_get_position(const MotorController *mc) { return mc->current_position; }
int32_t motor_ctrl_get_duty(const MotorController *mc)     { return mc->output_duty; }
MotorCtrl_Fault motor_ctrl_get_fault(const MotorController *mc) { return mc->fault; }
