#ifndef __BSP_MOTOR_CTRL_H
#define __BSP_MOTOR_CTRL_H

#include "ti_msp_dl_config.h"
#include "drv_encoder.h"
#include "bsp_encoder.h"
#include "mid_pid.h"

/* ================================================================
 * MG310 编码电机闭环控制 (BSP 层)
 *
 * 级联 PID 结构:
 *                     ┌──────────┐      ┌──────────┐
 *  target_pos ──►○──►│ Pos PID  ├──►○──►│Speed PID ├──► PWM ──► Motor
 *                ▲   └──────────┘  ▲   └──────────┘     │
 *                │                 │                     │
 *             encoder.count   encoder.speed         encoder
 *
 * 三种模式:
 *   STOP      — 电机停止
 *   SPEED     — 仅速度环 (target_speed → SpeedPID → duty)
 *   POSITION  — 级联 (target_pos → PosPID → SpeedPID → duty)
 *
 * 分层:
 *   Application ──► bsp_motor_ctrl ──► drv_encoder   (编码器)
 *                                    ├─ bsp_encoder   (PWM)
 *                                    └─ mid_pid       (PID)
 * ================================================================ */

typedef enum {
    MOTOR_CTRL_MODE_STOP     = 0,
    MOTOR_CTRL_MODE_SPEED    = 1,
    MOTOR_CTRL_MODE_POSITION = 2,
} MotorCtrl_Mode;

typedef enum {
    MOTOR_CTRL_FAULT_NONE = 0,
    MOTOR_CTRL_FAULT_STALL = 1,
    MOTOR_CTRL_FAULT_FEEDBACK_DIRECTION = 2,
} MotorCtrl_Fault;

typedef struct {
    Encoder      encoder;         /* 编码器句柄                        */
    MG310_Motor  motor;           /* 电机编号 (A / B)                  */
    MotorCtrl_Mode mode;          /* 当前控制模式                      */

    PID_Controller pid_speed;     /* 速度环 PID                        */
    PID_Controller pid_position;  /* 位置环 PID                        */

    int32_t  target_speed;        /* 目标速度 (脉冲/秒)                */
    int32_t  target_position;     /* 目标位置 (脉冲)                   */

    /* 只读反馈值 */
    int32_t  output_duty;         /* PWM 占空比 (-100 ~ +100)          */
    int32_t  current_speed;       /* 实测速度 (脉冲/秒)                */
    int32_t  current_position;    /* 实测位置 (脉冲)                   */
    uint32_t last_speed_sample_sequence;
    uint8_t stall_sample_count;
    uint8_t direction_error_count;
    MotorCtrl_Fault fault;
} MotorController;

/* ---- API ---- */

/** @brief  初始化控制器 (编码器 + PID + 电机硬件) */
void motor_ctrl_init(MotorController *mc, MG310_Motor motor);

/** @brief  设置速度环 PID 参数 */
void motor_ctrl_set_speed_pid(MotorController *mc,
                              float Kp, float Ki, float Kd,
                              float integral_limit, float output_limit);

/** @brief  设置位置环 PID 参数 */
void motor_ctrl_set_position_pid(MotorController *mc,
                                 float Kp, float Ki, float Kd,
                                 float integral_limit, float output_limit);

/** @brief  启动速度闭环 */
/** @param speed_mm_s Target wheel linear speed in mm/s. */
void motor_ctrl_set_speed(MotorController *mc, int32_t speed_mm_s);

/** @brief  启动位置闭环 (级联) */
void motor_ctrl_set_position(MotorController *mc, int32_t pos_pulses);

/** @brief  停止电机 (惯性滑行, 积分清零) */
void motor_ctrl_stop(MotorController *mc);
void motor_ctrl_clear_fault(MotorController *mc);

/** @brief  主控制更新；应跟随编码器测速周期调用（当前为 50 ms） */
void motor_ctrl_update(MotorController *mc);

/* 查询接口 */
int32_t motor_ctrl_get_speed(const MotorController *mc);
int32_t motor_ctrl_get_position(const MotorController *mc);
int32_t motor_ctrl_get_duty(const MotorController *mc);
MotorCtrl_Fault motor_ctrl_get_fault(const MotorController *mc);

#endif /* __BSP_MOTOR_CTRL_H */
