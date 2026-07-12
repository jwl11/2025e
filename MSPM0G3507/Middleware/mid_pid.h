#ifndef __MID_PID_H
#define __MID_PID_H

/* ================================================================
 *  PID 控制器 — 用于 FOC 位置环 / 速度环
 * ================================================================ */

typedef struct {
    float Kp;             /* Proportional gain               */
    float Ki;             /* Integral gain                   */
    float Kd;             /* Derivative gain                 */
    float integral;       /* Integral accumulator            */
    float prev_error;     /* Previous error (for derivative) */
    float integral_limit; /* Anti-windup: max |integral|     */
    float output_limit;   /* Output saturation               */
    float output;         /* Latest output value             */
} PID_Controller;

/* ---- Position-loop PID (P = 2.0, I = 0.01, D = 0.05) ---- */
extern PID_Controller g_pid_angle;

/* ---- Speed-loop PID (P = 0.5, I = 0.1, D = 0.01) ---- */
extern PID_Controller g_pid_speed;

/**
 * @brief  Initialize a PID controller with gains and limits
 */
void pid_init(PID_Controller *pid,
              float Kp, float Ki, float Kd,
              float integral_limit, float output_limit);

/**
 * @brief  Initialize both angle-loop and speed-loop PIDs
 */
void pid_init_all(void);

/**
 * @brief  Run one PID iteration
 * @param  pid    Pointer to PID instance
 * @param  error  Setpoint - measurement
 * @return Control output (clamped to output_limit)
 */
float pid_update(PID_Controller *pid, float error);

/* ---- Convenience wrappers for FOC integration ---- */

static inline float angle_control(float error) {
    return pid_update(&g_pid_angle, error);
}

static inline float speed_control(float error) {
    return pid_update(&g_pid_speed, error);
}

#endif /* __MID_PID_H */
