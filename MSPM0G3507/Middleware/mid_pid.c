#include "mid_pid.h"
#include "mid_math_constants.h"

/* ================================================================
 *  Default PID instances
 *
 *  Gains are conservative starting-points for a small 7-pole-pair
 *  BLDC motor.  Tune for your specific motor.
 * ================================================================ */

PID_Controller g_pid_angle = {0};
PID_Controller g_pid_speed = {0};

/**
 * @brief  Initialize a PID controller
 */
void pid_init(PID_Controller *pid,
              float Kp, float Ki, float Kd,
              float integral_limit, float output_limit)
{
    pid->Kp             = Kp;
    pid->Ki             = Ki;
    pid->Kd             = Kd;
    pid->integral       = 0.0f;
    pid->prev_error     = 0.0f;
    pid->integral_limit = integral_limit;
    pid->output_limit   = output_limit;
    pid->output         = 0.0f;
}

/**
 * @brief  Initialize both controllers with default gains
 */
void pid_init_all(void)
{
    /* Position loop (matched to original STM32 FOC) */
    pid_init(&g_pid_angle,
             0.043f,  /* Kp */
             0.0005f, /* Ki */
             0.22f,   /* Kd */
             2000.0f, /* integral limit (original ±2000) */
             6.0f);   /* output limit (Uq max ≈ Vbus/2) */

    /* Speed loop (matched to original STM32 FOC) */
    pid_init(&g_pid_speed,
             0.11f,   /* Kp */
             0.005f,  /* Ki */
             0.0008f, /* Kd */
             3000.0f, /* integral limit (original ±3000) */
             6.0f);   /* output limit */
}

/**
 * @brief  Run one PID iteration
 *
 *         Standard positional PID:
 *           P = Kp * error
 *           I = Ki * integral(error)   (with anti-windup)
 *           D = Kd * (error - prev_error)
 *
 *         Output is clamped to [-output_limit, +output_limit].
 */
float pid_update(PID_Controller *pid, float error)
{
    float p_term, i_term, d_term;

    /* Proportional */
    p_term = pid->Kp * error;

    /* Integral with anti-windup */
    pid->integral += error;
    pid->integral  = _constrain(pid->integral,
                                -pid->integral_limit,
                                 pid->integral_limit);
    i_term = pid->Ki * pid->integral;

    /* Derivative (on error, not measurement) */
    d_term = pid->Kd * (error - pid->prev_error);
    pid->prev_error = error;

    /* Sum and clamp */
    pid->output = p_term + i_term + d_term;
    pid->output = _constrain(pid->output,
                             -pid->output_limit,
                              pid->output_limit);

    return pid->output;
}
