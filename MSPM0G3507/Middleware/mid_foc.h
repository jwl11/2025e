#ifndef __MID_FOC_H
#define __MID_FOC_H

#include "ti_msp_dl_config.h"
#include <math.h>

/* ================================================================
 *  Motor parameters
 * ================================================================ */
extern int  PP;       /* Pole pairs (e.g. 7)               */
extern int  DIR;      /* Rotation direction: +1 or -1      */

/* ================================================================
 *  Global variables
 * ================================================================ */
extern float voltage_limit;          /* Output voltage clamp (V)       */
extern float voltage_power_supply;   /* DC bus voltage (V)             */
extern float zero_electric_angle;    /* Calibrated zero-electric offset */
extern float Ua, Ub, Uc;             /* Three-phase voltages            */
extern float dc_a, dc_b, dc_c;       /* Three-phase duty cycles (0~1)  */

/* ================================================================
 *  FOC API
 * ================================================================ */

/**
 * @brief  Initialize FOC: PWM + PID
 * @param  pwr  DC bus voltage (e.g. 12.6f)
 */
void foc_init(float pwr);

/**
 * @brief  Encoder init + automatic zero-angle calibration
 *
 *         Applies a 3V alignment torque for 3 seconds to lock the
 *         rotor to a known electrical angle (3*PI/2), then records
 *         the offset as zero_electric_angle.
 *
 *         Prerequisites: I2C already initialized (SYSCFG_DL_init).
 *
 * @param  _PP   Pole pairs
 * @param  _DIR  Rotation direction (+1 or -1)
 */
void foc_as5600_init(int _PP, int _DIR);

/**
 * @brief  Torque control (FOC core): inverse-Park + Clarke → 3-phase PWM
 * @param  Uq       Quadrature voltage (torque component)
 * @param  angle_el Electrical angle (rad)
 */
void foc_set_torque(float Uq, float angle_el);

/**
 * @brief  Position-loop control (calls angle PID → torque)
 * @param  target_angle  Target mechanical angle (rad, multiturn)
 */
void foc_set_angle(float target_angle);

/**
 * @brief  Speed-loop control (calls speed PID → torque)
 * @param  target_speed  Target angular velocity (rad/s)
 */
void foc_set_speed(float target_speed);

/**
 * @brief  Mechanical → electrical angle (with zero-offset correction)
 * @return Electrical angle in [0, 2*PI)
 */
float foc_electrical_angle(void);

/**
 * @brief  Normalize angle to [0, 2*PI)
 */
float foc_normalize(float angle);

/**
 * @brief  PWM output: 3-phase voltages → duty cycles → hardware
 */
void foc_pwm_output(float Ua, float Ub, float Uc);

/**
 * @brief  Open-loop spin: rotate voltage vector at fixed speed, no feedback.
 *
 *         Diagnostic tool — if the motor spins smoothly here but shakes in
 *         closed-loop, the problem is sensor/electric-angle.  If it shakes
 *         here too, the problem is wiring / phase order / PWM hardware.
 *
 * @param  voltage       Voltage magnitude (start with 1.0f ~ 2.0f)
 * @param  speed_rad_s   Electrical rotation speed (rad/s, e.g. 10.0f)
 * @param  dt_s          Time step since last call (seconds)
 */
void foc_openloop_spin(float voltage, float speed_rad_s, float dt_s);

/**
 * @brief  Stop all PWM output (all phases = 50% = 0 V differential)
 */
void foc_stop(void);

#endif /* __MID_FOC_H */
