#include "mid_foc.h"
#include "mid_math_constants.h"
#include "drv_tim.h"
#include "mid_pid.h"
#include "bsp_AS5600.h"
#include "mid_delay.h"

/* ================================================================
 *  Motor parameters
 * ================================================================ */
int   PP  = 7;      /* Pole pairs (typical small BLDC)   */
int   DIR = 1;      /* Direction: +1 = forward, -1 = reverse */

/* ================================================================
 *  Global variables
 * ================================================================ */
float voltage_limit        = 6.0f;   /* Output clamp                 */
float voltage_power_supply = 12.6f;   /* DC bus voltage               */
float zero_electric_angle  = 0.0f;    /* Filled by calibration        */
float Ua = 0, Ub = 0, Uc = 0;
float dc_a = 0, dc_b = 0, dc_c = 0;

/* Targets & sensor readings for position/speed loops */
float sensor_angle = 0.0f;
float angle_target = 30.0f;           /* default: 30 degrees          */
float sensor_speed = 0.0f;
float speed_target = 40.0f;           /* default: 40 rad/s            */

/* ================================================================
 *  Utility
 * ================================================================ */

float foc_normalize(float angle)
{
    float a = fmodf(angle, _2PI);
    return (a >= 0.0f) ? a : (a + _2PI);
}

/* ================================================================
 *  Electrical angle: mechanical → electrical, zero-offset corrected
 * ================================================================ */

float foc_electrical_angle(void)
{
    return foc_normalize((float)(DIR * PP)
                         * as5600_get_angle_radians()
                         - zero_electric_angle);
}

/* ================================================================
 *  PWM output: 3-phase voltage → duty cycle → hardware
 * ================================================================ */

void foc_pwm_output(float Ua, float Ub, float Uc)
{
    /* Clamp to [0, voltage_limit] */
    Ua = _constrain(Ua, 0.0f, voltage_limit);
    Ub = _constrain(Ub, 0.0f, voltage_limit);
    Uc = _constrain(Uc, 0.0f, voltage_limit);

    /* Duty cycle = voltage / Vbus, clamped to [0, 1] */
    dc_a = _constrain(Ua / voltage_power_supply, 0.0f, 1.0f);
    dc_b = _constrain(Ub / voltage_power_supply, 0.0f, 1.0f);
    dc_c = _constrain(Uc / voltage_power_supply, 0.0f, 1.0f);

    /* pwm_setAllDuty takes 0-100 (percent) */
    pwm_setAllDuty((uint32_t)(dc_a * 100.0f),
                   (uint32_t)(dc_b * 100.0f),
                   (uint32_t)(dc_c * 100.0f));
}

/* ================================================================
 *  Torque control: inverse-Park + Clarke → 3-phase
 * ================================================================ */

void foc_set_torque(float Uq, float angle_el)
{
    float Ualpha, Ubeta;

    Uq = _constrain(Uq, -voltage_power_supply / 2.0f,
                         voltage_power_supply / 2.0f);
    angle_el = foc_normalize(angle_el);

    /* Inverse Park */
    Ualpha = -Uq * sinf(angle_el);
    Ubeta  =  Uq * cosf(angle_el);

    /* Clarke → 3-phase, biased to Vbus/2 for all-positive output */
    Ua = Ualpha + voltage_power_supply / 2.0f;
    Ub = (_SQRT3 * Ubeta - Ualpha) / 2.0f + voltage_power_supply / 2.0f;
    Uc = (-Ualpha - _SQRT3 * Ubeta) / 2.0f + voltage_power_supply / 2.0f;

    foc_pwm_output(Ua, Ub, Uc);
}

/* ================================================================
 *  Init: PWM + PID
 * ================================================================ */

void foc_init(float pwr)
{
    voltage_power_supply = pwr;
    pwm_Init();
    pid_init_all();
}

/* ================================================================
 *  AS5600 init + auto zero-angle calibration
 *
 *  I2C is assumed already initialized by SYSCFG_DL_init().
 *  SysTick timing is maintained by mid_delay (delay_ms keeps it on).
 * ================================================================ */

void foc_as5600_init(int _PP, int _DIR)
{
    PP  = _PP;
    DIR = _DIR;

    as5600_set_direction(DIR);  /* sensor velocity now includes DIR */

    /* Apply 3V alignment torque to lock rotor at known position */
    foc_set_torque(3.0f, _3PI_2);
    delay_ms(3000);

    /* Record zero-offset and release torque */
    zero_electric_angle = foc_electrical_angle();
    foc_set_torque(0.0f, _3PI_2);

    sensor_speed = as5600_get_speed();
}

/* ================================================================
 *  Position loop
 * ================================================================ */

void foc_set_angle(float target)
{
    angle_target = target;
    sensor_angle = as5600_get_angle_radians();  /* single-turn, matched to original */

    /* Error in degrees → PID → torque */
    float error = (angle_target - (float)DIR * sensor_angle)
                * 180.0f / PI;
    float angle_out = angle_control(error);

    foc_set_torque(angle_out, foc_electrical_angle());
}

/* ================================================================
 *  Speed loop
 * ================================================================ */

void foc_set_speed(float target)
{
    speed_target = target;
    sensor_speed = as5600_get_speed();

    float error = speed_target - sensor_speed;  /* DIR already in as5600_get_speed */
    float speed_out = speed_control(error);

    foc_set_torque(speed_out, foc_electrical_angle());
}

/* ================================================================
 *  Open-loop spin: no sensor feedback, just rotate voltage vector
 * ================================================================ */

void foc_openloop_spin(float voltage, float speed_rad_s, float dt_s)
{
    /* Start at π/2 (90°) ahead of calibration zero → max starting torque */
    static float elec_angle = _PI_2;

    elec_angle += speed_rad_s * dt_s;
    elec_angle  = foc_normalize(elec_angle);

    foc_set_torque(voltage, elec_angle);
}

/* ================================================================
 *  Stop: disable PWM timer → all outputs Hi-Z → motor off
 * ================================================================ */

void foc_stop(void)
{
    pwm_stop();   /* 关定时器，彻底断电，不发热 */
}
