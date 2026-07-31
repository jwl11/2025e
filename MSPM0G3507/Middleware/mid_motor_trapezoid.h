#ifndef MID_MOTOR_TRAPEZOID_H
#define MID_MOTOR_TRAPEZOID_H

#include <stdint.h>

/*
 * Open-loop trapezoidal PWM generator.
 *
 * The module does not read an encoder and does not use PID feedback.
 * Positive duty means forward, negative duty means reverse, and zero
 * means stop.  The caller applies the returned duty to the motor BSP.
 */
typedef enum {
    MID_MOTOR_TRAPEZOID_IDLE = 0,
    MID_MOTOR_TRAPEZOID_ACCEL,
    MID_MOTOR_TRAPEZOID_CRUISE,
    MID_MOTOR_TRAPEZOID_DECEL
} MidMotorTrapezoidPhase;

typedef struct {
    int32_t peak_duty;
    int32_t current_duty;
    uint32_t accel_ms;
    uint32_t cruise_ms;
    uint32_t decel_ms;
    uint32_t start_ms;
    MidMotorTrapezoidPhase phase;
    uint8_t running;
} MidMotorTrapezoid;

/* Initialize one independent ramp instance in the stopped state. */
void mid_motor_trapezoid_init(MidMotorTrapezoid *ramp);

/*
 * Start a new 0 -> peak -> hold -> 0 profile.
 *
 * peak_duty: signed PWM percentage, clamped to -100..100.
 * now_ms:    current monotonic millisecond tick.
 *
 * A zero-length acceleration starts at peak duty immediately.
 * A zero-length deceleration stops immediately after the cruise phase.
 */
void mid_motor_trapezoid_start(MidMotorTrapezoid *ramp,
                               int32_t peak_duty,
                               uint32_t accel_ms,
                               uint32_t cruise_ms,
                               uint32_t decel_ms,
                               uint32_t now_ms);

/*
 * Advance the profile without blocking and return signed duty -100..100.
 * Call periodically with the same monotonic millisecond timebase.
 */
int32_t mid_motor_trapezoid_update(MidMotorTrapezoid *ramp,
                                   uint32_t now_ms);

/* Abort the profile immediately and force the generated duty to zero. */
void mid_motor_trapezoid_stop(MidMotorTrapezoid *ramp);

int32_t mid_motor_trapezoid_get_duty(const MidMotorTrapezoid *ramp);
MidMotorTrapezoidPhase
mid_motor_trapezoid_get_phase(const MidMotorTrapezoid *ramp);
uint8_t mid_motor_trapezoid_is_running(const MidMotorTrapezoid *ramp);

#endif /* MID_MOTOR_TRAPEZOID_H */
