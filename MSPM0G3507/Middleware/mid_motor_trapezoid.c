#include "mid_motor_trapezoid.h"

#define MID_MOTOR_DUTY_MAX       100
#define MID_MOTOR_DURATION_MAX   42949672UL

static int32_t clamp_duty(int32_t duty)
{
    if (duty > MID_MOTOR_DUTY_MAX) {
        return MID_MOTOR_DUTY_MAX;
    }
    if (duty < -MID_MOTOR_DUTY_MAX) {
        return -MID_MOTOR_DUTY_MAX;
    }
    return duty;
}

static uint32_t clamp_duration(uint32_t duration_ms)
{
    /*
     * Keeps peak_duty * elapsed inside uint32_t.  The resulting maximum
     * duration is over 11 hours, well beyond a normal motor ramp.
     */
    if (duration_ms > MID_MOTOR_DURATION_MAX) {
        return MID_MOTOR_DURATION_MAX;
    }
    return duration_ms;
}

static int32_t signed_duty(int32_t peak_duty, uint32_t magnitude)
{
    return (peak_duty < 0) ? -(int32_t)magnitude : (int32_t)magnitude;
}

void mid_motor_trapezoid_init(MidMotorTrapezoid *ramp)
{
    if (ramp == 0) {
        return;
    }

    ramp->peak_duty = 0;
    ramp->current_duty = 0;
    ramp->accel_ms = 0U;
    ramp->cruise_ms = 0U;
    ramp->decel_ms = 0U;
    ramp->start_ms = 0U;
    ramp->phase = MID_MOTOR_TRAPEZOID_IDLE;
    ramp->running = 0U;
}

void mid_motor_trapezoid_start(MidMotorTrapezoid *ramp,
                               int32_t peak_duty,
                               uint32_t accel_ms,
                               uint32_t cruise_ms,
                               uint32_t decel_ms,
                               uint32_t now_ms)
{
    if (ramp == 0) {
        return;
    }

    ramp->peak_duty = clamp_duty(peak_duty);
    ramp->current_duty = 0;
    ramp->accel_ms = clamp_duration(accel_ms);
    ramp->cruise_ms = clamp_duration(cruise_ms);
    ramp->decel_ms = clamp_duration(decel_ms);
    ramp->start_ms = now_ms;
    ramp->phase = MID_MOTOR_TRAPEZOID_ACCEL;
    ramp->running = (ramp->peak_duty != 0) ? 1U : 0U;

    if (ramp->running == 0U) {
        ramp->phase = MID_MOTOR_TRAPEZOID_IDLE;
    }
}

int32_t mid_motor_trapezoid_update(MidMotorTrapezoid *ramp,
                                   uint32_t now_ms)
{
    uint32_t elapsed;
    uint32_t peak_magnitude;
    uint32_t magnitude;

    if ((ramp == 0) || (ramp->running == 0U)) {
        return 0;
    }

    elapsed = now_ms - ramp->start_ms;
    peak_magnitude = (ramp->peak_duty < 0) ?
                     (uint32_t)(-ramp->peak_duty) :
                     (uint32_t)ramp->peak_duty;

    if ((ramp->accel_ms != 0U) && (elapsed < ramp->accel_ms)) {
        magnitude = (peak_magnitude * elapsed) / ramp->accel_ms;
        ramp->current_duty = signed_duty(ramp->peak_duty, magnitude);
        ramp->phase = MID_MOTOR_TRAPEZOID_ACCEL;
        return ramp->current_duty;
    }

    if (elapsed >= ramp->accel_ms) {
        elapsed -= ramp->accel_ms;
    } else {
        elapsed = 0U;
    }

    if (elapsed < ramp->cruise_ms) {
        ramp->current_duty = ramp->peak_duty;
        ramp->phase = MID_MOTOR_TRAPEZOID_CRUISE;
        return ramp->current_duty;
    }

    elapsed -= ramp->cruise_ms;

    if ((ramp->decel_ms != 0U) && (elapsed < ramp->decel_ms)) {
        magnitude =
            (peak_magnitude * (ramp->decel_ms - elapsed)) /
            ramp->decel_ms;
        ramp->current_duty = signed_duty(ramp->peak_duty, magnitude);
        ramp->phase = MID_MOTOR_TRAPEZOID_DECEL;
        return ramp->current_duty;
    }

    mid_motor_trapezoid_stop(ramp);
    return 0;
}

void mid_motor_trapezoid_stop(MidMotorTrapezoid *ramp)
{
    if (ramp == 0) {
        return;
    }

    ramp->current_duty = 0;
    ramp->phase = MID_MOTOR_TRAPEZOID_IDLE;
    ramp->running = 0U;
}

int32_t mid_motor_trapezoid_get_duty(const MidMotorTrapezoid *ramp)
{
    return (ramp != 0) ? ramp->current_duty : 0;
}

MidMotorTrapezoidPhase
mid_motor_trapezoid_get_phase(const MidMotorTrapezoid *ramp)
{
    return (ramp != 0) ? ramp->phase : MID_MOTOR_TRAPEZOID_IDLE;
}

uint8_t mid_motor_trapezoid_is_running(const MidMotorTrapezoid *ramp)
{
    return (ramp != 0) ? ramp->running : 0U;
}
