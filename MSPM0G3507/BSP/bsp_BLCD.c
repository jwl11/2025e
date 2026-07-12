#include "bsp_BLCD.h"
#include "mid_foc.h"
#include "bsp_AS5600.h"

/* ================================================================
 *  BLDC BSP — 薄封装层
 * ================================================================ */

void bsp_bldc_init(float vbus, int pp, int dir)
{
    foc_init(vbus);
    foc_as5600_init(pp, dir);
}

void bsp_bldc_set_speed(float target_rad_s)
{
    foc_set_speed(target_rad_s);
}

void bsp_bldc_set_angle(float target_rad)
{
    foc_set_angle(target_rad);
}

float bsp_bldc_get_speed(void)
{
    return as5600_get_speed();
}

float bsp_bldc_get_angle(void)
{
    return as5600_get_angle_radians();
}

void bsp_bldc_stop(void)
{
    foc_stop();
}
