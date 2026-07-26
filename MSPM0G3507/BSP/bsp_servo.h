#ifndef __BSP_SERVO_H
#define __BSP_SERVO_H

#include "ti_msp_dl_config.h"
#include "drv_servo.h"

/* ================================================================
 * 舵机 BSP 层
 *
 * 封装 drv_servo, 供应用层调用。
 * ================================================================ */

void bsp_servo_init(void);
void bsp_servo_setAngle(uint8_t angle);
void bsp_servo_setCC(uint32_t cc_val);

#endif /* __BSP_SERVO_H */
