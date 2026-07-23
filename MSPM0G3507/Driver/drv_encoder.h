#ifndef __DRV_ENCODER_H
#define __DRV_ENCODER_H

#include "ti_msp_dl_config.h"

/*
 * MG310 quadrature encoder driver
 *
 * The four encoder signals remain ordinary GPIO inputs in empty.syscfg.
 * This driver enables a rising-edge interrupt on each motor's A phase.
 * The corresponding B phase level is read in the ISR to determine direction,
 * matching the common single-motor reference implementation (1x decoding).
 *
 * MOTOR_MG310/TIMA1 keeps the user's 50 ms periodic LOAD configuration.
 * Its interrupt calculates wheel linear speed in mm/s.
 *
 * Wheel diameter: 48 mm
 * Encoder resolution: 260 counts/revolution in A-rising 1x mode
 */

typedef enum {
    ENCODER_MOTOR_A = 0,
    ENCODER_MOTOR_B = 1,
} EncoderChannel;

#define ENCODER_SPEED_FILTER_SIZE  4U

typedef struct {
    GPIO_Regs *port_a;
    uint32_t   pin_a;
    GPIO_Regs *port_b;
    uint32_t   pin_b;
    EncoderChannel channel;
    int8_t direction_sign;
    volatile int32_t count;
    volatile int32_t speed;
    volatile int32_t last_speed_count;
    volatile uint32_t speed_sample_sequence;
    int32_t speed_history[ENCODER_SPEED_FILTER_SIZE];
    int32_t speed_history_sum;
    uint8_t speed_history_index;
    uint8_t speed_history_count;
} Encoder;

void encoder_init(Encoder *enc,
                  EncoderChannel channel,
                  int8_t direction_sign);

int32_t encoder_get_count(const Encoder *enc);
/** @return Wheel linear speed in mm/s. */
int32_t encoder_get_speed(const Encoder *enc);
/** @return Monotonic counter incremented after every 50 ms speed sample. */
uint32_t encoder_get_speed_sample_sequence(const Encoder *enc);
void encoder_reset_count(Encoder *enc);

#endif /* __DRV_ENCODER_H */
