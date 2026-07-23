#include "drv_encoder.h"

#define ENCODER_SPEED_PERIOD_MS       50U
#define ENCODER_SPEED_SCALE_PER_SEC  (1000U / ENCODER_SPEED_PERIOD_MS)
#define ENCODER_COUNTS_PER_REV       260.0f
#define MOTOR_WHEEL_DIAMETER_MM       48.0f
#define MOTOR_PI                       3.1415926f

static Encoder *g_encoder[2] = {0};
static bool g_timer_started = false;

/*
 * GPIO polarity registers use two bits per pin:
 * 01 = rising edge, 10 = falling edge, 11 = both edges.
 * E1A/E2A are lower-port pins in the current SysConfig. Deriving the
 * polarity from the generated pin mask avoids hard-coding PB13/PB10 here.
 */
static uint32_t encoder_lower_pin_rising_polarity(uint32_t pin)
{
    uint32_t index = 0U;

    while (((pin & 1U) == 0U) && (index < 16U)) {
        pin >>= 1U;
        index++;
    }

    return (index < 16U) ? (1UL << (index * 2U)) : 0U;
}

static void encoder_count_a_rising(Encoder *enc)
{
    int32_t delta;

    if (enc == 0) {
        return;
    }

    /*
     * A has just risen. B=0 and B=1 represent opposite directions.
     * If the physical motor direction is reversed, direction_sign in the
     * BSP mapping changes the sign without rewiring the encoder.
     */
    delta = (DL_GPIO_readPins(enc->port_b, enc->pin_b) == 0U) ? 1 : -1;
    enc->count += (int32_t)enc->direction_sign * delta;
}

static void encoder_update_speed(Encoder *enc)
{
    int32_t count_now;
    int32_t count_delta;
    int32_t instant_speed;
    float speed_mm_s;

    if (enc == 0) {
        return;
    }

    count_now = enc->count;
    count_delta = count_now - enc->last_speed_count;

    /*
     * speed(mm/s) = counts / counts_per_rev
     *             * wheel_circumference(mm) / sample_time(s)
     */
    speed_mm_s = ((float)count_delta *
                  (float)ENCODER_SPEED_SCALE_PER_SEC *
                  MOTOR_PI * MOTOR_WHEEL_DIAMETER_MM) /
                 ENCODER_COUNTS_PER_REV;

    instant_speed = (speed_mm_s >= 0.0f) ?
                    (int32_t)(speed_mm_s + 0.5f) :
                    (int32_t)(speed_mm_s - 0.5f);

    enc->speed_history_sum -=
        enc->speed_history[enc->speed_history_index];
    enc->speed_history[enc->speed_history_index] = instant_speed;
    enc->speed_history_sum += instant_speed;
    enc->speed_history_index++;
    if (enc->speed_history_index >= ENCODER_SPEED_FILTER_SIZE) {
        enc->speed_history_index = 0U;
    }
    if (enc->speed_history_count < ENCODER_SPEED_FILTER_SIZE) {
        enc->speed_history_count++;
    }

    enc->speed = enc->speed_history_sum /
                 (int32_t)enc->speed_history_count;
    enc->last_speed_count = count_now;
    enc->speed_sample_sequence++;
}

static void encoder_enable_motor_a_gpio_irq(void)
{
    /* Only E1A generates a rising-edge interrupt. E1B is direction. */
    DL_GPIO_disableInterrupt(MG310_E1A_PORT, MG310_E1A_PIN);
    DL_GPIO_setLowerPinsPolarity(
        MG310_E1A_PORT,
        encoder_lower_pin_rising_polarity(MG310_E1A_PIN));
    DL_GPIO_clearInterruptStatus(MG310_E1A_PORT, MG310_E1A_PIN);
    DL_GPIO_enableInterrupt(MG310_E1A_PORT, MG310_E1A_PIN);
}

static void encoder_enable_motor_b_gpio_irq(void)
{
    /* Only E2A generates a rising-edge interrupt. E2B is direction. */
    DL_GPIO_disableInterrupt(MG310_E2A_PORT, MG310_E2A_PIN);
    DL_GPIO_setLowerPinsPolarity(
        MG310_E2A_PORT,
        encoder_lower_pin_rising_polarity(MG310_E2A_PIN));
    DL_GPIO_clearInterruptStatus(MG310_E2A_PORT, MG310_E2A_PIN);
    DL_GPIO_enableInterrupt(MG310_E2A_PORT, MG310_E2A_PIN);
}

void encoder_init(Encoder *enc,
                  EncoderChannel channel,
                  int8_t direction_sign)
{
    enc->channel = channel;
    enc->direction_sign = (direction_sign < 0) ? -1 : 1;
    enc->count = 0;
    enc->speed = 0;
    enc->last_speed_count = 0;
    enc->speed_sample_sequence = 0U;
    enc->speed_history_sum = 0;
    enc->speed_history_index = 0U;
    enc->speed_history_count = 0U;
    for (uint32_t i = 0U; i < ENCODER_SPEED_FILTER_SIZE; i++) {
        enc->speed_history[i] = 0;
    }

    if (channel == ENCODER_MOTOR_A) {
        enc->port_a = MG310_E1A_PORT;
        enc->pin_a  = MG310_E1A_PIN;
        enc->port_b = MG310_E1B_PORT;
        enc->pin_b  = MG310_E1B_PIN;
        g_encoder[ENCODER_MOTOR_A] = enc;
        encoder_enable_motor_a_gpio_irq();
    } else {
        enc->port_a = MG310_E2A_PORT;
        enc->pin_a  = MG310_E2A_PIN;
        enc->port_b = MG310_E2B_PORT;
        enc->pin_b  = MG310_E2B_PIN;
        g_encoder[ENCODER_MOTOR_B] = enc;
        encoder_enable_motor_b_gpio_irq();
    }

    NVIC_ClearPendingIRQ(GPIOB_INT_IRQn);
    NVIC_EnableIRQ(GPIOB_INT_IRQn);

    if (!g_timer_started) {
        NVIC_ClearPendingIRQ(MOTOR_MG310_INST_INT_IRQN);
        NVIC_EnableIRQ(MOTOR_MG310_INST_INT_IRQN);
        DL_TimerA_startCounter(MOTOR_MG310_INST);
        g_timer_started = true;
    }
}

int32_t encoder_get_count(const Encoder *enc)
{
    return enc->count;
}

int32_t encoder_get_speed(const Encoder *enc)
{
    return enc->speed;
}

uint32_t encoder_get_speed_sample_sequence(const Encoder *enc)
{
    return enc->speed_sample_sequence;
}

void encoder_reset_count(Encoder *enc)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    enc->count = 0;
    enc->speed = 0;
    enc->last_speed_count = 0;
    enc->speed_history_sum = 0;
    enc->speed_history_index = 0U;
    enc->speed_history_count = 0U;
    for (uint32_t i = 0U; i < ENCODER_SPEED_FILTER_SIZE; i++) {
        enc->speed_history[i] = 0;
    }
    if (primask == 0U) {
        __enable_irq();
    }
}

static void encoder_handle_gpiob(void)
{
    const uint32_t mask = MG310_E1A_PIN | MG310_E2A_PIN;
    uint32_t status = DL_GPIO_getEnabledInterruptStatus(GPIOB, mask);

    if ((status & MG310_E1A_PIN) != 0U) {
        DL_GPIO_clearInterruptStatus(GPIOB, MG310_E1A_PIN);
        encoder_count_a_rising(g_encoder[ENCODER_MOTOR_A]);
    }

    if ((status & MG310_E2A_PIN) != 0U) {
        DL_GPIO_clearInterruptStatus(GPIOB, MG310_E2A_PIN);
        encoder_count_a_rising(g_encoder[ENCODER_MOTOR_B]);
    }
}

void GROUP1_IRQHandler(void)
{
    switch (DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1)) {
    case DL_INTERRUPT_GROUP1_IIDX_GPIOB:
        encoder_handle_gpiob();
        break;

    default:
        break;
    }
}

void MOTOR_MG310_INST_IRQHandler(void)
{
    /*
     * Read IIDX exactly once. Reading IIDX acknowledges the highest-priority
     * pending source. Re-reading it in a do/while loop can keep this ISR busy
     * on some timer states and prevent the main program from stopping PWM.
     */
    if (DL_TimerA_getPendingInterrupt(MOTOR_MG310_INST) ==
        DL_TIMERA_IIDX_LOAD) {
        encoder_update_speed(g_encoder[ENCODER_MOTOR_A]);
        encoder_update_speed(g_encoder[ENCODER_MOTOR_B]);
    }
}
