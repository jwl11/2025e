#include "bsp_f32c_gimbal.h"
#include "drv_uart.h"

#define F32C_FRAME_HEAD              0x7AU
#define F32C_FRAME_TAIL              0x7BU
#define F32C_FUNC_SELECT_MODE        0x00U
#define F32C_FUNC_SET_SPEED          0x01U
#define F32C_FUNC_SINGLE_TURN_ANGLE  0x03U
#define F32C_FUNC_DISABLE            0x05U
#define F32C_FUNC_ENABLE             0x06U
#define F32C_MODE_SINGLE_TURN_T      0x0002U
#define F32C_MAX_SPEED_RPM           1000U
#define F32C_MAX_ANGLE_TENTHS        3599U

static uint8_t f32c_bcc(const uint8_t *frame, uint8_t length)
{
    uint8_t i;
    uint8_t value = 0U;

    for (i = 0U; i < length; i++) {
        value ^= frame[i];
    }
    return value;
}

static void f32c_send_command0(uint8_t address, uint8_t function)
{
    uint8_t frame[5];

    frame[0] = F32C_FRAME_HEAD;
    frame[1] = address;
    frame[2] = function;
    frame[3] = f32c_bcc(frame, 3U);
    frame[4] = F32C_FRAME_TAIL;
    drv_f32c_uart_write(frame, (uint8_t)sizeof(frame));
}

static void f32c_send_command16(uint8_t address, uint8_t function,
                                uint16_t value)
{
    uint8_t frame[7];

    frame[0] = F32C_FRAME_HEAD;
    frame[1] = address;
    frame[2] = function;
    frame[3] = (uint8_t)(value >> 8);
    frame[4] = (uint8_t)value;
    frame[5] = f32c_bcc(frame, 5U);
    frame[6] = F32C_FRAME_TAIL;
    drv_f32c_uart_write(frame, (uint8_t)sizeof(frame));
}

void bsp_f32c_gimbal_init(void)
{
    drv_f32c_uart_init();
}

void bsp_f32c_motor_enable(uint8_t address)
{
    f32c_send_command0(address, F32C_FUNC_ENABLE);
}

void bsp_f32c_motor_disable(uint8_t address)
{
    f32c_send_command0(address, F32C_FUNC_DISABLE);
}

void bsp_f32c_motor_set_single_turn_mode(uint8_t address)
{
    f32c_send_command16(address, F32C_FUNC_SELECT_MODE,
                        F32C_MODE_SINGLE_TURN_T);
}

bool bsp_f32c_motor_set_speed(uint8_t address, uint16_t rpm)
{
    if ((rpm == 0U) || (rpm > F32C_MAX_SPEED_RPM)) {
        return false;
    }

    f32c_send_command16(address, F32C_FUNC_SET_SPEED, rpm);
    return true;
}

bool bsp_f32c_motor_set_single_turn_angle(uint8_t address, float degree)
{
    uint16_t angle_tenths;

    if ((degree < 0.0f) || (degree > 359.9f)) {
        return false;
    }

    angle_tenths = (uint16_t)(degree * 10.0f + 0.5f);
    if (angle_tenths > F32C_MAX_ANGLE_TENTHS) {
        angle_tenths = F32C_MAX_ANGLE_TENTHS;
    }

    f32c_send_command16(address, F32C_FUNC_SINGLE_TURN_ANGLE,
                        angle_tenths);
    return true;
}

void bsp_f32c_gimbal_enable(void)
{
    bsp_f32c_motor_enable(F32C_GIMBAL_X_ID);
    bsp_f32c_motor_enable(F32C_GIMBAL_Y_ID);
}

void bsp_f32c_gimbal_disable(void)
{
    bsp_f32c_motor_disable(F32C_GIMBAL_X_ID);
    bsp_f32c_motor_disable(F32C_GIMBAL_Y_ID);
}

void bsp_f32c_gimbal_set_single_turn_mode(void)
{
    bsp_f32c_motor_set_single_turn_mode(F32C_GIMBAL_X_ID);
    bsp_f32c_motor_set_single_turn_mode(F32C_GIMBAL_Y_ID);
}

bool bsp_f32c_gimbal_set_speed(uint16_t rpm)
{
    if ((rpm == 0U) || (rpm > F32C_MAX_SPEED_RPM)) {
        return false;
    }

    (void)bsp_f32c_motor_set_speed(F32C_GIMBAL_X_ID, rpm);
    (void)bsp_f32c_motor_set_speed(F32C_GIMBAL_Y_ID, rpm);
    return true;
}

bool bsp_f32c_gimbal_set_angle(float x_degree, float y_degree)
{
    if ((x_degree < 0.0f) || (x_degree > 359.9f) ||
        (y_degree < 0.0f) || (y_degree > 359.9f)) {
        return false;
    }

    (void)bsp_f32c_motor_set_single_turn_angle(F32C_GIMBAL_X_ID, x_degree);
    (void)bsp_f32c_motor_set_single_turn_angle(F32C_GIMBAL_Y_ID, y_degree);
    return true;
}
