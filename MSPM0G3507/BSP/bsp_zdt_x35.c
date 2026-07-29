#include "bsp_zdt_x35.h"

#include "drv_zdt_x35_uart.h"

#include <string.h>

#define ZDT_X35_CMD_READ_VERSION 0x1FU
#define ZDT_X35_CMD_ENABLE       0xF3U
#define ZDT_X35_CMD_VELOCITY     0xF6U
#define ZDT_X35_CMD_POSITION     0xFDU
#define ZDT_X35_CMD_STOP         0xFEU

#define ZDT_X35_ENABLE_KEY       0xABU
#define ZDT_X35_STOP_KEY         0x98U
#define ZDT_X35_MAX_SPEED_RPM    3000U

static ZdtX35ProbeState g_zdt_x35_probe;
static uint8_t g_zdt_x35_build[ZDT_X35_FRAME_MAX_LENGTH];
static uint8_t g_zdt_x35_build_length;

static ZdtX35Result zdt_x35_send_frame(
    const uint8_t *frame,
    uint16_t length)
{
    return drv_zdt_x35_uart_write(frame, length)
        ? ZDT_X35_RESULT_OK
        : ZDT_X35_RESULT_UART_ERROR;
}

void zdt_x35_init(void)
{
    memset(&g_zdt_x35_probe, 0, sizeof(g_zdt_x35_probe));
    memset(g_zdt_x35_build, 0, sizeof(g_zdt_x35_build));
    g_zdt_x35_build_length = 0U;
    drv_zdt_x35_uart_init();
}

ZdtX35Result zdt_x35_request_version(void)
{
    const uint8_t frame[] = {
        ZDT_X35_MOTOR_ADDRESS,
        ZDT_X35_CMD_READ_VERSION,
        ZDT_X35_FIXED_CHECKSUM
    };

    return zdt_x35_send_frame(frame, sizeof(frame));
}

ZdtX35Result zdt_x35_set_enable(bool enable, bool sync)
{
    const uint8_t frame[] = {
        ZDT_X35_MOTOR_ADDRESS,
        ZDT_X35_CMD_ENABLE,
        ZDT_X35_ENABLE_KEY,
        enable ? 0x01U : 0x00U,
        sync ? 0x01U : 0x00U,
        ZDT_X35_FIXED_CHECKSUM
    };

    return zdt_x35_send_frame(frame, sizeof(frame));
}

ZdtX35Result zdt_x35_run_velocity(
    ZdtX35Direction direction,
    uint16_t speed_rpm,
    uint8_t acc,
    bool sync)
{
    uint8_t frame[8];

    if (((direction != ZDT_X35_DIRECTION_CW) &&
         (direction != ZDT_X35_DIRECTION_CCW)) ||
        (speed_rpm > ZDT_X35_MAX_SPEED_RPM)) {
        return ZDT_X35_RESULT_BAD_ARGUMENT;
    }

    /*
     * Emm_V5速度帧：
     * 地址 F6 方向 速度高字节 速度低字节 加速度 同步 6B
     */
    frame[0] = ZDT_X35_MOTOR_ADDRESS;
    frame[1] = ZDT_X35_CMD_VELOCITY;
    frame[2] = (uint8_t)direction;
    frame[3] = (uint8_t)(speed_rpm >> 8U);
    frame[4] = (uint8_t)speed_rpm;
    frame[5] = acc;
    frame[6] = sync ? 0x01U : 0x00U;
    frame[7] = ZDT_X35_FIXED_CHECKSUM;

    return zdt_x35_send_frame(frame, sizeof(frame));
}

ZdtX35Result zdt_x35_move_position(
    ZdtX35Direction direction,
    uint16_t speed_rpm,
    uint8_t acc,
    uint32_t pulse_count,
    ZdtX35PositionMode mode,
    bool sync)
{
    uint8_t frame[13];

    if (((direction != ZDT_X35_DIRECTION_CW) &&
         (direction != ZDT_X35_DIRECTION_CCW)) ||
        ((mode != ZDT_X35_POSITION_RELATIVE) &&
         (mode != ZDT_X35_POSITION_ABSOLUTE)) ||
        (speed_rpm > ZDT_X35_MAX_SPEED_RPM)) {
        return ZDT_X35_RESULT_BAD_ARGUMENT;
    }

    /*
     * Emm_V5位置帧：
     * 地址 FD 方向 速度H 速度L 加速度 脉冲数(32位大端)
     * 相对/绝对 同步 6B
     */
    frame[0] = ZDT_X35_MOTOR_ADDRESS;
    frame[1] = ZDT_X35_CMD_POSITION;
    frame[2] = (uint8_t)direction;
    frame[3] = (uint8_t)(speed_rpm >> 8U);
    frame[4] = (uint8_t)speed_rpm;
    frame[5] = acc;
    frame[6] = (uint8_t)(pulse_count >> 24U);
    frame[7] = (uint8_t)(pulse_count >> 16U);
    frame[8] = (uint8_t)(pulse_count >> 8U);
    frame[9] = (uint8_t)pulse_count;
    frame[10] = (uint8_t)mode;
    frame[11] = sync ? 0x01U : 0x00U;
    frame[12] = ZDT_X35_FIXED_CHECKSUM;

    return zdt_x35_send_frame(frame, sizeof(frame));
}

ZdtX35Result zdt_x35_stop(bool sync)
{
    const uint8_t frame[] = {
        ZDT_X35_MOTOR_ADDRESS,
        ZDT_X35_CMD_STOP,
        ZDT_X35_STOP_KEY,
        sync ? 0x01U : 0x00U,
        ZDT_X35_FIXED_CHECKSUM
    };

    return zdt_x35_send_frame(frame, sizeof(frame));
}

static void zdt_x35_decode_frame(void)
{
    g_zdt_x35_probe.is_version_response = false;
    g_zdt_x35_probe.is_command_response = false;
    g_zdt_x35_probe.command = 0U;
    g_zdt_x35_probe.command_status = 0U;
    g_zdt_x35_probe.firmware_version = 0U;
    g_zdt_x35_probe.hardware_version = 0U;

    if (g_zdt_x35_probe.length < 3U) {
        return;
    }

    g_zdt_x35_probe.command = g_zdt_x35_probe.data[1];

    /* 版本回复固定为：地址 1F 固件版本 硬件版本 6B。 */
    if ((g_zdt_x35_probe.length == 5U) &&
        (g_zdt_x35_probe.command == ZDT_X35_CMD_READ_VERSION)) {
        g_zdt_x35_probe.firmware_version = g_zdt_x35_probe.data[2];
        g_zdt_x35_probe.hardware_version = g_zdt_x35_probe.data[3];
        g_zdt_x35_probe.is_version_response = true;
        return;
    }

    /*
     * 控制命令回复固定为：地址 功能码 状态 6B。
     * 状态0x02表示接收成功，0xE2表示条件不满足。
     */
    if ((g_zdt_x35_probe.length == 4U) &&
        ((g_zdt_x35_probe.command == ZDT_X35_CMD_ENABLE) ||
         (g_zdt_x35_probe.command == ZDT_X35_CMD_VELOCITY) ||
         (g_zdt_x35_probe.command == ZDT_X35_CMD_POSITION) ||
         (g_zdt_x35_probe.command == ZDT_X35_CMD_STOP))) {
        g_zdt_x35_probe.command_status = g_zdt_x35_probe.data[2];
        g_zdt_x35_probe.is_command_response = true;
    }
}

static void zdt_x35_parse_byte(uint8_t data)
{
    /* 等待地址1作为一帧开头，过滤上电杂波。 */
    if (g_zdt_x35_build_length == 0U) {
        if (data != ZDT_X35_MOTOR_ADDRESS) {
            return;
        }
        g_zdt_x35_build[g_zdt_x35_build_length++] = data;
        return;
    }

    if (g_zdt_x35_build_length >= ZDT_X35_FRAME_MAX_LENGTH) {
        g_zdt_x35_probe.invalid_frame_count++;
        g_zdt_x35_build_length = 0U;
        return;
    }

    g_zdt_x35_build[g_zdt_x35_build_length++] = data;

    /*
     * 固定校验模式下0x6B为帧尾。此处故意先保存原始帧，不按V2或Emm
     * 的长度解释，便于通过实机回包确认X35_V1.3到底使用哪代协议。
     */
    if (data == ZDT_X35_FIXED_CHECKSUM) {
        if (!g_zdt_x35_probe.ready) {
            memcpy(
                g_zdt_x35_probe.data,
                g_zdt_x35_build,
                g_zdt_x35_build_length);
            g_zdt_x35_probe.length = g_zdt_x35_build_length;
            g_zdt_x35_probe.ready = true;
            g_zdt_x35_probe.valid_frame_count++;
            zdt_x35_decode_frame();
        } else {
            /* 上一帧尚未处理时不覆盖，记录为丢帧。 */
            g_zdt_x35_probe.invalid_frame_count++;
        }
        g_zdt_x35_build_length = 0U;
    }
}

void zdt_x35_poll(void)
{
    int16_t value;

    while (drv_zdt_x35_uart_available() > 0U) {
        value = drv_zdt_x35_uart_read();
        if (value >= 0) {
            zdt_x35_parse_byte((uint8_t)value);
        }
    }
}

const ZdtX35ProbeState *zdt_x35_get_probe_state(void)
{
    return &g_zdt_x35_probe;
}

void zdt_x35_consume_frame(void)
{
    g_zdt_x35_probe.ready = false;
    g_zdt_x35_probe.length = 0U;
    g_zdt_x35_probe.is_version_response = false;
    g_zdt_x35_probe.is_command_response = false;
}
