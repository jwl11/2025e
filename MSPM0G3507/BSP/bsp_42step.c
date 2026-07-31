#include "bsp_42step.h"
#include "drv_zdt_x35_uart.h"

/*
 * Emm_V5.0 串口协议帧：
 *   地址(1) + 命令(1) + 数据(N) + 校验(1)
 *
 * 校验默认固定 0x6B，每条指令最后一字节固定为 0x6B。
 * 地址固定 0x01，转速 0~5000 RPM。
 */

#define ADDR           0x01U
#define CHECKSUM       0x6BU
#define CMD_VERSION    0x1FU
#define CMD_ENABLE     0xF3U
#define CMD_VELOCITY   0xF6U
#define CMD_POSITION   0xFDU
#define CMD_STOP       0xFEU
#define ENABLE_KEY     0xABU
#define STOP_KEY       0x98U
#define MAX_RPM        5000U

/* ---- 内部发送 ---- */

static Step42Result send_frame(const uint8_t *frame, uint16_t len)
{
    return drv_zdt_x35_uart_write(frame, len)
           ? STEP42_OK : STEP42_ERR_UART;
}

/* ---- API ---- */

void Step42_Init(void)
{
    drv_zdt_x35_uart_init();
}

Step42Result Step42_QueryVersion(void)
{
    const uint8_t f[] = { ADDR, CMD_VERSION, CHECKSUM };
    return send_frame(f, sizeof(f));
}

Step42Result Step42_Enable(bool enable)
{
    const uint8_t f[] = {
        ADDR, CMD_ENABLE, ENABLE_KEY,
        enable ? 0x01U : 0x00U, 0x00U, CHECKSUM
    };
    return send_frame(f, sizeof(f));
}

Step42Result Step42_RunVelocity(Step42Dir dir, uint16_t rpm, uint8_t acc)
{
    if (rpm > MAX_RPM) return STEP42_ERR_ARG;

    const uint8_t f[] = {
        ADDR, CMD_VELOCITY, (uint8_t)dir,
        (uint8_t)(rpm >> 8), (uint8_t)rpm, acc, 0x00U, CHECKSUM
    };
    return send_frame(f, sizeof(f));
}

Step42Result Step42_MovePosition(Step42Dir dir, uint16_t rpm, uint8_t acc,
                                  uint32_t pulses, Step42PosMode mode)
{
    if (rpm > MAX_RPM) return STEP42_ERR_ARG;

    const uint8_t f[] = {
        ADDR, CMD_POSITION, (uint8_t)dir,
        (uint8_t)(rpm >> 8), (uint8_t)rpm, acc,
        (uint8_t)(pulses >> 24), (uint8_t)(pulses >> 16),
        (uint8_t)(pulses >> 8),  (uint8_t)pulses,
        (uint8_t)mode, 0x00U, CHECKSUM
    };
    return send_frame(f, sizeof(f));
}

Step42Result Step42_Stop(void)
{
    const uint8_t f[] = { ADDR, CMD_STOP, STOP_KEY, 0x00U, CHECKSUM };
    return send_frame(f, sizeof(f));
}
