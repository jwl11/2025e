#include "bsp_42step.h"
#include "drv_zdt_x35_uart.h"
#include "mid_delay.h"

/*
 * Emm_V5.0 串口协议帧：
 *   地址(1) + 命令(1) + 数据(N) + 校验(1)
 * 校验默认固定 0x6B。
 */

#define ADDR             0x01U
#define CHECKSUM         0x6BU
#define CMD_VERSION      0x1FU
#define CMD_ENABLE       0xF3U
#define CMD_VELOCITY     0xF6U
#define CMD_POSITION     0xFDU
#define CMD_STOP         0xFEU
#define CMD_READ_STATUS  0x3AU
#define CMD_READ_POS     0x36U
#define CMD_READ_ERR     0x37U
#define CMD_CLEAR_POS    0x0AU
#define CMD_CLEAR_KEY    0x6DU
#define CMD_SET_ZERO     0x93U
#define CMD_SET_ZERO_KEY 0x88U
#define CMD_GO_HOME      0x9AU
#define CMD_MODE_SWITCH  0x46U
#define CMD_MODE_KEY     0x69U
#define ENABLE_KEY       0xABU
#define STOP_KEY         0x98U
#define MAX_RPM          5000U
#define ACK_TIMEOUT_MS   100U

static uint32_t pulses_per_rev = 3200;   /* 16 细分 */

/* ---- 内部发送 ---- */

static Step42Result send_frame(const uint8_t *frame, uint16_t len)
{
    return drv_zdt_x35_uart_write(frame, len)
           ? STEP42_OK : STEP42_ERR_UART;
}

/* ---- 等待 ACK ---- */

/**
 * 读取一条回复帧到 buf (最大 maxlen), 返回帧长度, 超时返回 0。
 * 控制命令回复格式: ADDR cmd status CHECKSUM
 * status: 0x02=成功, 0xE2=条件不满足, 0xEE=命令错误
 */
static uint8_t read_ack(uint8_t *buf, uint8_t maxlen)
{
    uint32_t t0 = 0;
    uint8_t  len = 0;

    while (len < maxlen) {
        int16_t ch = drv_zdt_x35_uart_read();
        if (ch >= 0) {
            buf[len++] = (uint8_t)ch;
            if (ch == CHECKSUM) break;  /* 帧尾 */
        } else {
            if (len == 0) {
                /* 还没收到任何字节, 检查超时 */
                if (t0 == 0) t0 = get_system_ms();
                else if (get_system_ms() - t0 > ACK_TIMEOUT_MS) return 0;
            }
        }
    }
    return len;
}

/** 发送命令并检查 ACK */
static Step42Result send_with_ack(const uint8_t *frame, uint16_t len)
{
    if (!drv_zdt_x35_uart_write(frame, len))
        return STEP42_ERR_UART;

    /* 清 RX 缓冲区中可能残留的旧数据 */
    while (drv_zdt_x35_uart_available() > 0)
        drv_zdt_x35_uart_read();

    uint8_t ack[8];
    uint8_t alen = read_ack(ack, sizeof(ack));
    if (alen == 0) return STEP42_ERR_TIMEOUT;
    if (alen < 4)  return STEP42_ERR_ACK;   /* 帧太短 */

    /* 帧格式: ADDR cmd status CHECKSUM */
    if (ack[2] == 0x02U) return STEP42_OK;        /* 成功 */
    if (ack[2] == 0xE2U) return STEP42_ERR_ACK;   /* 条件不满足 */
    if (ack[2] == 0xEEU) return STEP42_ERR_ACK;   /* 命令错误 */
    return STEP42_ERR_ACK;
}

/* ---- 基础 API (带 ACK) ---- */

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
    return send_with_ack(f, sizeof(f));
}

Step42Result Step42_RunVelocity(Step42Dir dir, uint16_t rpm, uint8_t acc)
{
    if (rpm > MAX_RPM) return STEP42_ERR_ARG;
    const uint8_t f[] = {
        ADDR, CMD_VELOCITY, (uint8_t)dir,
        (uint8_t)(rpm >> 8), (uint8_t)rpm, acc, 0x00U, CHECKSUM
    };
    return send_frame(f, sizeof(f));  /* 控制循环不发 ACK */
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
    return send_frame(f, sizeof(f));  /* 控制循环不发 ACK */
}

Step42Result Step42_Stop(void)
{
    const uint8_t f[] = { ADDR, CMD_STOP, STOP_KEY, 0x00U, CHECKSUM };
    return send_frame(f, sizeof(f));  /* 控制循环不发 ACK */
}

/* ---- 闭环 & 状态 API ---- */

Step42Result Step42_SetClosedLoop(void)
{
    const uint8_t f[] = {
        ADDR, CMD_MODE_SWITCH, CMD_MODE_KEY,
        0x01U, 0x02U, CHECKSUM       /* 参数=1(接收), 模式=2(闭环) */
    };
    return send_with_ack(f, sizeof(f));
}

Step42Result Step42_ClearPosition(void)
{
    const uint8_t f[] = { ADDR, CMD_CLEAR_POS, CMD_CLEAR_KEY, CHECKSUM };
    return send_with_ack(f, sizeof(f));
}

Step42Result Step42_SetZero(void)
{
    const uint8_t f[] = { ADDR, CMD_SET_ZERO, CMD_SET_ZERO_KEY, CHECKSUM };
    return send_with_ack(f, sizeof(f));
}

Step42Result Step42_GoHome(void)
{
    const uint8_t f[] = { ADDR, CMD_GO_HOME, CHECKSUM };
    return send_with_ack(f, sizeof(f));
}

Step42Result Step42_ReadStatus(Step42Status *st)
{
    const uint8_t f[] = { ADDR, CMD_READ_STATUS, CHECKSUM };
    if (!drv_zdt_x35_uart_write(f, sizeof(f)))
        return STEP42_ERR_UART;

    /* 清旧数据 */
    while (drv_zdt_x35_uart_available() > 0)
        drv_zdt_x35_uart_read();

    uint8_t ack[8];
    uint8_t alen = read_ack(ack, sizeof(ack));
    if (alen < 5) return STEP42_ERR_TIMEOUT;

    /* 帧格式: ADDR 3A status 6B */
    uint8_t s = ack[2];
    st->enabled          = (s & 0x01) != 0;
    st->position_reached = (s & 0x02) != 0;
    st->stalled          = (s & 0x04) != 0;
    st->homing_done      = (s & 0x08) != 0;
    return STEP42_OK;
}

Step42Result Step42_ReadPosition(int32_t *pos)
{
    const uint8_t f[] = { ADDR, CMD_READ_POS, CHECKSUM };
    if (!drv_zdt_x35_uart_write(f, sizeof(f)))
        return STEP42_ERR_UART;

    while (drv_zdt_x35_uart_available() > 0)
        drv_zdt_x35_uart_read();

    uint8_t ack[8];
    uint8_t alen = read_ack(ack, sizeof(ack));
    if (alen < 7) return STEP42_ERR_TIMEOUT;

    /* 帧格式: ADDR 36 pos[4] 6B (大端) */
    *pos = ((int32_t)ack[2] << 24) | ((int32_t)ack[3] << 16)
         | ((int32_t)ack[4] << 8)  |  (int32_t)ack[5];
    return STEP42_OK;
}

Step42Result Step42_ReadPositionError(int32_t *err)
{
    const uint8_t f[] = { ADDR, CMD_READ_ERR, CHECKSUM };
    if (!drv_zdt_x35_uart_write(f, sizeof(f)))
        return STEP42_ERR_UART;

    while (drv_zdt_x35_uart_available() > 0)
        drv_zdt_x35_uart_read();

    uint8_t ack[8];
    uint8_t alen = read_ack(ack, sizeof(ack));
    if (alen < 7) return STEP42_ERR_TIMEOUT;

    /* 帧格式: ADDR 37 err[4] 6B (大端) */
    *err = ((int32_t)ack[2] << 24) | ((int32_t)ack[3] << 16)
         | ((int32_t)ack[4] << 8)  |  (int32_t)ack[5];
    return STEP42_OK;
}

bool Step42_IsStalled(void)
{
    Step42Status st;
    if (Step42_ReadStatus(&st) != STEP42_OK) return false;
    return st.stalled;
}

/* ---- 角度控制 ---- */

void Step42_SetPulsesPerRev(uint32_t ppr)
{
    pulses_per_rev = ppr;
}

Step42Result Step42_MoveToAngle(float degrees, uint16_t rpm, uint8_t acc)
{
    int32_t pulses = (int32_t)(degrees / 360.0f * (float)pulses_per_rev);
    Step42Dir dir = (pulses >= 0) ? STEP42_DIR_CW : STEP42_DIR_CCW;
    uint32_t abs_p = (uint32_t)(pulses >= 0 ? pulses : -pulses);
    return Step42_MovePosition(dir, rpm, acc, abs_p, STEP42_POS_ABSOLUTE);
}
