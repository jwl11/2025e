#include "drv_uart.h"
#include <stdio.h>

#define UART0_TX_RETRY_LIMIT  100000U

static bool uart0_write_byte(uint8_t data)
{
    uint32_t retry;

    for (retry = 0U; retry < UART0_TX_RETRY_LIMIT; retry++) {
        if (DL_UART_Main_transmitDataCheck(debug_INST, data)) {
            return true;
        }
    }

    return false;
}

void drv_uart0_init(void)
{
    DL_UART_Main_enable(debug_INST);
}

/**
 * @brief  重定向 printf 底层输出到 UART
 */
int fputc(int ch, FILE *f)
{
    (void)f;
    (void)uart0_write_byte((uint8_t)ch);
    return ch;
}
/**
 * @brief  发送字符串（阻塞）
 */
void drv_uart_send_string(const char *str)
{
    while (*str != '\0')
    {
        if (!uart0_write_byte((uint8_t)(*str))) {
            return;
        }
        str++;
    }
}

/**
 * @brief  打印无符号整数（绕过 printf 格式化限制）
 *
 *         嵌入式 newlib-nano 的 printf 在多参数或非 long
 *         类型时可能无法正确打印，此函数直接逐位转换发送。
 */
void drv_uart_print_num(unsigned long num)
{
    char  buf[12];  /* 0-4294967295 最多 10 位 + 符号 + null */
    uint8_t i = 0;

    if (num == 0) {
        buf[i++] = '0';
    } else {
        /* 从低位到高位填入 */
        uint8_t j = 0;
        char    tmp[12];
        while (num > 0) {
            tmp[j++] = '0' + (num % 10);
            num /= 10;
        }
        /* 反转得到高位到低位 */
        while (j > 0) {
            buf[i++] = tmp[--j];
        }
    }
    buf[i] = '\0';
    drv_uart_send_string(buf);
}

/**
 * @brief  打印有符号整数
 *
 *         负数打印 '-' 前缀，内部转 unsigned long 复用 print_num。
 */
void drv_uart_print_signed(long num)
{
    if (num < 0) {
        drv_uart_send_string("-");
        num = -num;
    }
    drv_uart_print_num((unsigned long)num);
}

/**
 * @brief  打印两位十六进制数（0x00 ~ 0xFF）
 */
void drv_uart_print_hex(uint8_t num)
{
    char hex[3];
    char nibble;

    nibble = (num >> 4) & 0x0F;
    hex[0] = (nibble < 10) ? ('0' + nibble) : ('A' + nibble - 10);
    nibble = num & 0x0F;
    hex[1] = (nibble < 10) ? ('0' + nibble) : ('A' + nibble - 10);
    hex[2] = '\0';
    drv_uart_send_string(hex);
}

/* ================================================================
 * UART1 (fishpath) 中断接收 — 12路循迹模块数据采集
 *
 * 循迹模块通过 UART1 (PB7 RX / PB6 TX, 115200-8N1) 周期发送
 * "#xxxxxxxxxxxx!" 固定帧，x 为各路传感器的 ASCII '0' / '1'。
 *
 * 使用环形缓冲区, ISR 写入 → 主循环读取, 无需关中断保护
 * (Cortex-M0+ aligned 8-bit 访问为原子操作)。
 * ================================================================ */

#define UART1_RX_BUF_SIZE   1024U

static volatile uint8_t uart1_rx_buf[UART1_RX_BUF_SIZE];
static volatile uint16_t uart1_rx_head = 0;
static volatile uint16_t uart1_rx_tail = 0;

/**
 * @brief  初始化 UART1 中断接收
 * @note   硬件初始化已在 SYSCFG_DL_fishpath_init() 中完成,
 *         此处仅开启 RX 中断。
 */
void drv_uart1_init(void)
{
    uart1_rx_head = 0;
    uart1_rx_tail = 0;

    DL_UART_Main_enableInterrupt(fishpath_INST, DL_UART_MAIN_INTERRUPT_RX);
    NVIC_EnableIRQ(fishpath_INST_INT_IRQN);
}

/**
 * @brief  获取环形缓冲区中待读取字节数
 */
uint16_t drv_uart1_available(void)
{
    return (uart1_rx_head - uart1_rx_tail) & (UART1_RX_BUF_SIZE - 1);
}

/**
 * @brief  从环形缓冲区读取一个字节
 * @return 0~255 有效字节, -1 表示缓冲区空
 */
int16_t drv_uart1_read(void)
{
    uint8_t data;

    if (uart1_rx_head == uart1_rx_tail) {
        return -1;
    }
    data = uart1_rx_buf[uart1_rx_tail];
    uart1_rx_tail = (uart1_rx_tail + 1) & (UART1_RX_BUF_SIZE - 1);
    return (int16_t)data;
}

/**
 * @brief  清空环形缓冲区 (丢弃所有未处理数据)
 */
void drv_uart1_flush(void)
{
    uart1_rx_head = 0;
    uart1_rx_tail = 0;
}

/**
 * @brief  UART1 中断服务函数
 *
 *         将接收到的字节写入环形缓冲区。
 *         缓冲区满时丢弃新字节 (不影响旧数据)。
 */
void UART1_IRQHandler(void)
{
    uint32_t status = DL_UART_Main_getPendingInterrupt(fishpath_INST);

    if (status == DL_UART_MAIN_IIDX_RX) {
        uint8_t data = DL_UART_Main_receiveData(fishpath_INST);
        uint16_t next = (uart1_rx_head + 1U) & (UART1_RX_BUF_SIZE - 1U);

        if (next != uart1_rx_tail) {
            uart1_rx_buf[uart1_rx_head] = data;
            uart1_rx_head = next;
        }
        /* else: buffer full, drop byte (keep existing data intact) */
    }
}

/* ================================================================
 * UART3 (maixcam) — MaixCAM2 坐标接收 + ISR 直解析 @X,Y#
 *
 * PB3=RX, PB2=TX, 115200-8N1。
 * ISR 内直接解析协议帧，主循环零延迟读取坐标。
 * ================================================================ */

/* ISR 内协议解析状态 */
typedef enum {
    VISION_PARSE_WAIT_HEADER = 0,
    VISION_PARSE_X,
    VISION_PARSE_Y,
} VisionParseState;

static VisionParseState parse_state = VISION_PARSE_WAIT_HEADER;
static uint16_t          parse_x    = 0U;
static uint16_t          parse_y    = 0U;
volatile uint16_t vision_x   = 0U;
volatile uint16_t vision_y   = 0U;
static volatile uint32_t vision_frame_count = 0U;

void drv_vision_uart3_init(void)
{
    parse_state = VISION_PARSE_WAIT_HEADER;
    parse_x     = 0U;
    parse_y     = 0U;
    vision_x    = 0U;
    vision_y    = 0U;
    vision_frame_count = 0U;

    /* 重开 UART3 清 FIFO 和错误标志，保留 SysConfig 的波特率配置 */
    DL_UART_Main_disable(maixcam_INST);
    while (!DL_UART_Main_isRXFIFOEmpty(maixcam_INST)) {
        (void)DL_UART_Main_receiveData(maixcam_INST);
    }
    DL_UART_Main_enable(maixcam_INST);
    DL_UART_Main_enableInterrupt(maixcam_INST, DL_UART_MAIN_INTERRUPT_RX);
    NVIC_ClearPendingIRQ(maixcam_INST_INT_IRQN);
    NVIC_EnableIRQ(maixcam_INST_INT_IRQN);
}

bool drv_vision_uart3_write(const uint8_t *data, uint16_t length)
{
    uint16_t i;

    if ((data == 0) || (length == 0U)) {
        return false;
    }

    for (i = 0U; i < length; i++) {
        DL_UART_Main_transmitDataBlocking(maixcam_INST, data[i]);
    }
    return true;
}

uint16_t drv_vision_get_x(void)  { return vision_x; }
uint16_t drv_vision_get_y(void)  { return vision_y; }
uint32_t drv_vision_get_frame_count(void) { return vision_frame_count; }

void UART3_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(maixcam_INST)) {
    case DL_UART_MAIN_IIDX_RX:
        while (!DL_UART_Main_isRXFIFOEmpty(maixcam_INST)) {
            uint8_t data = DL_UART_Main_receiveData(maixcam_INST);

            switch (parse_state) {
            case VISION_PARSE_WAIT_HEADER:
                if (data == '@') { parse_state = VISION_PARSE_X; parse_x = 0U; }
                break;
            case VISION_PARSE_X:
                if (data >= '0' && data <= '9')
                    parse_x = parse_x * 10U + (uint16_t)(data - '0');
                else if (data == ',') { parse_state = VISION_PARSE_Y; parse_y = 0U; }
                else parse_state = VISION_PARSE_WAIT_HEADER;
                break;
            case VISION_PARSE_Y:
                if (data >= '0' && data <= '9')
                    parse_y = parse_y * 10U + (uint16_t)(data - '0');
                else if (data == '#') {
                    vision_x = parse_x; vision_y = parse_y;
                    vision_frame_count++; parse_state = VISION_PARSE_WAIT_HEADER;
                } else parse_state = VISION_PARSE_WAIT_HEADER;
                break;
            }
        }
        break;
    default:
        break;
    }
}
