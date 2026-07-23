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
 * 循迹模块通过 UART1 (PA9 RX / PA8 TX, 115200-8N1) 连续发送
 * '0' / '1' 字符表示各路传感器状态。
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

    if (status & DL_UART_MAIN_IIDX_RX) {
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
 * UART3 (F32C) - 二自由度云台阻塞发送
 *
 * UART3 已由 SYSCFG_DL_f32c_init() 按 115200-8N1 初始化并使能，
 * 此处只封装使用 f32c_INST 的底层发送操作。
 * ================================================================ */

void drv_f32c_uart_init(void)
{
    /* UART3 已在 SYSCFG_DL_init() 中完成初始化，无需再次等待或复位。 */
}

void drv_f32c_uart_write(const uint8_t *data, uint8_t length)
{
    uint8_t i;

    if (data == 0) {
        return;
    }

    for (i = 0U; i < length; i++) {
        DL_UART_Main_transmitDataBlocking(f32c_INST, data[i]);
    }

    /*
     * 不在此处无限等待 UART BUSY 清零。阻塞发送已经保证所有字节
     * 写入 FIFO，上层的帧间隔会等待最后一个字节发送完成。
     */
}
