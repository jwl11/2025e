#include "drv_zdt_x35_uart.h"

/* 缓冲区长度必须是2的整数次幂，便于使用位运算回绕。 */
#define ZDT_X35_UART_RX_BUFFER_SIZE 128U
#define ZDT_X35_UART_RX_BUFFER_MASK (ZDT_X35_UART_RX_BUFFER_SIZE - 1U)

/* 单字节发送最大轮询次数，防止串口故障时程序永久卡死。 */
#define ZDT_X35_UART_TX_RETRY_LIMIT 100000U

static volatile uint8_t g_zdt_x35_rx_buffer[ZDT_X35_UART_RX_BUFFER_SIZE];
static volatile uint16_t g_zdt_x35_rx_head;
static volatile uint16_t g_zdt_x35_rx_tail;
static volatile uint32_t g_zdt_x35_rx_overflow_count;

void drv_zdt_x35_uart_init(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    g_zdt_x35_rx_head = 0U;
    g_zdt_x35_rx_tail = 0U;
    g_zdt_x35_rx_overflow_count = 0U;

    /* 清除上电前后可能残留在硬件FIFO中的无效字节。 */
    while (!DL_UART_Main_isRXFIFOEmpty(ZDT_X35_INST)) {
        (void)DL_UART_Main_receiveData(ZDT_X35_INST);
    }

    DL_UART_Main_enableInterrupt(ZDT_X35_INST, DL_UART_MAIN_INTERRUPT_RX);
    NVIC_ClearPendingIRQ(ZDT_X35_INST_INT_IRQN);
    NVIC_EnableIRQ(ZDT_X35_INST_INT_IRQN);

    if (primask == 0U) {
        __enable_irq();
    }
}

bool drv_zdt_x35_uart_write(const uint8_t *data, uint16_t length)
{
    uint16_t i;

    if ((data == 0) || (length == 0U)) {
        return false;
    }

    for (i = 0U; i < length; i++) {
        uint32_t retry;

        for (retry = 0U; retry < ZDT_X35_UART_TX_RETRY_LIMIT; retry++) {
            if (DL_UART_Main_transmitDataCheck(ZDT_X35_INST, data[i])) {
                break;
            }
        }

        if (retry >= ZDT_X35_UART_TX_RETRY_LIMIT) {
            return false;
        }
    }

    return true;
}

uint16_t drv_zdt_x35_uart_available(void)
{
    return (g_zdt_x35_rx_head - g_zdt_x35_rx_tail) &
        ZDT_X35_UART_RX_BUFFER_MASK;
}

int16_t drv_zdt_x35_uart_read(void)
{
    uint8_t data;

    if (g_zdt_x35_rx_head == g_zdt_x35_rx_tail) {
        return -1;
    }

    data = g_zdt_x35_rx_buffer[g_zdt_x35_rx_tail];
    g_zdt_x35_rx_tail =
        (g_zdt_x35_rx_tail + 1U) & ZDT_X35_UART_RX_BUFFER_MASK;
    return (int16_t)data;
}

void drv_zdt_x35_uart_flush(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    g_zdt_x35_rx_head = 0U;
    g_zdt_x35_rx_tail = 0U;
    if (primask == 0U) {
        __enable_irq();
    }
}

uint32_t drv_zdt_x35_uart_get_overflow_count(void)
{
    return g_zdt_x35_rx_overflow_count;
}

void ZDT_X35_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(ZDT_X35_INST)) {
    case DL_UART_MAIN_IIDX_RX:
        while (!DL_UART_Main_isRXFIFOEmpty(ZDT_X35_INST)) {
            uint8_t data = DL_UART_Main_receiveData(ZDT_X35_INST);
            uint16_t next =
                (g_zdt_x35_rx_head + 1U) & ZDT_X35_UART_RX_BUFFER_MASK;

            if (next != g_zdt_x35_rx_tail) {
                g_zdt_x35_rx_buffer[g_zdt_x35_rx_head] = data;
                g_zdt_x35_rx_head = next;
            } else {
                /* 缓冲区满时丢弃新字节，并记录溢出次数。 */
                g_zdt_x35_rx_overflow_count++;
            }
        }
        break;

    default:
        break;
    }
}
