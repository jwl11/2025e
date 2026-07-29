#ifndef __DRV_ZDT_X35_UART_H
#define __DRV_ZDT_X35_UART_H

#include "ti_msp_dl_config.h"

#include <stdbool.h>
#include <stdint.h>

/*
 * 张大头 X35_V1.3（Emm_V5.0固件）TTL 串口底层。
 *
 * 接线与参数：
 *   PA23(UART2_TX) -> 电机RX
 *   PA24(UART2_RX) <- 电机TX
 *   GND            -> 电机GND
 *   115200、8数据位、无校验、1停止位
 *
 * 本层只收发字节，不解释上层命令。
 */

/** 初始化UART2接收环形缓冲区并打开接收中断。 */
void drv_zdt_x35_uart_init(void);

/** 阻塞发送一段数据；发送成功返回true。 */
bool drv_zdt_x35_uart_write(const uint8_t *data, uint16_t length);

/** 返回尚未处理的接收字节数。 */
uint16_t drv_zdt_x35_uart_available(void);

/** 读取一个字节；没有数据时返回-1。 */
int16_t drv_zdt_x35_uart_read(void);

/** 清空尚未处理的接收字节。 */
void drv_zdt_x35_uart_flush(void);

/** 返回因软件环形缓冲区已满而丢弃的字节数。 */
uint32_t drv_zdt_x35_uart_get_overflow_count(void);

#endif /* __DRV_ZDT_X35_UART_H */
