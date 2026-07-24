/*
 * SysConfig Configuration Steps:
 *   UART:
 *     1. Add an UART module.
 *     2. Name it as "UART_WIT".
 *     3. Set "Target Baud Rate" according to module configuration.
 *     4. Set "Communication Direction" to "RX only".
 *     5. Enable FIFO and set RX FIFO threshold to one entry.
 *     6. Enable the UART RX DMA trigger.
 *     7. Enable the DMA_DONE_RX interrupt.
 *     8. Name the RX DMA channel "DMA_WIT".
 *     9. Set its address mode to fixed-to-block and both widths to byte.
 *     10. Set the pin according to your needs.
 *
 * DMA fills alternating 66-byte buffers. The CPU is interrupted once per
 * completed block and the byte-stream parser resynchronizes on 0x55.
*/

#ifndef __BSP_WIT_H
#define __BSP_WIT_H

#include "ti_msp_dl_config.h"

typedef struct {
    float pitch;
    float roll;
    float yaw;
    float temperature;
    int16_t ax;
    int16_t ay;
    int16_t az;
    int16_t gx;
    int16_t gy;
    int16_t gz;
    int16_t version;
} WIT_Data_t;

typedef struct {
    uint32_t received_bytes;
    uint32_t valid_frames;
    uint32_t checksum_errors;
    uint32_t accel_frames;
    uint32_t gyro_frames;
    uint32_t angle_frames;
    uint8_t rx_pin_high;
    uint8_t last_frame[11];
} WIT_Debug_t;

extern volatile WIT_Data_t wit_data;

void WIT_Init(void);
void WIT_Poll(void);
uint32_t WIT_GetValidFrameCount(void);
void WIT_GetDebugSnapshot(WIT_Debug_t *snapshot);

#endif /* #ifndef __WIT_H */
