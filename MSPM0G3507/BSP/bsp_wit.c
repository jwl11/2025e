#include "bsp_wit.h"

#define WIT_FRAME_HEADER       (0x55U)
#define WIT_FRAME_SIZE         (11U)
#define WIT_TYPE_ACCEL         (0x51U)
#define WIT_TYPE_GYRO          (0x52U)
#define WIT_TYPE_ANGLE         (0x53U)
#define WIT_MIN_FRAME_TYPE     (0x50U)
#define WIT_MAX_FRAME_TYPE     (0x5FU)
#define WIT_DMA_BLOCK_SIZE     (66U)
#define WIT_DMA_BUFFER_COUNT   (2U)

volatile WIT_Data_t wit_data;

static uint8_t wit_frame[WIT_FRAME_SIZE];
static uint8_t wit_last_frame[WIT_FRAME_SIZE];
static uint8_t wit_frame_pos;
static volatile uint32_t wit_received_byte_count;
static volatile uint32_t wit_valid_frame_count;
static volatile uint32_t wit_checksum_error_count;
static volatile uint32_t wit_accel_frame_count;
static volatile uint32_t wit_gyro_frame_count;
static volatile uint32_t wit_angle_frame_count;
static volatile uint8_t wit_dma_buffer[WIT_DMA_BUFFER_COUNT][WIT_DMA_BLOCK_SIZE];
static volatile uint8_t wit_dma_active_buffer;

static int16_t WIT_ReadS16(const uint8_t *data)
{
    return (int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
}

static bool WIT_ChecksumIsValid(const uint8_t *frame)
{
    uint8_t checksum = 0U;
    uint8_t i;

    for (i = 0U; i < (WIT_FRAME_SIZE - 1U); i++) {
        checksum = (uint8_t)(checksum + frame[i]);
    }

    return checksum == frame[WIT_FRAME_SIZE - 1U];
}

static void WIT_DecodeFrame(const uint8_t *frame)
{
    int16_t raw0 = WIT_ReadS16(&frame[2]);
    int16_t raw1 = WIT_ReadS16(&frame[4]);
    int16_t raw2 = WIT_ReadS16(&frame[6]);
    int16_t raw3 = WIT_ReadS16(&frame[8]);

    switch (frame[1]) {
        case WIT_TYPE_ACCEL:
            wit_data.ax = raw0;
            wit_data.ay = raw1;
            wit_data.az = raw2;
            wit_data.temperature = (float)raw3 / 100.0f;
            wit_accel_frame_count++;
            break;

        case WIT_TYPE_GYRO:
            wit_data.gx = raw0;
            wit_data.gy = raw1;
            wit_data.gz = raw2;
            wit_gyro_frame_count++;
            break;

        case WIT_TYPE_ANGLE:
            wit_data.roll = (float)raw0 * 180.0f / 32768.0f;
            wit_data.pitch = (float)raw1 * 180.0f / 32768.0f;
            wit_data.yaw = (float)raw2 * 180.0f / 32768.0f;
            wit_data.version = raw3;
            wit_angle_frame_count++;
            break;

        default:
            break;
    }

    wit_valid_frame_count++;
}

static void WIT_ParseByte(uint8_t data)
{
    uint8_t i;

    wit_received_byte_count++;

    if (wit_frame_pos == 0U) {
        if (data == WIT_FRAME_HEADER) {
            wit_frame[wit_frame_pos++] = data;
        }
        return;
    }

    if (wit_frame_pos == 1U) {
        if ((data >= WIT_MIN_FRAME_TYPE) && (data <= WIT_MAX_FRAME_TYPE)) {
            wit_frame[wit_frame_pos++] = data;
        } else if (data == WIT_FRAME_HEADER) {
            wit_frame[0] = data;
        } else {
            wit_frame_pos = 0U;
        }
        return;
    }

    wit_frame[wit_frame_pos++] = data;
    if (wit_frame_pos != WIT_FRAME_SIZE) {
        return;
    }

    for (i = 0U; i < WIT_FRAME_SIZE; i++) {
        wit_last_frame[i] = wit_frame[i];
    }

    if (WIT_ChecksumIsValid(wit_frame)) {
        WIT_DecodeFrame(wit_frame);
    } else {
        wit_checksum_error_count++;
    }
    wit_frame_pos = 0U;
}

void WIT_Init(void)
{
    DL_DMA_Config dmaCfg;

    wit_frame_pos = 0U;
    wit_received_byte_count = 0U;
    wit_valid_frame_count = 0U;
    wit_checksum_error_count = 0U;
    wit_accel_frame_count = 0U;
    wit_gyro_frame_count = 0U;
    wit_angle_frame_count = 0U;
    wit_dma_active_buffer = 0U;

    /*
     * SysConfig set up UART2 (115200-8N1-RX-FIFO) and DMA_CH0 skeleton.
     * Here we: (1) enable the UART→DMA handshake, (2) make the DMA
     * destination auto-increment across the ping-pong buffer,
     * (3) switch from per-byte RX IRQ to DMA-done IRQ.
     */
    NVIC_DisableIRQ(UART_WIT_INST_INT_IRQN);
    DL_UART_Main_disable(UART_WIT_INST);

    /* ---- (1) UART → DMA handshake ---- */
    DL_UART_Main_enableDMAReceiveEvent(UART_WIT_INST,
                                       DL_UART_DMA_INTERRUPT_RX);

    /* drain any stale bytes before starting */
    DL_UART_Main_enable(UART_WIT_INST);
    while (!DL_UART_Main_isRXFIFOEmpty(UART_WIT_INST)) {
        (void)DL_UART_Main_receiveData(UART_WIT_INST);
    }

    /* ---- (2) DMA: override SysConfig's UNCHANGED dest ---- */
    DL_DMA_disableChannel(DMA, DMA_WIT_CHAN_ID);

    dmaCfg.transferMode  = DL_DMA_SINGLE_TRANSFER_MODE;
    dmaCfg.extendedMode  = DL_DMA_NORMAL_MODE;
    dmaCfg.destIncrement = DL_DMA_ADDR_INCREMENT;   /* key fix */
    dmaCfg.srcIncrement  = DL_DMA_ADDR_UNCHANGED;
    dmaCfg.destWidth     = DL_DMA_WIDTH_BYTE;
    dmaCfg.srcWidth      = DL_DMA_WIDTH_BYTE;
    dmaCfg.trigger       = UART_WIT_INST_DMA_TRIGGER;
    dmaCfg.triggerType   = DL_DMA_TRIGGER_TYPE_EXTERNAL;

    DL_DMA_initChannel(DMA, DMA_WIT_CHAN_ID, &dmaCfg);
    DL_DMA_setSrcAddr(DMA, DMA_WIT_CHAN_ID,
        (uint32_t) &UART_WIT_INST->RXDATA);
    DL_DMA_setDestAddr(DMA, DMA_WIT_CHAN_ID,
        (uint32_t) &wit_dma_buffer[0][0]);
    DL_DMA_setTransferSize(DMA, DMA_WIT_CHAN_ID, WIT_DMA_BLOCK_SIZE);
    DL_DMA_enableChannel(DMA, DMA_WIT_CHAN_ID);

    /* ---- (3) UART interrupts: DMA-done (primary) + RX-timeout (safety) ---- */
    DL_UART_Main_disableInterrupt(UART_WIT_INST,
        DL_UART_MAIN_INTERRUPT_RX);                    /* DMA replaces per-byte RX */
    DL_UART_Main_enableInterrupt(UART_WIT_INST,
        DL_UART_MAIN_INTERRUPT_DMA_DONE_RX |
        DL_UART_MAIN_INTERRUPT_RX_TIMEOUT_ERROR);

    NVIC_ClearPendingIRQ(UART_WIT_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_WIT_INST_INT_IRQN);
}

void WIT_Poll(void)
{
    /* DMA owns UART2 RXDATA; completed blocks are handled by the UART2 ISR. */
}

uint32_t WIT_GetValidFrameCount(void)
{
    return wit_valid_frame_count;
}

void WIT_GetDebugSnapshot(WIT_Debug_t *snapshot)
{
    uint8_t i;

    if (snapshot == 0) {
        return;
    }

    NVIC_DisableIRQ(UART_WIT_INST_INT_IRQN);
    snapshot->received_bytes = wit_received_byte_count;
    snapshot->valid_frames = wit_valid_frame_count;
    snapshot->checksum_errors = wit_checksum_error_count;
    snapshot->accel_frames = wit_accel_frame_count;
    snapshot->gyro_frames = wit_gyro_frame_count;
    snapshot->angle_frames = wit_angle_frame_count;
    snapshot->rx_pin_high =
        (DL_GPIO_readPins(GPIO_UART_WIT_RX_PORT, GPIO_UART_WIT_RX_PIN) != 0U);
    for (i = 0U; i < WIT_FRAME_SIZE; i++) {
        snapshot->last_frame[i] = wit_last_frame[i];
    }
    NVIC_EnableIRQ(UART_WIT_INST_INT_IRQN);
}

void UART_WIT_INST_IRQHandler(void)
{
    uint8_t completed_buffer;
    uint8_t data;
    uint8_t i;

    switch (DL_UART_Main_getPendingInterrupt(UART_WIT_INST)) {

        /* ---- DMA done (main data path, ~16 times/s) ---- */
        case DL_UART_MAIN_IIDX_DMA_DONE_RX:
            completed_buffer = wit_dma_active_buffer;
            wit_dma_active_buffer ^= 1U;

            /* re-point DMA at the other buffer, reload count, restart */
            DL_DMA_setDestAddr(DMA, DMA_WIT_CHAN_ID,
                (uint32_t) &wit_dma_buffer[wit_dma_active_buffer][0]);
            DL_DMA_setTransferSize(
                DMA, DMA_WIT_CHAN_ID, WIT_DMA_BLOCK_SIZE);
            DL_DMA_enableChannel(DMA, DMA_WIT_CHAN_ID);

            /* parse the just-completed buffer */
            for (i = 0U; i < WIT_DMA_BLOCK_SIZE; i++) {
                WIT_ParseByte(wit_dma_buffer[completed_buffer][i]);
            }
            break;

        /* ---- RX timeout (safety net: DMA stalled, drain FIFO manually) ---- */
        case DL_UART_MAIN_IIDX_RX_TIMEOUT_ERROR:
            while (DL_UART_Main_receiveDataCheck(UART_WIT_INST, &data)) {
                WIT_ParseByte(data);
            }
            break;

        default:
            break;
    }
}
