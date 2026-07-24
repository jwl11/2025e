#include "ti_msp_dl_config.h"
#include "bsp_wit.h"
#include "bsp_OLED.h"
#include "drv_uart.h"
#include "mid_delay.h"

#define WIT_PAGE_ANGLE       (0U)
#define WIT_PAGE_ACCEL       (1U)
#define WIT_PAGE_GYRO        (2U)
#define WIT_PAGE_COUNT       (3U)
#define WIT_PAGE_TICKS       (10U)
#define WIT_DEBUG_TICKS      (10U)

static void UART_PrintRawFrame(const uint8_t *frame)
{
    uint8_t i;

    for (i = 0U; i < 11U; i++) {
        drv_uart_print_hex(frame[i]);
        if (i < 10U) {
            drv_uart_send_string(" ");
        }
    }
}

static void UART_PrintWITDebug(void)
{
    WIT_Debug_t debug;

    WIT_GetDebugSnapshot(&debug);

    drv_uart_send_string("[WIT] bytes=");
    drv_uart_print_num(debug.received_bytes);
    drv_uart_send_string(" valid=");
    drv_uart_print_num(debug.valid_frames);
    drv_uart_send_string(" bad_sum=");
    drv_uart_print_num(debug.checksum_errors);
    drv_uart_send_string(" frames(51/52/53)=");
    drv_uart_print_num(debug.accel_frames);
    drv_uart_send_string("/");
    drv_uart_print_num(debug.gyro_frames);
    drv_uart_send_string("/");
    drv_uart_print_num(debug.angle_frames);
    drv_uart_send_string(" PB18=");
    drv_uart_print_num(debug.rx_pin_high);
    drv_uart_send_string("\r\n");

    drv_uart_send_string("      angle_x10 P/R/Y=");
    drv_uart_print_signed((long)(wit_data.pitch * 10.0f));
    drv_uart_send_string("/");
    drv_uart_print_signed((long)(wit_data.roll * 10.0f));
    drv_uart_send_string("/");
    drv_uart_print_signed((long)(wit_data.yaw * 10.0f));
    drv_uart_send_string(" last=");
    UART_PrintRawFrame(debug.last_frame);
    drv_uart_send_string("\r\n");

    if (debug.received_bytes == 0U) {
        drv_uart_send_string(
            "      HINT: no UART2 data; check JY61P TX -> PB18 and GND.\r\n");
    } else if ((debug.valid_frames == 0U) &&
               (debug.checksum_errors != 0U)) {
        drv_uart_send_string(
            "      HINT: bytes arrive but checksum fails; check baud rate.\r\n");
    } else if ((debug.valid_frames != 0U) &&
               (debug.angle_frames == 0U)) {
        drv_uart_send_string(
            "      HINT: valid data but no type 0x53 angle frame.\r\n");
    }
}

static void OLED_ShowAngle(uint8_t line, float angle)
{
    int32_t tenths;
    uint32_t magnitude;

    if (angle >= 0.0f) {
        tenths = (int32_t)(angle * 10.0f + 0.5f);
        OLED_ShowChar(line, 7, '+');
    } else {
        tenths = (int32_t)(angle * 10.0f - 0.5f);
        OLED_ShowChar(line, 7, '-');
    }

    magnitude = (tenths < 0) ? (uint32_t)(-tenths) : (uint32_t)tenths;
    OLED_ShowNum(line, 8, magnitude / 10U, 3);
    OLED_ShowChar(line, 11, '.');
    OLED_ShowNum(line, 12, magnitude % 10U, 1);
}

static void OLED_DrawPage(uint8_t page)
{
    OLED_Clear();

    if (page == WIT_PAGE_ANGLE) {
        OLED_ShowString(1, 1, "Pitch:");
        OLED_ShowString(2, 1, "Roll :");
        OLED_ShowString(3, 1, "Yaw  :");
        OLED_ShowString(4, 1, "Frames:");
    } else if (page == WIT_PAGE_ACCEL) {
        OLED_ShowString(1, 1, "ACCEL RAW");
        OLED_ShowString(2, 1, "AX:");
        OLED_ShowString(3, 1, "AY:");
        OLED_ShowString(4, 1, "AZ:");
    } else {
        OLED_ShowString(1, 1, "GYRO RAW");
        OLED_ShowString(2, 1, "GX:");
        OLED_ShowString(3, 1, "GY:");
        OLED_ShowString(4, 1, "GZ:");
    }
}

static void OLED_UpdatePage(uint8_t page)
{
    if (page == WIT_PAGE_ANGLE) {
        OLED_ShowAngle(1, wit_data.pitch);
        OLED_ShowAngle(2, wit_data.roll);
        OLED_ShowAngle(3, wit_data.yaw);
        OLED_ShowNum(4, 8, WIT_GetValidFrameCount() % 100000000U, 8);
    } else if (page == WIT_PAGE_ACCEL) {
        OLED_ShowSignedNum(2, 4, wit_data.ax, 5);
        OLED_ShowSignedNum(3, 4, wit_data.ay, 5);
        OLED_ShowSignedNum(4, 4, wit_data.az, 5);
    } else {
        OLED_ShowSignedNum(2, 4, wit_data.gx, 5);
        OLED_ShowSignedNum(3, 4, wit_data.gy, 5);
        OLED_ShowSignedNum(4, 4, wit_data.gz, 5);
    }
}

void app_wit_test(void)
{
    uint8_t page = WIT_PAGE_ANGLE;
    uint8_t page_ticks = 0U;
    uint8_t debug_ticks = 0U;

    drv_uart0_init();
    drv_uart_send_string("\r\n========================================\r\n");
    drv_uart_send_string(" JY61P WIT test\r\n");
    drv_uart_send_string(" Sensor RX: UART2 PB18, 115200-8N1\r\n");
    drv_uart_send_string(" RX mode   : DMA_CH0, double 66-byte buffer\r\n");
    drv_uart_send_string(" Debug TX : UART0 PA10, 115200-8N1\r\n");
    drv_uart_send_string(" Expected : 0x51 accel, 0x52 gyro, 0x53 angle\r\n");
    drv_uart_send_string("========================================\r\n");

    OLED_Init();
    WIT_Init();

    OLED_DrawPage(page);

    while (1) 
    {
        OLED_UpdatePage(page);
        delay_ms(100U);

        debug_ticks++;
        if (debug_ticks >= WIT_DEBUG_TICKS) {
            debug_ticks = 0U;
            UART_PrintWITDebug();
        }

        page_ticks++;
        if (page_ticks >= WIT_PAGE_TICKS) {
            page_ticks = 0U;
            page++;
            if (page >= WIT_PAGE_COUNT) {
                page = WIT_PAGE_ANGLE;
            }
            OLED_DrawPage(page);
        }
    }
}
