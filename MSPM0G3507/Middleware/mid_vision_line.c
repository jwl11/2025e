#include "mid_vision_line.h"
#include "drv_uart.h"

#define VISION_SOF0 0xAAU
#define VISION_SOF1 0x55U

static uint8_t frame_buffer[VISION_LINE_FRAME_SIZE];
static uint8_t frame_index;
static uint32_t crc_error_count;
static uint32_t format_error_count;

static int16_t read_i16_le(const uint8_t *data)
{
    return (int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8U));
}

static uint16_t read_u16_le(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
}

uint16_t vision_line_crc16_ccitt(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFFU;
    uint16_t i;

    for (i = 0U; i < length; i++) {
        uint8_t bit;
        crc ^= (uint16_t)data[i] << 8U;

        for (bit = 0U; bit < 8U; bit++) {
            if ((crc & 0x8000U) != 0U) {
                crc = (uint16_t)((crc << 1U) ^ 0x1021U);
            } else {
                crc <<= 1U;
            }
        }
    }
    return crc;
}

void vision_line_protocol_init(void)
{
    frame_index = 0U;
    crc_error_count = 0U;
    format_error_count = 0U;
    drv_vision_uart3_flush();
}

static bool decode_frame(VisionLineFrame *frame)
{
    uint16_t received_crc;
    uint16_t calculated_crc;

    if ((frame_buffer[2] != VISION_LINE_PROTOCOL_VERSION) ||
        (frame_buffer[3] != VISION_LINE_MSG_ID)) {
        format_error_count++;
        return false;
    }

    received_crc = read_u16_le(&frame_buffer[14]);
    calculated_crc = vision_line_crc16_ccitt(frame_buffer, 14U);
    if (received_crc != calculated_crc) {
        crc_error_count++;
        return false;
    }

    frame->seq = frame_buffer[4];
    frame->flags = frame_buffer[5];
    frame->error_x = read_i16_le(&frame_buffer[6]);
    frame->angle_cdeg = read_i16_le(&frame_buffer[8]);
    frame->confidence = read_u16_le(&frame_buffer[10]);
    frame->line_width = read_u16_le(&frame_buffer[12]);

    if ((frame->error_x < -1000) || (frame->error_x > 1000) ||
        (frame->angle_cdeg < -9000) || (frame->angle_cdeg > 9000) ||
        (frame->confidence > 1000U)) {
        format_error_count++;
        return false;
    }

    return true;
}

bool vision_line_protocol_poll(VisionLineFrame *frame)
{
    int16_t value;

    if (frame == 0) {
        return false;
    }

    while (drv_vision_uart3_available() > 0U) {
        value = drv_vision_uart3_read();
        if (value < 0) {
            break;
        }

        if (frame_index == 0U) {
            if ((uint8_t)value == VISION_SOF0) {
                frame_buffer[frame_index++] = (uint8_t)value;
            }
            continue;
        }

        if (frame_index == 1U) {
            if ((uint8_t)value == VISION_SOF1) {
                frame_buffer[frame_index++] = (uint8_t)value;
            } else if ((uint8_t)value == VISION_SOF0) {
                frame_buffer[0] = VISION_SOF0;
            } else {
                frame_index = 0U;
            }
            continue;
        }

        frame_buffer[frame_index++] = (uint8_t)value;
        if (frame_index >= VISION_LINE_FRAME_SIZE) {
            bool valid = decode_frame(frame);
            frame_index = 0U;
            if (valid) {
                return true;
            }
        }
    }
    return false;
}

uint32_t vision_line_get_crc_error_count(void)
{
    return crc_error_count;
}

uint32_t vision_line_get_format_error_count(void)
{
    return format_error_count;
}
