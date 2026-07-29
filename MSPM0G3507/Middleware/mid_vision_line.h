#ifndef __MID_VISION_LINE_H
#define __MID_VISION_LINE_H

#include <stdbool.h>
#include <stdint.h>

#define VISION_LINE_FRAME_SIZE       16U
#define VISION_LINE_PROTOCOL_VERSION  1U
#define VISION_LINE_MSG_ID            1U

#define VISION_LINE_FLAG_VALID         0x01U
#define VISION_LINE_FLAG_INTERSECTION  0x02U
#define VISION_LINE_FLAG_PREDICTED     0x04U

typedef struct {
    uint8_t  seq;
    uint8_t  flags;
    int16_t  error_x;
    int16_t  angle_cdeg;
    uint16_t confidence;
    uint16_t line_width;
} VisionLineFrame;

void     vision_line_protocol_init(void);
bool     vision_line_protocol_poll(VisionLineFrame *frame);
uint16_t vision_line_crc16_ccitt(const uint8_t *data, uint16_t length);
uint32_t vision_line_get_crc_error_count(void);
uint32_t vision_line_get_format_error_count(void);

#endif /* __MID_VISION_LINE_H */
