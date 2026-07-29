#ifndef __BSP_ZDT_X35_H
#define __BSP_ZDT_X35_H

#include <stdbool.h>
#include <stdint.h>

/*
 * 张大头 X35_V1.3 + Emm_V5.0 串口协议层。
 *
 * 实机版本回包为 01 1F 05 82 6B：
 *   0x05：固件版本字段（Emm V5 系列）
 *   0x82：硬件版本字段，十进制为130，对应X35_V1.3
 *
 * 本文件只负责拼装协议帧和解析回复；是否允许电机动作由应用层决定。
 */

#define ZDT_X35_MOTOR_ADDRESS    0x01U
#define ZDT_X35_FIXED_CHECKSUM   0x6BU
#define ZDT_X35_FRAME_MAX_LENGTH 32U

typedef enum {
    ZDT_X35_RESULT_OK = 0,
    ZDT_X35_RESULT_BAD_ARGUMENT,
    ZDT_X35_RESULT_UART_ERROR,
} ZdtX35Result;

typedef enum {
    ZDT_X35_DIRECTION_CW = 0x00U,
    ZDT_X35_DIRECTION_CCW = 0x01U,
} ZdtX35Direction;

typedef enum {
    ZDT_X35_POSITION_RELATIVE = 0x00U,
    ZDT_X35_POSITION_ABSOLUTE = 0x01U,
} ZdtX35PositionMode;

typedef struct {
    uint8_t data[ZDT_X35_FRAME_MAX_LENGTH];
    uint8_t length;
    bool ready;
    bool is_version_response;
    bool is_command_response;
    uint8_t command;
    uint8_t command_status;
    uint8_t firmware_version;
    uint8_t hardware_version;
    uint32_t valid_frame_count;
    uint32_t invalid_frame_count;
} ZdtX35ProbeState;

/** 初始化UART2和协议解析器；初始化本身不会使能或转动电机。 */
void zdt_x35_init(void);

/** 发送只读版本查询：01 1F 6B。 */
ZdtX35Result zdt_x35_request_version(void);

/** 电机使能/失能；sync=true时等待多机同步触发。 */
ZdtX35Result zdt_x35_set_enable(bool enable, bool sync);

/**
 * 速度模式控制。
 * speed_rpm范围为0~3000 RPM；acc为0~255加速度档位。
 */
ZdtX35Result zdt_x35_run_velocity(
    ZdtX35Direction direction,
    uint16_t speed_rpm,
    uint8_t acc,
    bool sync);

/**
 * 位置模式控制。
 * pulse_count是驱动器接收的脉冲数，不能直接当作角度；换算取决于细分设置。
 */
ZdtX35Result zdt_x35_move_position(
    ZdtX35Direction direction,
    uint16_t speed_rpm,
    uint8_t acc,
    uint32_t pulse_count,
    ZdtX35PositionMode mode,
    bool sync);

/** 立即停止速度/位置运动；sync=true时等待多机同步触发。 */
ZdtX35Result zdt_x35_stop(bool sync);

/** 在主循环周期调用，把UART2字节拼成以0x6B结尾的原始回复帧。 */
void zdt_x35_poll(void);

/** 获取探测状态。 */
const ZdtX35ProbeState *zdt_x35_get_probe_state(void);

/** 标记当前原始回复已处理，允许保存下一帧。 */
void zdt_x35_consume_frame(void);

#endif /* __BSP_ZDT_X35_H */
