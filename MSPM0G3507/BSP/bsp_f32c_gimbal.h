#ifndef __BSP_F32C_GIMBAL_H
#define __BSP_F32C_GIMBAL_H

#include <stdbool.h>
#include <stdint.h>

#define F32C_GIMBAL_X_ID  1U
#define F32C_GIMBAL_Y_ID  2U

/** 初始化 UART3 底层驱动。硬件配置来自 ti_msp_dl_config.h。 */
void bsp_f32c_gimbal_init(void);

/** 使能/失能 X、Y 两个电机。 */
void bsp_f32c_gimbal_enable(void);
void bsp_f32c_gimbal_disable(void);

/** 将 X、Y 电机均切换为单圈绝对位置模式（带 T 型轨迹规划，模式 0x0002）。 */
void bsp_f32c_gimbal_set_single_turn_mode(void);

/**
 * @brief 设置两个轴的位置模式转速。
 * @param rpm 1~1000 RPM。
 * @return 参数有效返回 true。
 */
bool bsp_f32c_gimbal_set_speed(uint16_t rpm);

/**
 * @brief 设置二维云台单圈绝对角度。
 * @param x_degree X 轴目标角度，范围 0.0~359.9°。
 * @param y_degree Y 轴目标角度，范围 0.0~359.9°。
 * @return 两个角度均有效返回 true；参数无效时不发送任何角度命令。
 */
bool bsp_f32c_gimbal_set_angle(float x_degree, float y_degree);

/** 以下单电机接口便于独立调试。 */
void bsp_f32c_motor_enable(uint8_t address);
void bsp_f32c_motor_disable(uint8_t address);
void bsp_f32c_motor_set_single_turn_mode(uint8_t address);
bool bsp_f32c_motor_set_speed(uint8_t address, uint16_t rpm);
bool bsp_f32c_motor_set_single_turn_angle(uint8_t address, float degree);

#endif /* __BSP_F32C_GIMBAL_H */
