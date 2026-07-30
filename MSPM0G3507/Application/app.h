#ifndef __APP_H
#define __APP_H


#include "ti_msp_dl_config.h"

void app_delay_test(void);
void app_debug_test(void);
void app_pwm_test(void);
void app_as5600_test(void);
void app_BLCD_test(void);
void app_MG310_test(void);
void app_fishpath_test(void);
void app_fishpath_motor_test(void);
void app_button_test(void);
void app_vision_line_test(void);
void app_motor_ctrl_test(void);
void app_motor_position_test(void);
void app_zdt_x35_probe(void);
void app_zdt_x35_motion_test(void);
void app_zdt_x35_position_test(void);
bool app_zdt_x35_move_down(uint16_t speed_rpm, uint32_t pulse_count);
bool app_zdt_x35_move_up(uint16_t speed_rpm, uint32_t pulse_count);
void MPU6050_test(void);
void MPU6050_straight_test(void);
void app_servo_test(void);


/***
 * 6道题目
 * 
 * ****/
void topic1(void);
void topic2(void);
void topic3(void);
void topic4(void);
void topic5(void);
void topic6(void);






#endif // __APP_H
