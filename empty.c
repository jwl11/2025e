/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "ti_msp_dl_config.h"
#include "mid_delay.h"
#include "bsp_led.h"
#include "bsp_OLED.h"
#include "bsp_menu.h"
#include "app.h"

int main(void)
{
    SYSCFG_DL_init();
    __enable_irq(); //打开所有中断

    // OLED_Init();
    // menu0();

    //app_pwm_test();

    //app_BLCD_test();
    //app_as5600_test();
    //app_MG310_test();
    //app_fishpath_test();
    //app_vision_line_test();
    //app_button_test();
    //app_motor_ctrl_test();
    //app_motor_position_test();
    //app_zdt_x35_motion_test();
    //MPU6050_straight_test();
    //app_servo_test();
    //app_zdt_x35_position_test();
     //topic3();
   // app_zdt_x35_motion_test();
    //app_RX_maixcam_test();
   // topic();
    topic4();
    //app_42step_test();
    //app_mpu6050_accel_test();

    while (1) {
        
        //app_delay_test();
        //app_debug_test();
        

    }
}



