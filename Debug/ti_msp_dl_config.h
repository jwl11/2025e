/*
 * Copyright (c) 2023, Texas Instruments Incorporated - http://www.ti.com
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

/*
 *  ============ ti_msp_dl_config.h =============
 *  Configured MSPM0 DriverLib module declarations
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G350X
 *  by the SysConfig tool.
 */
#ifndef ti_msp_dl_config_h
#define ti_msp_dl_config_h

#define CONFIG_MSPM0G350X
#define CONFIG_MSPM0G3507

#if defined(__ti_version__) || defined(__TI_COMPILER_VERSION__)
#define SYSCONFIG_WEAK __attribute__((weak))
#elif defined(__IAR_SYSTEMS_ICC__)
#define SYSCONFIG_WEAK __weak
#elif defined(__GNUC__)
#define SYSCONFIG_WEAK __attribute__((weak))
#endif

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform all required MSP DL initialization
 *
 *  This function should be called once at a point before any use of
 *  MSP DL.
 */


/* clang-format off */

#define POWER_STARTUP_DELAY                                                (16)


#define CPUCLK_FREQ                                                     32000000



/* Defines for BLDC */
#define BLDC_INST                                                          TIMA0
#define BLDC_INST_IRQHandler                                    TIMA0_IRQHandler
#define BLDC_INST_INT_IRQN                                      (TIMA0_INT_IRQn)
#define BLDC_INST_CLK_FREQ                                              32000000
/* GPIO defines for channel 0 */
#define GPIO_BLDC_C0_PORT                                                  GPIOA
#define GPIO_BLDC_C0_PIN                                           DL_GPIO_PIN_0
#define GPIO_BLDC_C0_IOMUX                                        (IOMUX_PINCM1)
#define GPIO_BLDC_C0_IOMUX_FUNC                       IOMUX_PINCM1_PF_TIMA0_CCP0
#define GPIO_BLDC_C0_IDX                                     DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_BLDC_C1_PORT                                                  GPIOB
#define GPIO_BLDC_C1_PIN                                          DL_GPIO_PIN_20
#define GPIO_BLDC_C1_IOMUX                                       (IOMUX_PINCM48)
#define GPIO_BLDC_C1_IOMUX_FUNC                      IOMUX_PINCM48_PF_TIMA0_CCP1
#define GPIO_BLDC_C1_IDX                                     DL_TIMER_CC_1_INDEX
/* GPIO defines for channel 2 */
#define GPIO_BLDC_C2_PORT                                                  GPIOB
#define GPIO_BLDC_C2_PIN                                          DL_GPIO_PIN_12
#define GPIO_BLDC_C2_IOMUX                                       (IOMUX_PINCM29)
#define GPIO_BLDC_C2_IOMUX_FUNC                      IOMUX_PINCM29_PF_TIMA0_CCP2
#define GPIO_BLDC_C2_IDX                                     DL_TIMER_CC_2_INDEX

/* Defines for MG310_PWM */
#define MG310_PWM_INST                                                     TIMG0
#define MG310_PWM_INST_IRQHandler                               TIMG0_IRQHandler
#define MG310_PWM_INST_INT_IRQN                                 (TIMG0_INT_IRQn)
#define MG310_PWM_INST_CLK_FREQ                                         32000000
/* GPIO defines for channel 0 */
#define GPIO_MG310_PWM_C0_PORT                                             GPIOA
#define GPIO_MG310_PWM_C0_PIN                                     DL_GPIO_PIN_12
#define GPIO_MG310_PWM_C0_IOMUX                                  (IOMUX_PINCM34)
#define GPIO_MG310_PWM_C0_IOMUX_FUNC                 IOMUX_PINCM34_PF_TIMG0_CCP0
#define GPIO_MG310_PWM_C0_IDX                                DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_MG310_PWM_C1_PORT                                             GPIOA
#define GPIO_MG310_PWM_C1_PIN                                     DL_GPIO_PIN_13
#define GPIO_MG310_PWM_C1_IOMUX                                  (IOMUX_PINCM35)
#define GPIO_MG310_PWM_C1_IOMUX_FUNC                 IOMUX_PINCM35_PF_TIMG0_CCP1
#define GPIO_MG310_PWM_C1_IDX                                DL_TIMER_CC_1_INDEX




/* Defines for as5600 */
#define as5600_INST                                                         I2C1
#define as5600_INST_IRQHandler                                   I2C1_IRQHandler
#define as5600_INST_INT_IRQN                                       I2C1_INT_IRQn
#define as5600_BUS_SPEED_HZ                                               400000
#define GPIO_as5600_SDA_PORT                                               GPIOA
#define GPIO_as5600_SDA_PIN                                       DL_GPIO_PIN_18
#define GPIO_as5600_IOMUX_SDA                                    (IOMUX_PINCM40)
#define GPIO_as5600_IOMUX_SDA_FUNC                     IOMUX_PINCM40_PF_I2C1_SDA
#define GPIO_as5600_SCL_PORT                                               GPIOA
#define GPIO_as5600_SCL_PIN                                       DL_GPIO_PIN_17
#define GPIO_as5600_IOMUX_SCL                                    (IOMUX_PINCM39)
#define GPIO_as5600_IOMUX_SCL_FUNC                     IOMUX_PINCM39_PF_I2C1_SCL


/* Defines for debug */
#define debug_INST                                                         UART0
#define debug_INST_FREQUENCY                                            32000000
#define debug_INST_IRQHandler                                   UART0_IRQHandler
#define debug_INST_INT_IRQN                                       UART0_INT_IRQn
#define GPIO_debug_RX_PORT                                                 GPIOA
#define GPIO_debug_TX_PORT                                                 GPIOA
#define GPIO_debug_RX_PIN                                         DL_GPIO_PIN_11
#define GPIO_debug_TX_PIN                                         DL_GPIO_PIN_10
#define GPIO_debug_IOMUX_RX                                      (IOMUX_PINCM22)
#define GPIO_debug_IOMUX_TX                                      (IOMUX_PINCM21)
#define GPIO_debug_IOMUX_RX_FUNC                       IOMUX_PINCM22_PF_UART0_RX
#define GPIO_debug_IOMUX_TX_FUNC                       IOMUX_PINCM21_PF_UART0_TX
#define debug_BAUD_RATE                                                 (115200)
#define debug_IBRD_32_MHZ_115200_BAUD                                       (17)
#define debug_FBRD_32_MHZ_115200_BAUD                                       (23)
/* Defines for fishpath */
#define fishpath_INST                                                      UART1
#define fishpath_INST_FREQUENCY                                         32000000
#define fishpath_INST_IRQHandler                                UART1_IRQHandler
#define fishpath_INST_INT_IRQN                                    UART1_INT_IRQn
#define GPIO_fishpath_RX_PORT                                              GPIOB
#define GPIO_fishpath_TX_PORT                                              GPIOB
#define GPIO_fishpath_RX_PIN                                       DL_GPIO_PIN_7
#define GPIO_fishpath_TX_PIN                                       DL_GPIO_PIN_6
#define GPIO_fishpath_IOMUX_RX                                   (IOMUX_PINCM24)
#define GPIO_fishpath_IOMUX_TX                                   (IOMUX_PINCM23)
#define GPIO_fishpath_IOMUX_RX_FUNC                    IOMUX_PINCM24_PF_UART1_RX
#define GPIO_fishpath_IOMUX_TX_FUNC                    IOMUX_PINCM23_PF_UART1_TX
#define fishpath_BAUD_RATE                                              (115200)
#define fishpath_IBRD_32_MHZ_115200_BAUD                                    (17)
#define fishpath_FBRD_32_MHZ_115200_BAUD                                    (23)





/* Port definition for Pin Group use_led */
#define use_led_PORT                                                     (GPIOB)

/* Defines for PIN_22: GPIOB.22 with pinCMx 50 on package pin 21 */
#define use_led_PIN_22_PIN                                      (DL_GPIO_PIN_22)
#define use_led_PIN_22_IOMUX                                     (IOMUX_PINCM50)
/* Port definition for Pin Group KEY */
#define KEY_PORT                                                         (GPIOB)

/* Defines for KEY1: GPIOB.15 with pinCMx 32 on package pin 3 */
#define KEY_KEY1_PIN                                            (DL_GPIO_PIN_15)
#define KEY_KEY1_IOMUX                                           (IOMUX_PINCM32)
/* Defines for AIN1: GPIOB.17 with pinCMx 43 on package pin 14 */
#define MG310_AIN1_PORT                                                  (GPIOB)
#define MG310_AIN1_PIN                                          (DL_GPIO_PIN_17)
#define MG310_AIN1_IOMUX                                         (IOMUX_PINCM43)
/* Defines for AIN2: GPIOB.19 with pinCMx 45 on package pin 16 */
#define MG310_AIN2_PORT                                                  (GPIOB)
#define MG310_AIN2_PIN                                          (DL_GPIO_PIN_19)
#define MG310_AIN2_IOMUX                                         (IOMUX_PINCM45)
/* Defines for BIN1: GPIOA.16 with pinCMx 38 on package pin 9 */
#define MG310_BIN1_PORT                                                  (GPIOA)
#define MG310_BIN1_PIN                                          (DL_GPIO_PIN_16)
#define MG310_BIN1_IOMUX                                         (IOMUX_PINCM38)
/* Defines for BIN2: GPIOB.24 with pinCMx 52 on package pin 23 */
#define MG310_BIN2_PORT                                                  (GPIOB)
#define MG310_BIN2_PIN                                          (DL_GPIO_PIN_24)
#define MG310_BIN2_IOMUX                                         (IOMUX_PINCM52)
/* Port definition for Pin Group OLED */
#define OLED_PORT                                                        (GPIOB)

/* Defines for OLED_SCL: GPIOB.9 with pinCMx 26 on package pin 61 */
#define OLED_OLED_SCL_PIN                                        (DL_GPIO_PIN_9)
#define OLED_OLED_SCL_IOMUX                                      (IOMUX_PINCM26)
/* Defines for OLED_SDA: GPIOB.8 with pinCMx 25 on package pin 60 */
#define OLED_OLED_SDA_PIN                                        (DL_GPIO_PIN_8)
#define OLED_OLED_SDA_IOMUX                                      (IOMUX_PINCM25)




/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_BLDC_init(void);
void SYSCFG_DL_MG310_PWM_init(void);
void SYSCFG_DL_as5600_init(void);
void SYSCFG_DL_debug_init(void);
void SYSCFG_DL_fishpath_init(void);

void SYSCFG_DL_SYSTICK_init(void);

bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
