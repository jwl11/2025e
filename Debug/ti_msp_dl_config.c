/*
 * Copyright (c) 2023, Texas Instruments Incorporated
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
 *  ============ ti_msp_dl_config.c =============
 *  Configured MSPM0 DriverLib module definitions
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G350X
 *  by the SysConfig tool.
 */

#include "ti_msp_dl_config.h"

DL_TimerA_backupConfig gBLDCBackup;
DL_TimerA_backupConfig gMG310_PWMBackup;

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform any initialization needed before using any board APIs
 */
SYSCONFIG_WEAK void SYSCFG_DL_init(void)
{
    SYSCFG_DL_initPower();
    SYSCFG_DL_GPIO_init();
    /* Module-Specific Initializations*/
    SYSCFG_DL_SYSCTL_init();
    SYSCFG_DL_BLDC_init();
    SYSCFG_DL_MG310_PWM_init();
    SYSCFG_DL_as5600_init();
    SYSCFG_DL_debug_init();
    SYSCFG_DL_fishpath_init();
    SYSCFG_DL_SYSTICK_init();
    /* Ensure backup structures have no valid state */
	gBLDCBackup.backupRdy 	= false;
	gMG310_PWMBackup.backupRdy 	= false;


}
/*
 * User should take care to save and restore register configuration in application.
 * See Retention Configuration section for more details.
 */
SYSCONFIG_WEAK bool SYSCFG_DL_saveConfiguration(void)
{
    bool retStatus = true;

	retStatus &= DL_TimerA_saveConfiguration(BLDC_INST, &gBLDCBackup);
	retStatus &= DL_TimerA_saveConfiguration(MG310_PWM_INST, &gMG310_PWMBackup);

    return retStatus;
}


SYSCONFIG_WEAK bool SYSCFG_DL_restoreConfiguration(void)
{
    bool retStatus = true;

	retStatus &= DL_TimerA_restoreConfiguration(BLDC_INST, &gBLDCBackup, false);
	retStatus &= DL_TimerA_restoreConfiguration(MG310_PWM_INST, &gMG310_PWMBackup, false);

    return retStatus;
}

SYSCONFIG_WEAK void SYSCFG_DL_initPower(void)
{
    DL_GPIO_reset(GPIOA);
    DL_GPIO_reset(GPIOB);
    DL_TimerA_reset(BLDC_INST);
    DL_TimerA_reset(MG310_PWM_INST);
    DL_I2C_reset(as5600_INST);
    DL_UART_Main_reset(debug_INST);
    DL_UART_Main_reset(fishpath_INST);


    DL_GPIO_enablePower(GPIOA);
    DL_GPIO_enablePower(GPIOB);
    DL_TimerA_enablePower(BLDC_INST);
    DL_TimerA_enablePower(MG310_PWM_INST);
    DL_I2C_enablePower(as5600_INST);
    DL_UART_Main_enablePower(debug_INST);
    DL_UART_Main_enablePower(fishpath_INST);

    delay_cycles(POWER_STARTUP_DELAY);
}

SYSCONFIG_WEAK void SYSCFG_DL_GPIO_init(void)
{

    DL_GPIO_initPeripheralOutputFunction(GPIO_BLDC_C0_IOMUX,GPIO_BLDC_C0_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_BLDC_C0_PORT, GPIO_BLDC_C0_PIN);
    DL_GPIO_initPeripheralOutputFunction(GPIO_BLDC_C1_IOMUX,GPIO_BLDC_C1_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_BLDC_C1_PORT, GPIO_BLDC_C1_PIN);
    DL_GPIO_initPeripheralOutputFunction(GPIO_BLDC_C2_IOMUX,GPIO_BLDC_C2_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_BLDC_C2_PORT, GPIO_BLDC_C2_PIN);
    DL_GPIO_initPeripheralOutputFunction(GPIO_MG310_PWM_C0_IOMUX,GPIO_MG310_PWM_C0_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_MG310_PWM_C0_PORT, GPIO_MG310_PWM_C0_PIN);
    DL_GPIO_initPeripheralOutputFunction(GPIO_MG310_PWM_C1_IOMUX,GPIO_MG310_PWM_C1_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_MG310_PWM_C1_PORT, GPIO_MG310_PWM_C1_PIN);

    DL_GPIO_initPeripheralInputFunctionFeatures(GPIO_as5600_IOMUX_SDA,
        GPIO_as5600_IOMUX_SDA_FUNC, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initPeripheralInputFunctionFeatures(GPIO_as5600_IOMUX_SCL,
        GPIO_as5600_IOMUX_SCL_FUNC, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_enableHiZ(GPIO_as5600_IOMUX_SDA);
    DL_GPIO_enableHiZ(GPIO_as5600_IOMUX_SCL);

    DL_GPIO_initPeripheralOutputFunction(
        GPIO_debug_IOMUX_TX, GPIO_debug_IOMUX_TX_FUNC);
    DL_GPIO_initPeripheralInputFunction(
        GPIO_debug_IOMUX_RX, GPIO_debug_IOMUX_RX_FUNC);
    DL_GPIO_initPeripheralOutputFunction(
        GPIO_fishpath_IOMUX_TX, GPIO_fishpath_IOMUX_TX_FUNC);
    DL_GPIO_initPeripheralInputFunction(
        GPIO_fishpath_IOMUX_RX, GPIO_fishpath_IOMUX_RX_FUNC);

    DL_GPIO_initDigitalOutput(use_led_PIN_22_IOMUX);

    DL_GPIO_initDigitalInputFeatures(KEY_KEY1_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalOutput(MG310_AIN1_IOMUX);

    DL_GPIO_initDigitalOutput(MG310_AIN2_IOMUX);

    DL_GPIO_initDigitalOutput(MG310_BIN1_IOMUX);

    DL_GPIO_initDigitalOutput(MG310_BIN2_IOMUX);

    DL_GPIO_initDigitalOutput(OLED_OLED_SCL_IOMUX);

    DL_GPIO_initDigitalOutput(OLED_OLED_SDA_IOMUX);

    DL_GPIO_clearPins(GPIOA, MG310_AIN1_PIN |
		MG310_AIN2_PIN);
    DL_GPIO_enableOutput(GPIOA, MG310_AIN1_PIN |
		MG310_AIN2_PIN);
    DL_GPIO_clearPins(GPIOB, use_led_PIN_22_PIN |
		MG310_BIN1_PIN |
		MG310_BIN2_PIN);
    DL_GPIO_setPins(GPIOB, OLED_OLED_SCL_PIN |
		OLED_OLED_SDA_PIN);
    DL_GPIO_enableOutput(GPIOB, use_led_PIN_22_PIN |
		MG310_BIN1_PIN |
		MG310_BIN2_PIN |
		OLED_OLED_SCL_PIN |
		OLED_OLED_SDA_PIN);

}


SYSCONFIG_WEAK void SYSCFG_DL_SYSCTL_init(void)
{

	//Low Power Mode is configured to be SLEEP0
    DL_SYSCTL_setBORThreshold(DL_SYSCTL_BOR_THRESHOLD_LEVEL_0);

    DL_SYSCTL_setSYSOSCFreq(DL_SYSCTL_SYSOSC_FREQ_BASE);
    /* Set default configuration */
    DL_SYSCTL_disableHFXT();
    DL_SYSCTL_disableSYSPLL();
    DL_SYSCTL_setULPCLKDivider(DL_SYSCTL_ULPCLK_DIV_1);
    DL_SYSCTL_setMCLKDivider(DL_SYSCTL_MCLK_DIVIDER_DISABLE);

}


/*
 * Timer clock configuration to be sourced by  / 1 (32000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   32000000 Hz = 32000000 Hz / (1 * (0 + 1))
 */
static const DL_TimerA_ClockConfig gBLDCClockConfig = {
    .clockSel = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
    .prescale = 0U
};

static const DL_TimerA_PWMConfig gBLDCConfig = {
    .pwmMode = DL_TIMER_PWM_MODE_EDGE_ALIGN,
    .period = 1066,
    .isTimerWithFourCC = true,
    .startTimer = DL_TIMER_STOP,
};

SYSCONFIG_WEAK void SYSCFG_DL_BLDC_init(void) {

    DL_TimerA_setClockConfig(
        BLDC_INST, (DL_TimerA_ClockConfig *) &gBLDCClockConfig);

    DL_TimerA_initPWMMode(
        BLDC_INST, (DL_TimerA_PWMConfig *) &gBLDCConfig);

    // Set Counter control to the smallest CC index being used
    DL_TimerA_setCounterControl(BLDC_INST,DL_TIMER_CZC_CCCTL0_ZCOND,DL_TIMER_CAC_CCCTL0_ACOND,DL_TIMER_CLC_CCCTL0_LCOND);

    DL_TimerA_setCaptureCompareOutCtl(BLDC_INST, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
		DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
		DL_TIMERA_CAPTURE_COMPARE_0_INDEX);

    DL_TimerA_setCaptCompUpdateMethod(BLDC_INST, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERA_CAPTURE_COMPARE_0_INDEX);
    DL_TimerA_setCaptureCompareValue(BLDC_INST, 1066, DL_TIMER_CC_0_INDEX);

    DL_TimerA_setCaptureCompareOutCtl(BLDC_INST, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
		DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
		DL_TIMERA_CAPTURE_COMPARE_1_INDEX);

    DL_TimerA_setCaptCompUpdateMethod(BLDC_INST, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERA_CAPTURE_COMPARE_1_INDEX);
    DL_TimerA_setCaptureCompareValue(BLDC_INST, 1066, DL_TIMER_CC_1_INDEX);

    DL_TimerA_setCaptureCompareOutCtl(BLDC_INST, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
		DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
		DL_TIMERA_CAPTURE_COMPARE_2_INDEX);

    DL_TimerA_setCaptCompUpdateMethod(BLDC_INST, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERA_CAPTURE_COMPARE_2_INDEX);
    DL_TimerA_setCaptureCompareValue(BLDC_INST, 1066, DL_TIMER_CC_2_INDEX);

    DL_TimerA_enableClock(BLDC_INST);


    
    DL_TimerA_setCCPDirection(BLDC_INST , DL_TIMER_CC0_OUTPUT | DL_TIMER_CC1_OUTPUT | DL_TIMER_CC2_OUTPUT );


}
/*
 * Timer clock configuration to be sourced by  / 1 (32000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   32000000 Hz = 32000000 Hz / (1 * (0 + 1))
 */
static const DL_TimerA_ClockConfig gMG310_PWMClockConfig = {
    .clockSel = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
    .prescale = 0U
};

static const DL_TimerA_PWMConfig gMG310_PWMConfig = {
    .pwmMode = DL_TIMER_PWM_MODE_EDGE_ALIGN_UP,
    .period = 3200,
    .isTimerWithFourCC = false,
    .startTimer = DL_TIMER_STOP,
};

SYSCONFIG_WEAK void SYSCFG_DL_MG310_PWM_init(void) {

    DL_TimerA_setClockConfig(
        MG310_PWM_INST, (DL_TimerA_ClockConfig *) &gMG310_PWMClockConfig);

    DL_TimerA_initPWMMode(
        MG310_PWM_INST, (DL_TimerA_PWMConfig *) &gMG310_PWMConfig);

    // Set Counter control to the smallest CC index being used
    DL_TimerA_setCounterControl(MG310_PWM_INST,DL_TIMER_CZC_CCCTL0_ZCOND,DL_TIMER_CAC_CCCTL0_ACOND,DL_TIMER_CLC_CCCTL0_LCOND);

    DL_TimerA_setCaptureCompareOutCtl(MG310_PWM_INST, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
		DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
		DL_TIMERA_CAPTURE_COMPARE_0_INDEX);

    DL_TimerA_setCaptCompUpdateMethod(MG310_PWM_INST, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERA_CAPTURE_COMPARE_0_INDEX);
    DL_TimerA_setCaptureCompareValue(MG310_PWM_INST, 0, DL_TIMER_CC_0_INDEX);

    DL_TimerA_setCaptureCompareOutCtl(MG310_PWM_INST, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
		DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
		DL_TIMERA_CAPTURE_COMPARE_1_INDEX);

    DL_TimerA_setCaptCompUpdateMethod(MG310_PWM_INST, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERA_CAPTURE_COMPARE_1_INDEX);
    DL_TimerA_setCaptureCompareValue(MG310_PWM_INST, 0, DL_TIMER_CC_1_INDEX);

    DL_TimerA_enableClock(MG310_PWM_INST);


    
    DL_TimerA_setCCPDirection(MG310_PWM_INST , DL_TIMER_CC0_OUTPUT | DL_TIMER_CC1_OUTPUT );


}


static const DL_I2C_ClockConfig gas5600ClockConfig = {
    .clockSel = DL_I2C_CLOCK_BUSCLK,
    .divideRatio = DL_I2C_CLOCK_DIVIDE_1,
};

SYSCONFIG_WEAK void SYSCFG_DL_as5600_init(void) {

    DL_I2C_setClockConfig(as5600_INST,
        (DL_I2C_ClockConfig *) &gas5600ClockConfig);
    DL_I2C_setAnalogGlitchFilterPulseWidth(as5600_INST,
        DL_I2C_ANALOG_GLITCH_FILTER_WIDTH_50NS);
    DL_I2C_enableAnalogGlitchFilter(as5600_INST);

    /* Configure Controller Mode */
    DL_I2C_resetControllerTransfer(as5600_INST);
    /* Set frequency to 400000 Hz*/
    DL_I2C_setTimerPeriod(as5600_INST, 7);
    DL_I2C_setControllerTXFIFOThreshold(as5600_INST, DL_I2C_TX_FIFO_LEVEL_EMPTY);
    DL_I2C_setControllerRXFIFOThreshold(as5600_INST, DL_I2C_RX_FIFO_LEVEL_BYTES_1);
    DL_I2C_enableControllerClockStretching(as5600_INST);


    /* Enable module */
    DL_I2C_enableController(as5600_INST);


}

static const DL_UART_Main_ClockConfig gdebugClockConfig = {
    .clockSel    = DL_UART_MAIN_CLOCK_BUSCLK,
    .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1
};

static const DL_UART_Main_Config gdebugConfig = {
    .mode        = DL_UART_MAIN_MODE_NORMAL,
    .direction   = DL_UART_MAIN_DIRECTION_TX_RX,
    .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
    .parity      = DL_UART_MAIN_PARITY_NONE,
    .wordLength  = DL_UART_MAIN_WORD_LENGTH_8_BITS,
    .stopBits    = DL_UART_MAIN_STOP_BITS_ONE
};

SYSCONFIG_WEAK void SYSCFG_DL_debug_init(void)
{
    DL_UART_Main_setClockConfig(debug_INST, (DL_UART_Main_ClockConfig *) &gdebugClockConfig);

    DL_UART_Main_init(debug_INST, (DL_UART_Main_Config *) &gdebugConfig);
    /*
     * Configure baud rate by setting oversampling and baud rate divisors.
     *  Target baud rate: 115200
     *  Actual baud rate: 115211.52
     */
    DL_UART_Main_setOversampling(debug_INST, DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(debug_INST, debug_IBRD_32_MHZ_115200_BAUD, debug_FBRD_32_MHZ_115200_BAUD);


    /* Configure Interrupts */
    DL_UART_Main_enableInterrupt(debug_INST,
                                 DL_UART_MAIN_INTERRUPT_RX);
    /* Setting the Interrupt Priority */
    NVIC_SetPriority(debug_INST_INT_IRQN, 2);


    DL_UART_Main_enable(debug_INST);
}
static const DL_UART_Main_ClockConfig gfishpathClockConfig = {
    .clockSel    = DL_UART_MAIN_CLOCK_BUSCLK,
    .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1
};

static const DL_UART_Main_Config gfishpathConfig = {
    .mode        = DL_UART_MAIN_MODE_NORMAL,
    .direction   = DL_UART_MAIN_DIRECTION_TX_RX,
    .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
    .parity      = DL_UART_MAIN_PARITY_NONE,
    .wordLength  = DL_UART_MAIN_WORD_LENGTH_8_BITS,
    .stopBits    = DL_UART_MAIN_STOP_BITS_ONE
};

SYSCONFIG_WEAK void SYSCFG_DL_fishpath_init(void)
{
    DL_UART_Main_setClockConfig(fishpath_INST, (DL_UART_Main_ClockConfig *) &gfishpathClockConfig);

    DL_UART_Main_init(fishpath_INST, (DL_UART_Main_Config *) &gfishpathConfig);
    /*
     * Configure baud rate by setting oversampling and baud rate divisors.
     *  Target baud rate: 115200
     *  Actual baud rate: 115211.52
     */
    DL_UART_Main_setOversampling(fishpath_INST, DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(fishpath_INST, fishpath_IBRD_32_MHZ_115200_BAUD, fishpath_FBRD_32_MHZ_115200_BAUD);



    DL_UART_Main_enable(fishpath_INST);
}

SYSCONFIG_WEAK void SYSCFG_DL_SYSTICK_init(void)
{
    /* Initialize the period to 1.00 ms */
    DL_SYSTICK_init(32000);
}

