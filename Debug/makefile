################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

CG_TOOL_ROOT := E:/ccs/ccs/tools/compiler/ti-cgt-armllvm_4.0.4.LTS

GEN_OPTS__FLAG := @"./device.opt" 
GEN_CMDS__FLAG := -Wl,-l"./device_linker.cmd" 

ORDERED_OBJS += \
"./empty.o" \
"./ti_msp_dl_config.o" \
"./startup_mspm0g350x_ticlang.o" \
"./MSPM0G3507/Application/app.o" \
"./MSPM0G3507/Application/app_BLCD_test.o" \
"./MSPM0G3507/Application/app_MG310_test.o" \
"./MSPM0G3507/Application/app_PWM_test.o" \
"./MSPM0G3507/Application/app_as5600_test.o" \
"./MSPM0G3507/Application/app_button_test.o" \
"./MSPM0G3507/Application/app_debug_test.o" \
"./MSPM0G3507/Application/app_delay_test.o" \
"./MSPM0G3507/Application/app_f32c_gimbal_test.o" \
"./MSPM0G3507/Application/app_fishpath_test.o" \
"./MSPM0G3507/Application/app_motor.o" \
"./MSPM0G3507/Application/app_motor_ctrl_test.o" \
"./MSPM0G3507/BSP/bsp_AS5600.o" \
"./MSPM0G3507/BSP/bsp_BLCD.o" \
"./MSPM0G3507/BSP/bsp_OLED.o" \
"./MSPM0G3507/BSP/bsp_button.o" \
"./MSPM0G3507/BSP/bsp_encoder.o" \
"./MSPM0G3507/BSP/bsp_f32c_gimbal.o" \
"./MSPM0G3507/BSP/bsp_fishpath.o" \
"./MSPM0G3507/BSP/bsp_led.o" \
"./MSPM0G3507/BSP/bsp_motor.o" \
"./MSPM0G3507/BSP/bsp_motor_ctrl.o" \
"./MSPM0G3507/Driver/drv_IIC.o" \
"./MSPM0G3507/Driver/drv_encoder.o" \
"./MSPM0G3507/Driver/drv_tim.o" \
"./MSPM0G3507/Driver/drv_uart.o" \
"./MSPM0G3507/Middleware/mid_delay.o" \
"./MSPM0G3507/Middleware/mid_foc.o" \
"./MSPM0G3507/Middleware/mid_pid.o" \
"./MSPM0G3507/Middleware/mid_xunji.o" \
$(GEN_CMDS__FLAG) \
-Wl,-ldevice.cmd.genlibs \
-Wl,-llibc.a \

-include ../makefile.init

RM := DEL /F
RMDIR := RMDIR /S/Q

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir_vars.mk
-include MSPM0G3507/Application/subdir_vars.mk
-include MSPM0G3507/BSP/subdir_vars.mk
-include MSPM0G3507/Driver/subdir_vars.mk
-include MSPM0G3507/Middleware/subdir_vars.mk
-include subdir_rules.mk
-include MSPM0G3507/Application/subdir_rules.mk
-include MSPM0G3507/BSP/subdir_rules.mk
-include MSPM0G3507/Driver/subdir_rules.mk
-include MSPM0G3507/Middleware/subdir_rules.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C55_DEPS)),)
-include $(C55_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(S67_DEPS)),)
-include $(S67_DEPS)
endif
ifneq ($(strip $(S62_DEPS)),)
-include $(S62_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(OPT_DEPS)),)
-include $(OPT_DEPS)
endif
ifneq ($(strip $(C??_DEPS)),)
-include $(C??_DEPS)
endif
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(S??_DEPS)),)
-include $(S??_DEPS)
endif
ifneq ($(strip $(C64_DEPS)),)
-include $(C64_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(S64_DEPS)),)
-include $(S64_DEPS)
endif
ifneq ($(strip $(CLA_DEPS)),)
-include $(CLA_DEPS)
endif
ifneq ($(strip $(S55_DEPS)),)
-include $(S55_DEPS)
endif
ifneq ($(strip $(SV7A_DEPS)),)
-include $(SV7A_DEPS)
endif
ifneq ($(strip $(C62_DEPS)),)
-include $(C62_DEPS)
endif
ifneq ($(strip $(C67_DEPS)),)
-include $(C67_DEPS)
endif
ifneq ($(strip $(K_DEPS)),)
-include $(K_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C43_DEPS)),)
-include $(C43_DEPS)
endif
ifneq ($(strip $(S43_DEPS)),)
-include $(S43_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(SA_DEPS)),)
-include $(SA_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXE_OUTPUTS += \
first.out 

EXE_OUTPUTS__QUOTED += \
"first.out" 


# All Target
all: $(OBJS) $(GEN_CMDS)
	@$(MAKE) --no-print-directory -Onone "first.out"

# Tool invocations
first.out: $(OBJS) $(GEN_CMDS)
	@echo 'Arm Linker - building target: "$@"'
	"E:/ccs/ccs/tools/compiler/ti-cgt-armllvm_4.0.4.LTS/bin/tiarmclang.exe" @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O0 -gdwarf-3 -Wall -Wl,-m"first.map" -Wl,-i"C:/TI/mspm0_sdk_2_10_00_04/source" -Wl,-i"E:/ccs/project" -Wl,-i"E:/ccs/project/Debug/syscfg" -Wl,-i"E:/ccs/ccs/tools/compiler/ti-cgt-armllvm_4.0.4.LTS/lib" -Wl,--diag_wrap=off -Wl,--display_error_number -Wl,--warn_sections -Wl,--xml_link_info="first_linkInfo.xml" -Wl,--rom_model -o "first.out" $(ORDERED_OBJS)
	@echo 'Finished building target: "$@"'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(GEN_MISC_FILES__QUOTED)$(GEN_FILES__QUOTED)$(EXE_OUTPUTS__QUOTED)
	-$(RM) "empty.o" "ti_msp_dl_config.o" "startup_mspm0g350x_ticlang.o" "MSPM0G3507\Application\app.o" "MSPM0G3507\Application\app_BLCD_test.o" "MSPM0G3507\Application\app_MG310_test.o" "MSPM0G3507\Application\app_PWM_test.o" "MSPM0G3507\Application\app_as5600_test.o" "MSPM0G3507\Application\app_button_test.o" "MSPM0G3507\Application\app_debug_test.o" "MSPM0G3507\Application\app_delay_test.o" "MSPM0G3507\Application\app_f32c_gimbal_test.o" "MSPM0G3507\Application\app_fishpath_test.o" "MSPM0G3507\Application\app_motor.o" "MSPM0G3507\Application\app_motor_ctrl_test.o" "MSPM0G3507\BSP\bsp_AS5600.o" "MSPM0G3507\BSP\bsp_BLCD.o" "MSPM0G3507\BSP\bsp_OLED.o" "MSPM0G3507\BSP\bsp_button.o" "MSPM0G3507\BSP\bsp_encoder.o" "MSPM0G3507\BSP\bsp_f32c_gimbal.o" "MSPM0G3507\BSP\bsp_fishpath.o" "MSPM0G3507\BSP\bsp_led.o" "MSPM0G3507\BSP\bsp_motor.o" "MSPM0G3507\BSP\bsp_motor_ctrl.o" "MSPM0G3507\Driver\drv_IIC.o" "MSPM0G3507\Driver\drv_encoder.o" "MSPM0G3507\Driver\drv_tim.o" "MSPM0G3507\Driver\drv_uart.o" "MSPM0G3507\Middleware\mid_delay.o" 
	-$(RM) "MSPM0G3507\Middleware\mid_foc.o" "MSPM0G3507\Middleware\mid_pid.o" "MSPM0G3507\Middleware\mid_xunji.o" 
	-$(RM) "empty.d" "ti_msp_dl_config.d" "startup_mspm0g350x_ticlang.d" "MSPM0G3507\Application\app.d" "MSPM0G3507\Application\app_BLCD_test.d" "MSPM0G3507\Application\app_MG310_test.d" "MSPM0G3507\Application\app_PWM_test.d" "MSPM0G3507\Application\app_as5600_test.d" "MSPM0G3507\Application\app_button_test.d" "MSPM0G3507\Application\app_debug_test.d" "MSPM0G3507\Application\app_delay_test.d" "MSPM0G3507\Application\app_f32c_gimbal_test.d" "MSPM0G3507\Application\app_fishpath_test.d" "MSPM0G3507\Application\app_motor.d" "MSPM0G3507\Application\app_motor_ctrl_test.d" "MSPM0G3507\BSP\bsp_AS5600.d" "MSPM0G3507\BSP\bsp_BLCD.d" "MSPM0G3507\BSP\bsp_OLED.d" "MSPM0G3507\BSP\bsp_button.d" "MSPM0G3507\BSP\bsp_encoder.d" "MSPM0G3507\BSP\bsp_f32c_gimbal.d" "MSPM0G3507\BSP\bsp_fishpath.d" "MSPM0G3507\BSP\bsp_led.d" "MSPM0G3507\BSP\bsp_motor.d" "MSPM0G3507\BSP\bsp_motor_ctrl.d" "MSPM0G3507\Driver\drv_IIC.d" "MSPM0G3507\Driver\drv_encoder.d" "MSPM0G3507\Driver\drv_tim.d" "MSPM0G3507\Driver\drv_uart.d" "MSPM0G3507\Middleware\mid_delay.d" 
	-$(RM) "MSPM0G3507\Middleware\mid_foc.d" "MSPM0G3507\Middleware\mid_pid.d" "MSPM0G3507\Middleware\mid_xunji.d" 
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

