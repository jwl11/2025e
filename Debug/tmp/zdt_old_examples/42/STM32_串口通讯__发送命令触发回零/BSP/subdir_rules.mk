################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
tmp/zdt_old_examples/42/STM32_串口通讯__发送命令触发回零/BSP/%.o: ../tmp/zdt_old_examples/42/STM32_串口通讯__发送命令触发回零/BSP/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Arm Compiler: "$<"'
	"D:/TI/ccs2050/ccs/tools/compiler/ti-cgt-armllvm_4.0.4.LTS/bin/tiarmclang.exe" -c @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O0 -I"D:/2026电赛/2026H/MSPM0G3507/BSP" -I"D:/2026电赛/2026H/MSPM0G3507/Application" -I"D:/2026电赛/2026H" -I"D:/2026电赛/2026H/Debug" -I"C:/ti/mspm0_sdk_2_10_00_04/source/third_party/CMSIS/Core/Include" -I"C:/ti/mspm0_sdk_2_10_00_04/source" -I"D:/2026电赛/2026H/MSPM0G3507/Middleware" -I"D:/2026电赛/2026H/MSPM0G3507/Driver" -gdwarf-3 -Wall -MMD -MP -MF"tmp/zdt_old_examples/42/STM32_串口通讯__发送命令触发回零/BSP/$(basename $(<F)).d_raw" -MT"$(@)"  $(GEN_OPTS__FLAG) -o"$@" "$<"


