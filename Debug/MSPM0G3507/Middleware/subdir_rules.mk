################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
MSPM0G3507/Middleware/%.o: ../MSPM0G3507/Middleware/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Arm Compiler - building file: "$<"'
	"D:/TI2.0/CCS/ccs/tools/compiler/ti-cgt-armllvm_4.0.4.LTS/bin/tiarmclang.exe" -c @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O0 -I"D:/2025e/2025e/MSPM0G3507/BSP" -I"D:/2025e/2025e/MSPM0G3507/Application" -I"D:/2025e/2025e" -I"D:/2025e/2025e/Debug" -I"C:/TI/mspm0_sdk_2_10_00_04/source/third_party/CMSIS/Core/Include" -I"C:/TI/mspm0_sdk_2_10_00_04/source" -I"D:/2025e/2025e/MSPM0G3507/Middleware" -I"D:/2025e/2025e/MSPM0G3507/Driver" -gdwarf-3 -Wall -MMD -MP -MF"MSPM0G3507/Middleware/$(basename $(<F)).d_raw" -MT"$(@)"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


