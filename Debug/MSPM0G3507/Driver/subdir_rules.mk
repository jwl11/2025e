################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
MSPM0G3507/Driver/%.o: ../MSPM0G3507/Driver/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Arm Compiler - building file: "$<"'
	"E:/ccs/ccs/tools/compiler/ti-cgt-armllvm_4.0.4.LTS/bin/tiarmclang.exe" -c @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O0 -I"E:/ccs/project/first/MSPM0G3507/BSP" -I"E:/ccs/project/first/MSPM0G3507/Application" -I"E:/ccs/project/first" -I"E:/ccs/project/first/Debug" -I"C:/TI/mspm0_sdk_2_10_00_04/source/third_party/CMSIS/Core/Include" -I"C:/TI/mspm0_sdk_2_10_00_04/source" -I"E:/ccs/project/first/MSPM0G3507/Middleware" -I"E:/ccs/project/first/MSPM0G3507/Driver" -gdwarf-3 -Wall -MMD -MP -MF"MSPM0G3507/Driver/$(basename $(<F)).d_raw" -MT"$(@)"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


