################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BSP/Components/lps22hh/lps22hh.c \
../Drivers/BSP/Components/lps22hh/lps22hh_reg.c 

OBJS += \
./Drivers/BSP/Components/lps22hh/lps22hh.o \
./Drivers/BSP/Components/lps22hh/lps22hh_reg.o 

C_DEPS += \
./Drivers/BSP/Components/lps22hh/lps22hh.d \
./Drivers/BSP/Components/lps22hh/lps22hh_reg.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/Components/lps22hh/%.o: ../Drivers/BSP/Components/lps22hh/%.c Drivers/BSP/Components/lps22hh/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32WB55xx -c -I../Core/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WBxx/Include -I../Drivers/CMSIS/Include -I../X-CUBE-MEMS1/Target -I../Drivers/BSP/Components/lsm6dso -I../Drivers/BSP/Components/lis2dw12 -I../Drivers/BSP/Components/lis2mdl -I../Drivers/BSP/Components/hts221 -I../Drivers/BSP/Components/lps22hh -I../Drivers/BSP/Components/stts751 -I../Drivers/BSP/IKS01A3 -I../Drivers/BSP/Components/Common -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-BSP-2f-Components-2f-lps22hh

clean-Drivers-2f-BSP-2f-Components-2f-lps22hh:
	-$(RM) ./Drivers/BSP/Components/lps22hh/lps22hh.d ./Drivers/BSP/Components/lps22hh/lps22hh.o ./Drivers/BSP/Components/lps22hh/lps22hh_reg.d ./Drivers/BSP/Components/lps22hh/lps22hh_reg.o

.PHONY: clean-Drivers-2f-BSP-2f-Components-2f-lps22hh

