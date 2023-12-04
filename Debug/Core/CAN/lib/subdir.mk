################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/CAN/lib/can1.c \
../Core/CAN/lib/can2.c \
../Core/CAN/lib/flash.c 

OBJS += \
./Core/CAN/lib/can1.o \
./Core/CAN/lib/can2.o \
./Core/CAN/lib/flash.o 

C_DEPS += \
./Core/CAN/lib/can1.d \
./Core/CAN/lib/can2.d \
./Core/CAN/lib/flash.d 


# Each subdirectory must supply rules for building sources it contributes
Core/CAN/lib/%.o Core/CAN/lib/%.su Core/CAN/lib/%.cyclo: ../Core/CAN/lib/%.c Core/CAN/lib/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F091xC -c -I../Core/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F0xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-CAN-2f-lib

clean-Core-2f-CAN-2f-lib:
	-$(RM) ./Core/CAN/lib/can1.cyclo ./Core/CAN/lib/can1.d ./Core/CAN/lib/can1.o ./Core/CAN/lib/can1.su ./Core/CAN/lib/can2.cyclo ./Core/CAN/lib/can2.d ./Core/CAN/lib/can2.o ./Core/CAN/lib/can2.su ./Core/CAN/lib/flash.cyclo ./Core/CAN/lib/flash.d ./Core/CAN/lib/flash.o ./Core/CAN/lib/flash.su

.PHONY: clean-Core-2f-CAN-2f-lib

