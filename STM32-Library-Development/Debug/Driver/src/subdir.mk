################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Driver/src/gpio.c 

OBJS += \
./Driver/src/gpio.o 

C_DEPS += \
./Driver/src/gpio.d 


# Each subdirectory must supply rules for building sources it contributes
Driver/src/%.o: ../Driver/src/%.c Driver/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32F401CCFx -DSTM32 -DSTM32F4 -c -I"C:/Users/ASUS/Desktop/STM32-Library/STM32-Library-Development/Driver/inc" -I"C:/Users/ASUS/Desktop/STM32-Library/STM32-Library-Development/Testing/inc" -I../Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

