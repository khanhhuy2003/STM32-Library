################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Testing/src/gpio_test.c 

OBJS += \
./Testing/src/gpio_test.o 

C_DEPS += \
./Testing/src/gpio_test.d 


# Each subdirectory must supply rules for building sources it contributes
Testing/src/%.o: ../Testing/src/%.c Testing/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32F401CCFx -DSTM32 -DSTM32F4 -c -I"C:/Users/ASUS/Desktop/STM32-Library/STM32-Library-Development/Driver/inc" -I"C:/Users/ASUS/Desktop/STM32-Library/STM32-Library-Development/Testing/inc" -I../Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

