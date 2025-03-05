################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../OLED/Src/fonts.c \
../OLED/Src/ssd1306.c 

OBJS += \
./OLED/Src/fonts.o \
./OLED/Src/ssd1306.o 

C_DEPS += \
./OLED/Src/fonts.d \
./OLED/Src/ssd1306.d 


# Each subdirectory must supply rules for building sources it contributes
OLED/Src/%.o OLED/Src/%.su: ../OLED/Src/%.c OLED/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G0B1xx -c -I../Core/Inc -I"D:/HEXABITZ_COURSE/RepoProjectsCourseEmbeddedSystems/OLED/Inc" -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-OLED-2f-Src

clean-OLED-2f-Src:
	-$(RM) ./OLED/Src/fonts.d ./OLED/Src/fonts.o ./OLED/Src/fonts.su ./OLED/Src/ssd1306.d ./OLED/Src/ssd1306.o ./OLED/Src/ssd1306.su

.PHONY: clean-OLED-2f-Src

