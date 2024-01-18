################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../BSP/DS1307.c \
../BSP/LCD.c 

OBJS += \
./BSP/DS1307.o \
./BSP/LCD.o 

C_DEPS += \
./BSP/DS1307.d \
./BSP/LCD.d 


# Each subdirectory must supply rules for building sources it contributes
BSP/%.o BSP/%.su BSP/%.cyclo: ../BSP/%.c BSP/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DNUCLEO_F401RE -DSTM32 -DSTM32F401RETx -DSTM32F4 -c -I../Inc -I"C:/Users/ravis/Desktop/STM32F401RE/STM32F401XX_DRIVERS/DRIVERS/Inc" -I"C:/Users/ravis/Desktop/STM32F401RE/STM32F401XX_DRIVERS/BSP" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-BSP

clean-BSP:
	-$(RM) ./BSP/DS1307.cyclo ./BSP/DS1307.d ./BSP/DS1307.o ./BSP/DS1307.su ./BSP/LCD.cyclo ./BSP/LCD.d ./BSP/LCD.o ./BSP/LCD.su

.PHONY: clean-BSP

