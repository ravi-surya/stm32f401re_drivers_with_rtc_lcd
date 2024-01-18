################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../DRIVERS/Src/STM32F401xx_GPIO_DRIVER.c \
../DRIVERS/Src/STM32F401xx_I2C_DRIVER.c \
../DRIVERS/Src/STM32F401xx_SPI_DRIVER.c 

OBJS += \
./DRIVERS/Src/STM32F401xx_GPIO_DRIVER.o \
./DRIVERS/Src/STM32F401xx_I2C_DRIVER.o \
./DRIVERS/Src/STM32F401xx_SPI_DRIVER.o 

C_DEPS += \
./DRIVERS/Src/STM32F401xx_GPIO_DRIVER.d \
./DRIVERS/Src/STM32F401xx_I2C_DRIVER.d \
./DRIVERS/Src/STM32F401xx_SPI_DRIVER.d 


# Each subdirectory must supply rules for building sources it contributes
DRIVERS/Src/%.o DRIVERS/Src/%.su DRIVERS/Src/%.cyclo: ../DRIVERS/Src/%.c DRIVERS/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DNUCLEO_F401RE -DSTM32 -DSTM32F401RETx -DSTM32F4 -c -I../Inc -I"C:/Users/ravis/Desktop/STM32F401RE/STM32F401XX_DRIVERS/DRIVERS/Inc" -I"C:/Users/ravis/Desktop/STM32F401RE/STM32F401XX_DRIVERS/BSP" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-DRIVERS-2f-Src

clean-DRIVERS-2f-Src:
	-$(RM) ./DRIVERS/Src/STM32F401xx_GPIO_DRIVER.cyclo ./DRIVERS/Src/STM32F401xx_GPIO_DRIVER.d ./DRIVERS/Src/STM32F401xx_GPIO_DRIVER.o ./DRIVERS/Src/STM32F401xx_GPIO_DRIVER.su ./DRIVERS/Src/STM32F401xx_I2C_DRIVER.cyclo ./DRIVERS/Src/STM32F401xx_I2C_DRIVER.d ./DRIVERS/Src/STM32F401xx_I2C_DRIVER.o ./DRIVERS/Src/STM32F401xx_I2C_DRIVER.su ./DRIVERS/Src/STM32F401xx_SPI_DRIVER.cyclo ./DRIVERS/Src/STM32F401xx_SPI_DRIVER.d ./DRIVERS/Src/STM32F401xx_SPI_DRIVER.o ./DRIVERS/Src/STM32F401xx_SPI_DRIVER.su

.PHONY: clean-DRIVERS-2f-Src

