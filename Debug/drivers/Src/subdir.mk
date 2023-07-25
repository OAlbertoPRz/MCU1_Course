################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/Src/STM32F446RE_SPI_Driver.c \
../drivers/Src/STM32F446RE_gpio_driver.c 

OBJS += \
./drivers/Src/STM32F446RE_SPI_Driver.o \
./drivers/Src/STM32F446RE_gpio_driver.o 

C_DEPS += \
./drivers/Src/STM32F446RE_SPI_Driver.d \
./drivers/Src/STM32F446RE_gpio_driver.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/Src/STM32F446RE_SPI_Driver.o: ../drivers/Src/STM32F446RE_SPI_Driver.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F446RETx -DDEBUG -c -I../Inc -I"C:/Users/oaperez/Documents/MCU1-Course/STM32F446RE_Drivers/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"drivers/Src/STM32F446RE_SPI_Driver.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
drivers/Src/STM32F446RE_gpio_driver.o: ../drivers/Src/STM32F446RE_gpio_driver.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F446RETx -DDEBUG -c -I../Inc -I"C:/Users/oaperez/Documents/MCU1-Course/STM32F446RE_Drivers/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"drivers/Src/STM32F446RE_gpio_driver.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

