################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/006spi_tx_testing.c 

OBJS += \
./Src/006spi_tx_testing.o 

C_DEPS += \
./Src/006spi_tx_testing.d 


# Each subdirectory must supply rules for building sources it contributes
Src/006spi_tx_testing.o: ../Src/006spi_tx_testing.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F446RETx -DDEBUG -c -I../Inc -I"C:/Users/oaperez/Documents/MCU1-Course/STM32F446RE_Drivers/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/006spi_tx_testing.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

