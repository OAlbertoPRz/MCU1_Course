/*
 * 006spi_tx_testing.c
 *
 *  Created on: Jul 24, 2023
 *      Author: oaperez
 */


/*
 *	PB14 --> SPI2_MISO
 *	PB15 --> SPI2_MOSI
 *	PB13 --> SPI2_SCLK
 *	PB12 --> SPI2_NSS
 *	ALT function mode : 5
 */

#include "STM32F446RE_gpio_driver.h"
#include "STM32F446RE_SPI_Driver.h"
#include <stdint.h>
#include <string.h>


void _SPI_GPIO_INITS(void)
{
	GPIO_Handle_t SPI_Pins;
	memset (&SPI_Pins, 0, sizeof(SPI_Pins));
	SPI_Pins.pGPIOx = GPIOB;
	SPI_Pins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPI_Pins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPI_Pins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPI_Pins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPI_Pins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	
	// SCLK
	SPI_Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPI_Pins);

	// MOSI
	SPI_Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPI_Pins);

	// MISO
	SPI_Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(&SPI_Pins);

	// NSS
	SPI_Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPI_Pins);
	
}


void _SPI2_INITS(void)
{
	SPI_Handle_t SPI2_Handle;

	SPI2_Handle.pSPIx = SPI2;
	SPI2_Handle.SPI_Config.BusConfig = SPI_BUS_CONFIG_FD;
	SPI2_Handle.SPI_Config.DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2_Handle.SPI_Config.SclkSpeed = SPI_SLCK_SPEED_DIV2;
	SPI2_Handle.SPI_Config.DFF = SPI_DFF_8bit;
	SPI2_Handle.SPI_Config.CPAH = SPI_CPAH_LOW;
	SPI2_Handle.SPI_Config.CPOL = SPI_CPOL_LOW;
	SPI2_Handle.SPI_Config.SSM = SPI_SSM_HW;

	SPI_PeriClockControl(SPI2_Handle.pSPIx, ENABLE);
	SPI_Init(&SPI2_Handle);
}


int main()
{	
	char user_data[] = "Hello World";

	// Initializing peripherals
	SYSCFG_PeriClockControl(ENABLE);

	// Initializing GPIOB to function mode 5
	_SPI_GPIO_INITS();

	// Initializing SPI2
	_SPI2_INITS();

	// Send data through SPI2
	SPI_SendData(SPI2,(uint8_t*) user_data, strlen(user_data));
	
	while(1);
	return 0;
}
