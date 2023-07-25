/*
 * 001LedToggle.c
 *
 *  Created on: Jul 19, 2023
 *      Author: oaperez
 */

#include "STM32F446RE_gpio_driver.h"
#include <string.h>

void delay (void)
{
	for(uint32_t i = 0; i < 500000; i++);
}


void EXTI15_10_IRQHandler(void) 										//By this the .weak inside the startup, will be ignored.
{
	//Handle the interrupt
	GPIO_IRQHandling(GPIO_PIN_NO_13);
	GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);
	delay();
}


int main()
{
	// Initializing the SYSCFG
	SYSCFG_PeriClockControl(ENABLE);

	// Configuring the LED Button (PA5)
	GPIO_Handle_t gpio_led;
	memset(&gpio_led, 0, sizeof(gpio_led));

	gpio_led.pGPIOx = GPIOA;
	gpio_led.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	gpio_led.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpio_led.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpio_led.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	gpio_led.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	// Initializing GPIOA communication
	GPIO_PeriClockControl(GPIOA, ENABLE);

	GPIO_Init(&gpio_led);
	
	//////////////////////////////////////////////////////////////////////////////////////////////////////
	// Configuring the Interruption Button (PC13)

	GPIO_Handle_t button_int;
	memset(&button_int, 0, sizeof(button_int));

	button_int.pGPIOx = GPIOC;
	button_int.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	button_int.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	button_int.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	button_int.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PD;

	// Initializing GPIOC communication
	GPIO_PeriClockControl(GPIOC, ENABLE);

	GPIO_Init(&button_int);					// Sets it as an External interruption

	// IRQ Configurations Setting
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI15_10, IRQ_PRIO15);
	GPIO_IRQITConfig(IRQ_NO_EXTI15_10, ENABLE);

	while(1);
	return 0;
}
