/*
 * STM32F446RE_gpio.c
 *
 *  Created on: Jul 18, 2023
 *      Author: oaperez
 */

#include "STM32F446RE_gpio_driver.h"

/**********************************************************************************************************************************************************************
															CREATION OF REGISTERS
***********************************************************************************************************************************************************************/
GPIO_RegDef_t *pGPIOA = GPIOA;
GPIO_RegDef_t *pGPIOB = GPIOB;
GPIO_RegDef_t *pGPIOC = GPIOC;
GPIO_RegDef_t *pGPIOD = GPIOD;
GPIO_RegDef_t *pGPIOE = GPIOE;
GPIO_RegDef_t *pGPIOF = GPIOF;
GPIO_RegDef_t *pGPIOG = GPIOG;
GPIO_RegDef_t *pGPIOH = GPIOH;
RCC_RegDef_t *pRCC = RCC;
EXTI_RegDef_t *pEXTI = EXTI;
SYSCFG_RegDet_t *pSYSCFG = SYSCFG;



/**********************************************************************************************************************************************************************
 * 																	APIs															  								  *
 **********************************************************************************************************************************************************************/
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLCK_EN();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLCK_EN();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLCK_EN();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLCK_EN();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLCK_EN();
		}
		else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLCK_EN();
		}
		else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLCK_EN();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLCK_EN();
		}
	}
	else
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLCK_DI();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLCK_DI();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLCK_DI();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLCK_DI();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLCK_DI();
		}
		else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLCK_DI();
		}
		else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLCK_DI();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLCK_DI();
		}
	}

}


void SYSCFG_PeriClockControl(uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		SYSCFG_PCLCK_EN();
	}else
	{
		SYSCFG_PCLCK_DI();
	}
}


// Init and De-Init
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp = 0;
	// 1) Configure the mode of GPIO pin
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);
	
	if(pGPIOHandle -> GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		temp = (pGPIOHandle -> GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber)); // Multiplied by 2 because the mode are selected by 2 bits
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber); 	// Clearing the mode
		pGPIOHandle->pGPIOx->MODER |= temp;														// Setting the mode
	}
	/***************************************************************************************************************************************************
	*											INTERRUPTION CONFIG																					   *
	****************************************************************************************************************************************************/
	else
	{
		// Configuring the Interruption mode pin
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			// 1. Configure the FTSR (Falling Trigger Selection Register)
			pEXTI->FTSR |= (1 << pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber);
			// Clear the corresponding RTST bit
			pEXTI->RTSR &= ~(1 << pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			// 1. Configure the RTSR (Rising Trigger Selection Register)
			pEXTI->RTSR |= (1 << pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber);
			// Clear the corresponding RTST bit
			pEXTI->FTSR &= ~(1 << pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			// 1. Configure both RTSR and FTSR
			pEXTI->FTSR |= (1 << pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber);
			pEXTI->RTSR |= (1 << pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber);
		}
		
		// 2. Configure the GPIO port selection in SYSCFG_EXTICR
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber/4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber%4;
		uint8_t portcode = EXTI_GPIO_IDENTIFICATOR(pGPIOHandle->pGPIOx);
		pSYSCFG->EXTICR[temp1] |= portcode << (temp2 * 4);
		
		// 3. Enable the exit interrupt delivery using IMR
		pEXTI->IMR |= 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
	}
	  
	temp=0;
	// 2) Configure the Speed
	temp = (pGPIOHandle-> GPIO_PinConfig.GPIO_PinSpeed << (2* pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEED &= ~(0x3 << pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber); 	// Clearing the mode
	pGPIOHandle->pGPIOx->OSPEED |= temp;													// Setting the mode
	
	temp = 0;
	// 3) Configure the PUPD settings
	temp = (pGPIOHandle-> GPIO_PinConfig.GPIO_PinPuPdControl << (2* pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber); 	// Clearing the mode
	pGPIOHandle->pGPIOx->PUPDR |= temp;														// Setting the mode
	
	temp = 0;
	// 4) Configure the OPTYPE
	temp = (pGPIOHandle-> GPIO_PinConfig.GPIO_PinOPType << (2* pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber); 	// Clearing the mode
	pGPIOHandle->pGPIOx->OTYPER |= temp;													// Setting the mode
	
	temp = 0;
	// 5) Configure the alternate functionality
	if (pGPIOHandle ->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		uint32_t temp1, temp2;
		temp1 = pGPIOHandle ->GPIO_PinConfig.GPIO_PinNumber / 8; 												// This will give the position for the Alternate[0/1]
		temp2 = pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber %8; 												// This will give the value of the postion of the 32 bits.
		pGPIOHandle -> pGPIOx->AFR[temp1] &= ~(0xF << (4*temp2));												//Clearing
		pGPIOHandle -> pGPIOx->AFR[temp1] |= (pGPIOHandle -> GPIO_PinConfig.GPIO_PinAltFunMode << (4*temp2));
	}
}


void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}
	else if(pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}
	else if(pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}
	else if(pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}
	else if(pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}
	else if(pGPIOx == GPIOF)
	{
		GPIOF_REG_RESET();
	}
	else if(pGPIOx == GPIOG)
	{
		GPIOG_REG_RESET();
	}
	else if(pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}
	
	else
	{
		//TODO
	}
}


// Data Read and Write
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (pGPIOx->IDR >> PinNumber) & 0x00000001;
	return value;
}


uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t) pGPIOx->IDR;
	return value;
}


void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	 if(Value == GPIO_PIN_SET)
	 {
		pGPIOx->ODR |= (1 <<PinNumber);
	 }else
	 {
		pGPIOx->ODR &= ~(1 <<PinNumber);
	 }
}


void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
		pGPIOx->ODR = Value;
}


void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);
}


// IRQ Configuration and ISR handling
void GPIO_IRQITConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(IRQNumber <=31)
		{
			//Program the ISER0 register
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if (IRQNumber >31 && IRQNumber <64)
		{
			//Program the ISER1 register
			*NVIC_ISER1 |= (1 << IRQNumber%32);
		}
		else if (IRQNumber >=64 && IRQNumber <96)
		{
			//Program the ISER2 register
			*NVIC_ISER2 |= (1 << IRQNumber%64);
		}
	}
	else
	{
		if(IRQNumber <=31)
		{
			//Program the ICER0 register
			*NVIC_ICER0 |= (1 << IRQNumber);
		}
		else if (IRQNumber >31 && IRQNumber <64)
		{
			//Program the ICER1 register
			*NVIC_ICER1 |= (1 << IRQNumber%32);
		}
		else if (IRQNumber >=64 && IRQNumber <96)
		{
			//Program the ICER2 register
			*NVIC_ICER2 |= (1 << IRQNumber%64);
		}
	}
}


void GPIO_IRQPriorityConfig (uint8_t IRQNumber, uint8_t IRQPriority)
{
	// 1. First lets find out the IPR Register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount = (8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + (iprx)) |= (IRQPriority << shift_amount);
}


void GPIO_IRQHandling(uint8_t PinNumber)
{
	//Clear the EXTI PR register corresponding to the pin number
	if(pEXTI->PR & (1 << PinNumber))
	{
		//Clear
		pEXTI->PR |= (1 << PinNumber);
	}
}

