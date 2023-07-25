/*
 * STM32F446RE_SPI_Driver.c
 *
 *  Created on: Jul 24, 2023
 *      Author: oaperez
 */

#include "STM32F446RE_SPI_Driver.h"

SPI_RegDef_t *pSPI1 = SPI1;
SPI_RegDef_t *pSPI2 = SPI2;
SPI_RegDef_t *pSPI3 = SPI3;
SPI_RegDef_t *pSPI4 = SPI4;

static void _spi_ITHandler_RXNE(SPI_Handle_t *pHandle);
static void _spi_ITHandler_TXE(SPI_Handle_t *pHandle);
static void _spi_ITHandler_OVR(SPI_Handle_t *pHandle);
__weak void _SPI_App_EventCallBack(SPI_Handle_t *pHandle, uint8_t event);


/********************************************************************************************************************************************************
 * 																	APIs															  					*
 ********************************************************************************************************************************************************/
// Get flag status of TXE
uint8_t _SPI_Get_FlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	if (pSPIx->SR & FlagName) return FLAG_SET;
	else return FLAG_RESET;
}


// Peripheral Clock setup
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
    if(EnorDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CRC1_SPE);
		if(pSPIx == SPI1)
		{
			SPI1_PCLCK_EN();
		}
		else if(pSPIx == SPI2)
		{
			SPI2_PCLCK_EN();
		}
        else if(pSPIx == SPI3)
		{
			SPI3_PCLCK_EN();
		}
        else if(pSPIx == SPI4)
		{
			SPI4_PCLCK_EN();
		}
	}
	else
	{
		pSPIx->CR1 &= ~(1 << SPI_CRC1_SPE);
		if(pSPIx == SPI1)
		{
			SPI1_PCLCK_DI();
		}
		else if(pSPIx == SPI2)
		{
			SPI2_PCLCK_DI();
		}
        else if(pSPIx == SPI3)
		{
			SPI3_PCLCK_DI();
		}
        else if(pSPIx == SPI4)
		{
			SPI4_PCLCK_DI();
		}
	}
}


// Init and De-Init
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	uint32_t tempreg = 0;
	tempreg |= pSPIHandle->SPI_Config.DeviceMode << SPI_CRC1_MSTR;										// Master or Slave.
	if( pSPIHandle->SPI_Config.BusConfig == SPI_BUS_CONFIG_FD)  										// Selects the bus mode (Full DUPLEX, half DUPLEX, SIMPLEX)
	{
		// bidi should be clear
		tempreg &= ~(1<<SPI_CRC1_BIDIMODE);
	}
	else if (pSPIHandle->SPI_Config.BusConfig == SPI_BUS_CONFIG_HD)
	{
		// bidi should be set
		tempreg |= (1<<SPI_CRC1_BIDIMODE);
	}
	else if (pSPIHandle->SPI_Config.BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		// Bidi should be clear
		tempreg &= ~(1<<SPI_CRC1_BIDIMODE);
		// RXMode should be set
		tempreg |= (1<<SPI_CRC1_RXONLY);
	}
	if (pSPIHandle->SPI_Config.BusConfig < SPI_BUS_CONFIG_MULTIBUS) tempreg |= (1 << SPI_CRC1_SSI);		// Enables or Disable the SSI, SSI disabled only when multibus is required
	else tempreg &= ~(1 << SPI_CRC1_SSI);
	tempreg |= pSPIHandle->SPI_Config.SclkSpeed << SPI_CRC1_BR;											// Selects the speed
	tempreg |= pSPIHandle->SPI_Config.DFF << SPI_CRC1_DFF;												// Data Frame format
	tempreg |= pSPIHandle->SPI_Config.CPAH << SPI_CRC1_CPAH;											// Clock phase
	tempreg |= pSPIHandle->SPI_Config.CPOL << SPI_CRC1_CPOL;											// Clock polarity
	tempreg |= pSPIHandle->SPI_Config.SSM << SPI_CRC1_SSM;												// Software slave management
	
	pSPIHandle -> pSPIx->CR1 = tempreg;
}


void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	uint32_t tempreg = 0;
	pSPIx->CR1 = tempreg;
}


// Data Send and Receive
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
	while (Len > 0)
	{
		while (_SPI_Get_FlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);							// Wait for the TXE Buffer
		if (pSPIx->CR1 & (1<< SPI_CRC1_DFF))													// If '1' the Data Frame format is 16 bits
		{
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			Len --;
			Len --;
			(uint16_t*)pTxBuffer++;
		}
		else																					// Data frame format is 8bit
		{
			pSPIx->DR = *pTxBuffer;
			Len --;
			pTxBuffer++;
		}	
	}
}


void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
	while (Len > 0)
	{
		while (_SPI_Get_FlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET)							// Wait for the RXNE Buffer
		if (pSPIx->CR1 & (1<< SPI_CRC1_DFF))													// If '1' the Data Frame format is 16 bits
		{
			*((uint16_t*)pRxBuffer) = pSPIx->DR;
			Len --;
			Len --;
			(uint16_t*)pRxBuffer++;
		}
		else																					// Data frame format is 8bit
		{
			*pRxBuffer = pSPIx->DR;
			Len --;
			pRxBuffer++;
		}	
	}
}


uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->TxState;
	
	if (state != SPI_BUSY_IN_RX)
	{
		// 1. Save the TxBuffer address and Len information in sombe global variables.
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;
		
		// 2. Mark the SPI state as busy in transmission so that no other code and take
		// 	  over same SPI peripheral until transmission is over.
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		// 3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR.
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CRC2_TXEIE);
	}

	return state;
}


uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->RxState;
	
	if (state != SPI_BUSY_IN_TX)
	{
		// 1. Save the RxBuffer address and Len information in sombe global variables.
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Len;
		
		// 2. Mark the SPI state as busy in reception so that no other code and take
		// 	  over same SPI peripheral until reception is over.
		pSPIHandle->RxState = SPI_BUSY_IN_RX;

		// 3. Enable the RXNEIE control bit to get interrupt whenever RXE flag is set in SR.
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CRC2_RXNEIE);
	}

	return state;
}


// IRQ Configuration and ISR Handling
void SPI_IRQITConfig(uint8_t IRQNumber, uint8_t EnorDi)
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


void SPI_IRQPriorityConfig (uint8_t IRQNumber, uint8_t IRQPriority)
{
	// 1. First lets find out the IPR Register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount = (8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + (iprx)) |= (IRQPriority << shift_amount);
}


void SPI_IRQHandling(SPI_Handle_t *pHandle)
{
	// First find out what triggered the interruption
	// a) Handle the RXNE event
	// b) Handle the TXE Flag
	// c) Handle the Error
	uint8_t temp1, temp2;
	
	// Check for the RXNE
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_RXNE);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CRC2_RXNEIE);
	if ( temp1 && temp2)
	{
		// Handle RXNE
		_spi_ITHandler_RXNE(pHandle);
	}
	
	// Check for the TXE
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_TXE);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CRC2_TXEIE);
	if ( temp1 && temp2)
	{
		// Handle TXE
		_spi_ITHandler_TXE(pHandle);
	}

	// Check for the OVR
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_OVR);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CRC2_ERRIE);
	if ( temp1 && temp2)
	{
		// Handle OVR
		_spi_ITHandler_OVR(pHandle);
	}

}


// Other Peripheral Control APIs
static void _spi_ITHandler_RXNE(SPI_Handle_t *pHandle)
{
	while (_SPI_Get_FlagStatus(pHandle->pSPIx, SPI_RXNE_FLAG) == FLAG_RESET);							// Wait for the TXE Buffer
	
	if (pHandle->pSPIx->CR1 & (1<< SPI_CRC1_DFF))											// If '1' the Data Frame format is 16 bits
	{
		pHandle->pSPIx->DR = *((uint16_t*)pHandle->pRxBuffer);
		pHandle->RxLen -=2;
		(uint16_t*)pHandle->pRxBuffer++;
	}
	else																					// Data frame format is 8bit
	{
		pHandle->pSPIx->DR = *pHandle->pRxBuffer;
		pHandle->RxLen --;
		pHandle->pRxBuffer++;
	}

	if (pHandle->RxLen < 1)
	{
		// If the TxLen is zero, closes the SPI transmission and inform the application that Tx is over.
		pHandle->pSPIx->CR2 &= ~(1 << SPI_CRC2_RXNEIE);
		pHandle->pRxBuffer = NULL;
		pHandle->RxLen = 0;
		pHandle->RxState = SPI_READY;
		_SPI_App_EventCallBack(pHandle, SPI_EVENT_RX_COMPLETE);
	}
}


static void _spi_ITHandler_TXE(SPI_Handle_t *pHandle)
{
	while (_SPI_Get_FlagStatus(pHandle->pSPIx, SPI_TXE_FLAG) == FLAG_RESET);							// Wait for the TXE Buffer
	
	if (pHandle->pSPIx->CR1 & (1<< SPI_CRC1_DFF))											// If '1' the Data Frame format is 16 bits
	{
		pHandle->pSPIx->DR = *((uint16_t*)pHandle->pTxBuffer);
		pHandle->TxLen -=2;
		(uint16_t*)pHandle->pTxBuffer++;
	}
	else																					// Data frame format is 8bit
	{
		pHandle->pSPIx->DR = *pHandle->pTxBuffer;
		pHandle->TxLen --;
		pHandle->pTxBuffer++;
	}

	if (pHandle->TxLen < 1)
	{
		// If the TxLen is zero, closes the SPI transmission and inform the application that Tx is over.
		pHandle->pSPIx->CR2 &= ~(1 << SPI_CRC2_TXEIE);
		pHandle->pTxBuffer = NULL;
		pHandle->TxLen = 0;
		pHandle->TxState = SPI_READY;
		_SPI_App_EventCallBack(pHandle, SPI_EVENT_TX_COMPLETE);
	}
}


static void _spi_ITHandler_OVR(SPI_Handle_t *pHandle)
{
	uint8_t temp = 0;
	//1. Clear the OVR flag
	if (pHandle->TxState != SPI_BUSY_IN_TX)
	{
		temp |= pHandle->pSPIx->DR;
		temp |= pHandle->pSPIx->DR;
	}
	//2. Inform the application
	_SPI_App_EventCallBack(pHandle, SPI_EVENT_OVR_COMPLETE);
}

__weak void _SPI_App_EventCallBack(SPI_Handle_t *pHandle, uint8_t event)
{
	// TODO 
}