/*
 * STM32F446RE_gpio_driver.h
 *
 *  Created on: Jul 24, 2023
 *      Author: oaperez
 */

#ifndef INC_STM32F446RE_SPI_DRIVER_H_
#define INC_STM32F446RE_SPI_DRIVER_H_

#include "STM32F446RE.h"


// Defining the registers for SPIx
typedef struct 
{
	uint8_t DeviceMode;
	uint8_t BusConfig;
	uint8_t SclkSpeed;
	uint8_t DFF;
	uint8_t CPAH;
	uint8_t CPOL;
	uint8_t SSM;
}SPI_Config_t;


// This is a Handle structure for a SPI pin
typedef struct
{
	SPI_RegDef_t 	*pSPIx;					// Pointer to hold the base address of the SPI peripheral
    SPI_Config_t 	SPI_Config;            	// Holds the base address of the SPI port to which the pin belongs
	uint8_t			*pTxBuffer;				// Contains the Tx buffer address
	uint8_t			*pRxBuffer;				// Contains the Rx buffer address
	uint32_t		TxLen;					// Contains the Tx Len
	uint32_t		RxLen;					// Contains the Rx Len
	uint8_t			TxState;				// Contains the Tx State
	uint8_t			RxState;				// Contains the Rx State
}SPI_Handle_t;


/*********************************************************************************************************
*                                  GENERIC MACROS			                                             *
**********************************************************************************************************/
#define SPI_TXE_FLAG						(1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG						(1 << SPI_SR_RXNE)
#define SPI_CHSIDE_FLAG						(1 << SPI_SR_CHSIDE)
#define SPI_UDR_FLAG						(1 << SPI_SR_UDR)
#define SPI_CRCERR_FLAG						(1 << SPI_SR_CRCERR)
#define SPI_MODF_FLAG						(1 << SPI_SR_MODF)
#define SPI_OVR_FLAG						(1 << SPI_SR_OVR)
#define SPI_BUSY_FLAG						(1 << SPI_SR_BSY)
#define SPI_FRE_FLAG						(1 << SPI_SR_FRE)


/*********************************************************************************************************
*                                  MACROS FOR SPI_Config_t                                               *
**********************************************************************************************************/
// SPI_DevideMode							(bit 2 of the CPR1)
#define SPI_DEVICE_MODE_SLAVE               0
#define SPI_DEVICE_MODE_MASTER              1			// Produces the clock

// SPI_BusConfig							(Bit 15 of the CPR1)
#define SPI_BUS_CONFIG_FD                   1           // Full Duplex
#define SPI_BUS_CONFIG_HD                   2           // Half Duplex
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY       3           // Simplex Receiver only
#define SPI_BUS_CONFIG_MULTIBUS				4			// MultiBus configuration

// SPI_SclkSpeed
#define SPI_SLCK_SPEED_DIV2					0			// When the clock is devided by 2
#define SPI_SLCK_SPEED_DIV4					1			// When the clock is devided by 4
#define SPI_SLCK_SPEED_DIV8					2			// When the clock is devided by 8
#define SPI_SLCK_SPEED_DIV16				3			// When the clock is devided by 16
#define SPI_SLCK_SPEED_DIV32				4			// When the clock is devided by 32
#define SPI_SLCK_SPEED_DIV64				5			// When the clock is devided by 64
#define SPI_SLCK_SPEED_DIV128				6			// When the clock is devided by 128
#define SPI_SLCK_SPEED_DIV256				7			// When the clock is devided by 256

// SPI_DFF
#define SPI_DFF_8bit                        0			// Defualt
#define SPI_DFF_16bit                       1

// SPI_CPOL
#define SPI_CPOL_LOW						0
#define SPI_CPOL_HIGH						1

// SPI_CPAH
#define SPI_CPAH_LOW						0
#define SPI_CPAH_HIGH						1

// SPI_SSM
#define SPI_SSM_SW							0
#define SPI_SSM_HW							1


/*********************************************************************************************************
*                                  MACROS FOR CRC1 REGISTER (SPI)                                        *
**********************************************************************************************************/
// Bits of the CRC1 Register in the SPI
#define SPI_CRC1_CPAH				0
#define SPI_CRC1_CPOL				1
#define SPI_CRC1_MSTR				2
#define SPI_CRC1_BR					3
#define SPI_CRC1_SPE				6
#define SPI_CRC1_LSBFRST			7
#define SPI_CRC1_SSI				8
#define SPI_CRC1_SSM				9
#define SPI_CRC1_RXONLY				10
#define SPI_CRC1_DFF				11
#define SPI_CRC1_CRCNEXT			12
#define SPI_CRC1_CRCEN				13
#define SPI_CRC1_BIDIOE				14
#define SPI_CRC1_BIDIMODE			15


/*********************************************************************************************************
*                                  MACROS FOR CRC2 REGISTER (SPI)                                        *
**********************************************************************************************************/
// Bits of the CRC2 Register in the SPI
#define SPI_CRC2_RXDMAEN			0
#define SPI_CRC2_TXDMAEN			1
#define SPI_CRC2_SSOE				2
#define SPI_CRC2_FRF				4
#define SPI_CRC2_ERRIE				5
#define SPI_CRC2_RXNEIE				6
#define SPI_CRC2_TXEIE				7


/*********************************************************************************************************
*                                  MACROS FOR SR REGISTER (SPI)                                        *
**********************************************************************************************************/
// Bits of the Status Register in the SPI
#define SPI_SR_RXNE					0
#define SPI_SR_TXE					1
#define SPI_SR_CHSIDE				2
#define SPI_SR_UDR					3
#define SPI_SR_CRCERR				4
#define SPI_SR_MODF					5
#define SPI_SR_OVR					6
#define SPI_SR_BSY					7
#define SPI_SR_FRE					8


/*********************************************************************************************************
*                           MACROS FOR POSSIBLE SPI APPLICATION STATE                                    *
**********************************************************************************************************/
// Bits for possible SPI application state (For interruptions)
#define SPI_READY					0
#define SPI_BUSY_IN_RX				1
#define SPI_BUSY_IN_TX				2


/*********************************************************************************************************
*                           MACROS FOR POSSIBLE SPI APPLICATION EVENTS                                   *
**********************************************************************************************************/
// Bits for possible SPI application state (For interruptions)
#define SPI_EVENT_TX_COMPLETE				1
#define SPI_EVENT_RX_COMPLETE				2
#define SPI_EVENT_OVR_COMPLETE				3


/*********************************************************************************************************
 *								APIs supported by this driver
 *				For more information about this APIs check the function definitions
 *********************************************************************************************************/
// Peripheral Clock setup
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

// Init and De-Init
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

// Data Send and Receive
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len);

// IRQ Configuration and ISR Handling
void SPI_IRQITConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig (uint8_t IRQNumber, uint8_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);

// Other Peripheral Control APIs

#endif /* INC_STM32F446RE_SPI_DRIVER_H_ */
