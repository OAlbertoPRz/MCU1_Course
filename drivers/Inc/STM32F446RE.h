/*
 * STM32F446RE.h
 *
 *  Created on: Jul 18, 2023
 *      Author: oaperez
 */

#ifndef INC_STM32F446RE_H_
#define INC_STM32F446RE_H_
#define __vo 				volatile
#define __weak 				__attribute__((weak))
#include <stdint.h>
#include <stddef.h>


/************************************************************************************************************************************************************************
 * 																START: PROCESSOR SPECIFIC DETAILS
************************************************************************************************************************************************************************/
#define NVIC_PR_BASE_ADDR					((__vo uint32_t*) 0xE000E400)
#define NO_PR_BITS_IMPLEMENTED				4									// In this case

#define NVIC_ISER0							((__vo uint32_t*) 0xE000E100)
#define NVIC_ISER1							((__vo uint32_t*) 0xE000E104)
#define NVIC_ISER2							((__vo uint32_t*) 0xE000E108)
#define NVIC_ISER3							((__vo uint32_t*) 0xE000E10C)


#define NVIC_ICER0							((__vo uint32_t*) 0xE000E180)
#define NVIC_ICER1							((__vo uint32_t*) 0xE000E184)
#define NVIC_ICER2							((__vo uint32_t*) 0xE000E188)
#define NVIC_ICER3							((__vo uint32_t*) 0xE000E18C)

// Position of the Interrupt Request at EXTI (External Interrupt/Event Controller)
#define IRQ_NO_EXTI0						6					
#define IRQ_NO_EXTI1						7
#define IRQ_NO_EXTI2						8
#define IRQ_NO_EXTI3						9
#define IRQ_NO_EXTI4						10
#define IRQ_NO_EXTI9_5						23
#define IRQ_NO_EXTI15_10					40

// Position of the Interrupt Request at SPI
#define IRQ_NO_SPI1							35
#define IRQ_NO_SPI2							36
#define IRQ_NO_SPI3							51
#define IRQ_NO_SPI4							84

// Priority macros for the Interruption Request at EXTI
#define IRQ_PRIO0							0
#define IRQ_PRIO1							1
#define IRQ_PRIO2							2
#define IRQ_PRIO3							3
#define IRQ_PRIO4							4
#define IRQ_PRIO5							5
#define IRQ_PRIO6							6
#define IRQ_PRIO7							7
#define IRQ_PRIO8							8
#define IRQ_PRIO9							9
#define IRQ_PRIO10							10
#define IRQ_PRIO11							11
#define IRQ_PRIO12							12
#define IRQ_PRIO13							13
#define IRQ_PRIO14							14
#define IRQ_PRIO15							15


/***********************************************************************************************************************************
 * 														GENERIC MACROS
 ***********************************************************************************************************************************/
#define ENABLE 								1
#define DISABLE 							0
#define SET 								ENABLE
#define RESET 								DISABLE
#define GPIO_PIN_SET						SET
#define GPIO_PIN_RESET						RESET
#define EXTI_GPIO_IDENTIFICATOR(x) 			((x==GPIOA) ? 0 :\
											(x==GPIOB) ? 1 : \
											(x==GPIOC) ? 2 : \
											(x==GPIOD) ? 3 : \
											(x==GPIOE) ? 4 : \
											(x==GPIOF) ? 5 : \
											(x==GPIOG) ? 6: -1)
#define FLAG_RESET							RESET
#define FLAG_SET							SET


/***********************************************************************************************************************************
 * 														ADDRESSES
 ***********************************************************************************************************************************/
/*
 * Base Addresses of flash and SRAM memories
 */
#define FLASH_BASEADDR						0x08000000U				// Allocates the base address of Flash memory
#define SRAM1_BASEADDR						0x20000000U				// 112KB
#define SRAM2_BASEADDR						0x2001C000U				// 16KB
#define ROM									0x1FFF0000U				// (System Memory)
#define SRAM 								SRAM1_BASEADDR

/*
 * Defining the base addresses of the bus domains
 */

#define PERIPH_BASE							0x40000000U
/* AHB Peripherals (for those who use High speed data communication*/
#define AHB1PERIPH_BASEADDR					0x40020000U
#define AHB2PERIPH_BASEADDR					0x50000000U
/* APB Peripherals (for those who use Low speed data communication*/
#define APB1_BASEADDR						PERIPH_BASE				// Peripheral base address
#define APB2_BASEADDR						0x40010000U


/*
 * Defining every peripheral used in the course
 */
// GPIO ADDRESSES
#define GPIOA_BASEADDR						AHB1PERIPH_BASEADDR
#define GPIOB_BASEADDR						(AHB1PERIPH_BASEADDR+0x0400U)
#define GPIOC_BASEADDR						(AHB1PERIPH_BASEADDR+0x0800U)
#define GPIOD_BASEADDR						(AHB1PERIPH_BASEADDR+0x0C00U)
#define GPIOE_BASEADDR						(AHB1PERIPH_BASEADDR+0x1000U)
#define GPIOF_BASEADDR						(AHB1PERIPH_BASEADDR+0x1400U)
#define GPIOG_BASEADDR						(AHB1PERIPH_BASEADDR+0x1800U)
#define GPIOH_BASEADDR						(AHB1PERIPH_BASEADDR+0x1C00U)
//RCC
#define RCC_BASEADDR						(AHB1PERIPH_BASEADDR+0x3800U)
// I2C
#define I2C1_BASEADDR						(APB1_BASEADDR+0x5400U)
#define I2C2_BASEADDR						(APB1_BASEADDR+0x5C00U)
#define I2C3_BASEADDR						(APB1_BASEADDR+0x5800U)
// SPI
#define SPI1_BASEADDR						(APB2_BASEADDR+0x3000U)
#define SPI2_BASEADDR						(APB1_BASEADDR+0x3800U)
#define SPI3_BASEADDR						(APB1_BASEADDR+0x3C00U)
#define SPI4_BASEADDR						(APB2_BASEADDR+0x3400U)
// USART
#define USART1_BASEADDR						(APB2_BASEADDR+0x1000U)
#define USART2_BASEADDR						(APB1_BASEADDR+0x4400U)
#define USART3_BASEADDR						(APB1_BASEADDR+0x4800U)
#define USART6_BASEADDR						(APB2_BASEADDR+0x1400U)
// UART
#define UART4_BASEADDR						(APB1_BASEADDR+0x4C00U)
#define UART5_BASEADDR						(APB1_BASEADDR+0x5000U)
// EXTI
#define EXTI_BASEADDR						(APB2_BASEADDR+0x3C00U)
// SYSCFG
#define SYSCFG_BASEADDR						(APB2_BASEADDR+0x3800U)


/***********************************************************************************************************************************
 * 														STRUCTURES
 ***********************************************************************************************************************************/
/*
 *	Defining the registers of every GPIOx
 */
typedef struct
{
	__vo uint32_t MODER;						// Address offset: 0x00
	__vo uint32_t OTYPER;						// Address offset: 0x04
	__vo uint32_t OSPEED;						// Address offset: 0x08
	__vo uint32_t PUPDR;						// Address offset: 0x0C
	__vo uint32_t IDR;							// Address offset: 0x10
	__vo uint32_t ODR;							// Address offset: 0x14
	__vo uint16_t BSRRL;						// Address offset: 0x18
	__vo uint16_t BSRRH;						// Address offset: 0x1A
	__vo uint32_t LCKR;							// Address offset: 0x1C
	__vo uint32_t AFR[2];						// Address offset: 0x20-0x24
}	GPIO_RegDef_t;


/*
 *	Defining the registers for RCC
 */
typedef struct
{
	__vo uint32_t CR;							// Address offset: 0x00
	__vo uint32_t PLLCFGR;						// Address offset: 0x04
	__vo uint32_t CFGR;							// Address offset: 0x08
	__vo uint32_t CIR;							// Address offset: 0x0C
	__vo uint32_t AHB1RSTR;						// Address offset: 0x10
	__vo uint32_t AHB2RSTR;						// Address offset: 0x14
	__vo uint32_t AHB3RSTR;						// Address offset: 0x18
	__vo uint32_t RESERVED;						// Address offset: 0x1C
	__vo uint32_t APB1RSTR;						// Address offset: 0x20
	__vo uint32_t APB2RSTR;						// Address offset: 0x24
	__vo uint32_t RESERVED2;					// Address offset: 0x28
	__vo uint32_t RESERVED3;					// Address offset: 0x2C
	__vo uint32_t AHB1ENR;						// Address offset: 0x30
	__vo uint32_t AHB2ENR;						// Address offset: 0x34
	__vo uint32_t AHB3ENR;						// Address offset: 0x38
	__vo uint32_t RESERVED4;					// Address offset: 0x3C
	__vo uint32_t APB1ENR;						// Address offset: 0x40
	__vo uint32_t APB2ENR;						// Address offset: 0x44
	__vo uint32_t RESERVED5;					// Address offset: 0x48
	__vo uint32_t RESERVED6;					// Address offset: 0x4C
	__vo uint32_t AHB1LPENR;					// Address offset: 0x50
	__vo uint32_t AHB2LPENR;					// Address offset: 0x54
	__vo uint32_t AHB3LPENR;					// Address offset: 0x58
	__vo uint32_t RESERVED7;					// Address offset: 0x5C
	__vo uint32_t APB1LPENR;					// Address offset: 0x60
	__vo uint32_t APB2LPENR;					// Address offset: 0x64
	__vo uint32_t RESERVED8;					// Address offset: 0x68
	__vo uint32_t RESERVED9;					// Address offset: 0x6C
	__vo uint32_t BDCR;							// Address offset: 0x70
	__vo uint32_t CSR;							// Address offset: 0x74
	__vo uint32_t RESERVED10;					// Address offset: 0x78
	__vo uint32_t RESERVED11;					// Address offset: 0x7C
	__vo uint32_t SSCGR;						// Address offset: 0x80
	__vo uint32_t PLLI2SCFGR;					// Address offset: 0x84
	__vo uint32_t PLLSAICFGR;					// Address offset: 0x88
	__vo uint32_t DCKCFGR;						// Address offset: 0x8C
	__vo uint32_t CKGATENR;						// Address offset: 0x90
	__vo uint32_t DCKCFGR2;						// Address offset: 0x94
}	RCC_RegDef_t;


/*
 *	Defining the registers for EXTI
 */
typedef struct
{
	__vo uint32_t IMR;						// Address offset 0x00
	__vo uint32_t EMR;						// Address offset 0x04
	__vo uint32_t RTSR;						// Address offset 0x08
	__vo uint32_t FTSR;						// Address offset 0x0C
	__vo uint32_t SWIER;					// Address offset 0x10
	__vo uint32_t PR;						// Address offset 0x14
} 	EXTI_RegDef_t;


/*
 *	Defining the registers for SYSCFG
 */
typedef struct
{
	__vo uint32_t MEMRMP;					// Address offset 0x00
	__vo uint32_t PMC;						// Address offset 0x04
	__vo uint32_t EXTICR[4];				// Address offset 0x08 - 0x14
	__vo uint32_t RESERVED1[2];				// Address offset 0x18 - 0x1C
	__vo uint32_t CMPCR;					// Address offset 0x20
	__vo uint32_t RESERVED2[2];				// Address offset 0x24 - 0x28
	__vo uint32_t MCFGR;					// Address offset 0x2C
} 	SYSCFG_RegDet_t;


typedef struct
{
	__vo uint32_t CR1;								// Address offset: 0x00
	__vo uint32_t CR2;								// Address offset: 0x04
	__vo uint32_t SR;								// Address offset: 0x08
	__vo uint32_t DR;								// Address offset: 0x0C
	__vo uint32_t CRCPR;							// Address offset: 0x10
	__vo uint32_t RXCRCR;							// Address offset: 0x14
	__vo uint32_t TXCRCR;							// Address offset: 0x18
	__vo uint32_t I2SCFGR;							// Address offset: 0x1C
	__vo uint32_t I2SPR;							// Address offset: 0x20
}	SPI_RegDef_t;

#define GPIOA 								((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB 								((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC 								((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD 								((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE 								((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF 								((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG 								((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH 								((GPIO_RegDef_t*)GPIOH_BASEADDR)

#define RCC									((RCC_RegDef_t*) RCC_BASEADDR)

#define EXTI								((EXTI_RegDef_t*) EXTI_BASEADDR)

#define SYSCFG								((SYSCFG_RegDet_t*) SYSCFG_BASEADDR)

#define SPI1								((SPI_RegDef_t*) SPI1_BASEADDR)
#define SPI2								((SPI_RegDef_t*) SPI2_BASEADDR)
#define SPI3								((SPI_RegDef_t*) SPI3_BASEADDR)
#define SPI4								((SPI_RegDef_t*) SPI4_BASEADDR)


extern GPIO_RegDef_t *pGPIOA;
extern GPIO_RegDef_t *pGPIOB;
extern GPIO_RegDef_t *pGPIOC;
extern GPIO_RegDef_t *pGPIOD;
extern GPIO_RegDef_t *pGPIOE;
extern GPIO_RegDef_t *pGPIOF;
extern GPIO_RegDef_t *pGPIOG;
extern GPIO_RegDef_t *pGPIOH;
extern RCC_RegDef_t *pRCC;
extern EXTI_RegDef_t *pEXTI;
extern SYSCFG_RegDet_t *pSYSCFG;
extern SPI_RegDef_t *pSPI1;
extern SPI_RegDef_t *pSPI2;
extern SPI_RegDef_t *pSPI3;
extern SPI_RegDef_t *pSPI4;

//////////////////////////////////////////////////////////////	 ENABLERS   ///////////////////////////////////////////////////
/*
 * Clock Enable Macros for GPIOx peripherals
 */
#define GPIOA_PCLCK_EN()			(pRCC->AHB1ENR |= (1<<0))
#define GPIOB_PCLCK_EN()			(pRCC->AHB1ENR |= (1<<1))
#define GPIOC_PCLCK_EN()			(pRCC->AHB1ENR |= (1<<2))
#define GPIOD_PCLCK_EN()			(pRCC->AHB1ENR |= (1<<3))
#define GPIOE_PCLCK_EN()			(pRCC->AHB1ENR |= (1<<4))
#define GPIOF_PCLCK_EN()			(pRCC->AHB1ENR |= (1<<5))
#define GPIOG_PCLCK_EN()			(pRCC->AHB1ENR |= (1<<6))
#define GPIOH_PCLCK_EN()			(pRCC->AHB1ENR |= (1<<7))


/*
 * Clock Enable Macros for I2Cx peripherals
 */
#define I2C1_PCLCK_EN()				(pRCC->APB1ENR |= (1<<21))
#define I2C2_PCLCK_EN()				(pRCC->APB1ENR |= (1<<22))
#define I2C3_PCLCK_EN()				(pRCC->APB1ENR |= (1<<23))


/*
 * Clock Enable Macros for SPIx peripherals
 */
#define SPI1_PCLCK_EN()				(pRCC->APB2ENR |= (1<<12))
#define SPI2_PCLCK_EN()				(pRCC->APB1ENR |= (1<<14))
#define SPI3_PCLCK_EN()				(pRCC->APB1ENR |= (1<<15))
#define SPI4_PCLCK_EN()				(pRCC->APB2ENR |= (1<<13))


/*
 * Clock Enable Macros for USARTx peripherals
 */
#define USART1_PCLCK_EN()			(pRCC->APB2ENR |= (1<<4))
#define USART2_PCLCK_EN()			(pRCC->APB1ENR |= (1<<17))
#define USART3_PCLCK_EN()			(pRCC->APB1ENR |= (1<<18))
#define USART6_PCLCK_EN()			(pRCC->APB2ENR |= (1<<5))

/*
 * Clock Enable Macros for UARTx peripherals
 */
#define UART4_PCLCK_EN()			(pRCC->APB1ENR |= (1<<19))
#define UART5_PCLCK_EN()			(pRCC->APB1ENR |= (1<<20))


/*
 * Clock Enable Macros for SYSCFG peripherals
 */
#define SYSCFG_PCLCK_EN()			(pRCC->APB2ENR |= (1<<14))


//////////////////////////////////////////////////////////////	 DISABLERS   ///////////////////////////////////////////////////
/*
 * Clock Disable Macros for GPIOx peripherals
 */
#define GPIOA_PCLCK_DI()			(pRCC->AHB1ENR &= ~(1<<0))
#define GPIOB_PCLCK_DI()			(pRCC->AHB1ENR &= ~(1<<1))
#define GPIOC_PCLCK_DI()			(pRCC->AHB1ENR &= ~(1<<2))
#define GPIOD_PCLCK_DI()			(pRCC->AHB1ENR &= ~(1<<3))
#define GPIOE_PCLCK_DI()			(pRCC->AHB1ENR &= ~(1<<4))
#define GPIOF_PCLCK_DI()			(pRCC->AHB1ENR &= ~(1<<5))
#define GPIOG_PCLCK_DI()			(pRCC->AHB1ENR &= ~(1<<6))
#define GPIOH_PCLCK_DI()			(pRCC->AHB1ENR &= ~(1<<7))


/*
 * Clock Disable Macros for I2Cx peripherals
 */
#define I2C1_PCLCK_DI()				(pRCC->APB1ENR &= ~(1<<21))
#define I2C2_PCLCK_DI()				(pRCC->APB1ENR &= ~(1<<22))
#define I2C3_PCLCK_DI()				(pRCC->APB1ENR &= ~(1<<23))


/*
 * Clock Disable Macros for SPIx peripherals
 */
#define SPI1_PCLCK_DI()				(pRCC->APB2ENR &= ~(1<<12))
#define SPI2_PCLCK_DI()				(pRCC->APB1ENR &= ~(1<<14))
#define SPI3_PCLCK_DI()				(pRCC->APB1ENR &= ~(1<<15))
#define SPI4_PCLCK_DI()				(pRCC->APB2ENR &= ~(1<<13))


/*
 * Clock Disable Macros for USARTx peripherals
 */
#define USART1_PCLCK_DI()			(pRCC->APB2ENR &= ~(1<<4))
#define USART2_PCLCK_DI()			(pRCC->APB1ENR &= ~(1<<17))
#define USART3_PCLCK_DI()			(pRCC->APB1ENR &= ~(1<<18))
#define USART6_PCLCK_DI()			(pRCC->APB2ENR &= ~(1<<5))

/*
 * Clock Disable Macros for UARTx peripherals
 */
#define UART4_PCLCK_DI()			(pRCC->APB1ENR &= ~(1<<19))
#define UART5_PCLCK_DI()			(pRCC->APB1ENR &= ~(1<<20))


/*
 * Clock Disable Macros for SYSCFG peripherals
 */
#define SYSCFG_PCLCK_DI()			(pRCC->APB2ENR &= ~(1<<14))


//////////////////////////////////////////////////////////////	 RESETER   //////////////////////////////////////////////////
/*
 * Clock Reset Macros for GPIOx peripherals
 */
#define GPIOA_REG_RESET()			do{ (pRCC->AHB1RSTR |= (1<<0)); (pRCC->AHB1RSTR &= ~(1<<0)); } while(0)
#define GPIOB_REG_RESET()			do{ (pRCC->AHB1RSTR |= (1<<1)); (pRCC->AHB1RSTR &= ~(1<<1)); } while(0)
#define GPIOC_REG_RESET()			do{ (pRCC->AHB1RSTR |= (1<<2)); (pRCC->AHB1RSTR &= ~(1<<2)); } while(0)
#define GPIOD_REG_RESET()			do{ (pRCC->AHB1RSTR |= (1<<3)); (pRCC->AHB1RSTR &= ~(1<<3)); } while(0)
#define GPIOE_REG_RESET()			do{ (pRCC->AHB1RSTR |= (1<<4)); (pRCC->AHB1RSTR &= ~(1<<4)); } while(0)
#define GPIOF_REG_RESET()			do{ (pRCC->AHB1RSTR |= (1<<5)); (pRCC->AHB1RSTR &= ~(1<<5)); } while(0)
#define GPIOG_REG_RESET()			do{ (pRCC->AHB1RSTR |= (1<<6)); (pRCC->AHB1RSTR &= ~(1<<6)); } while(0)
#define GPIOH_REG_RESET()			do{ (pRCC->AHB1RSTR |= (1<<7)); (pRCC->AHB1RSTR &= ~(1<<7)); } while(0)


/*
 * Clock Reset Macros for I2Cx peripherals
 */
#define I2C1_REG_RESET()			do{ (pRCC->APB1RSTR |= (1<<21)); (pRCC->APB1RSTR &= ~(1<<21)); } while(0)
#define I2C2_REG_RESET()			do{ (pRCC->APB1RSTR |= (1<<22)); (pRCC->APB1RSTR &= ~(1<<22)); } while(0)
#define I2C3_REG_RESET()			do{ (pRCC->APB1RSTR |= (1<<23)); (pRCC->APB1RSTR &= ~(1<<23)); } while(0)


/*
 * Clock Reset Macros for SPIx peripherals
 */
#define SPI1_REG_RESET()			do{ (pRCC->APB2RSTR |= (1<<12)); (pRCC->APB2RSTR &= ~(1<<12)); } while(0)
#define SPI2_REG_RESET()			do{ (pRCC->APB1RSTR |= (1<<14)); (pRCC->APB1RSTR &= ~(1<<14)); } while(0)
#define SPI3_REG_RESET()			do{ (pRCC->APB1RSTR |= (1<<15)); (pRCC->APB1RSTR &= ~(1<<15)); } while(0)
#define SPI4_REG_RESET()			do{ (pRCC->APB2RSTR |= (1<<13)); (pRCC->APB2RSTR &= ~(1<<13)); } while(0)


/*
 * Clock Reset Macros for USARTx peripherals
 */
#define USART1_REG_RESET()			do{ (pRCC->APB2RSTR |= (1<<4)); (pRCC->APB2RSTR&= ~(1<<4)); } while(0)
#define USART2_REG_RESET()			do{ (pRCC->APB1RSTR |= (1<<17)); (pRCC->APB1RSTR &= ~(1<<17)); } while(0)
#define USART3_REG_RESET()			do{ (pRCC->APB1RSTR |= (1<<18)); (pRCC->APB1RSTR &= ~(1<<18)); } while(0)
#define USART6_REG_RESET()			do{ (pRCC->APB2RSTR |= (1<<5)); (pRCC->APB2RSTR&= ~(1<<5)); } while(0)

/*
 * Clock Reset Macros for UARTx peripherals
 */
#define UART4_REG_RESET()			do{ (pRCC->APB1RSTR |= (1<<19)); (pRCC->APB1RSTR &= ~(1<<19)); } while(0)
#define UART5_REG_RESET()			do{ (pRCC->APB1RSTR |= (1<<20)); (pRCC->APB1RSTR &= ~(1<<20)); } while(0)


/*
 * Clock Reset Macros for SYSCFG peripherals
 */
#define SYSCFG_REG_RESET()			do{ (pRCC->APB2RSTR |= (1<<14)); (pRCC->APB2RSTR &= ~(1<<14)); } while(0)



#endif /* INC_STM32F446RE_H_ */
