/*
 * stm32f407xx.h
 *
 *  Created on: Jan 8, 2025
 *      Author: Yunus
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>
#include <string.h>
#include <stddef.h>

/*
 * Microprocessor Defines
 *
 */
#define NVIC_ISER0						((uint32_t *)(0xE000E100))


#define __IO volatile

#define SET_BIT(REG,BIT)				( (REG) |=  (BIT) )
#define CLEAR_BIT(REG,BIT)				( (REG) &= ~(BIT) )
#define READ_BIT(REG,BIT)				( (REG) &   (BIT) )
#define UNUSED(x)						(void)x

typedef enum
{
	DISABLE = 0x0U,
	ENABLE= !DISABLE

}FunctionalState_t;

/*
 * IRQ Numbers of MCU == vector table
 *
 */
typedef enum
{
	EXTI0_IRQNumber = 6,
	EXTI1_IRQNumber = 7,
	EXTI2_IRQNumber = 8,
	EXTI3_IRQNumber = 9,
	SPI1_IRQNumber = 35

}IRQNumber_TypeDef_t;

/*
 * Memory Base Address
 *
 */

#define FLASH_BASE_ADDR					(0x08000000UL)				/* Flash Base Address 1MB */
#define SRAM1_BASE_ADDR					(0x20000000UL)				/* SRAM1 Base Address 1MB */
#define SRAM2_BASE_ADDR					(0x2001C000UL)				/* SRAM2 Base Address 1MB */


/*
 * Peripheral Base Addresses
 *
 */
#define PERIPH_BASE_ADDR				(0x40000000UL)							/* Base Address For All Peripherals	*/

#define APB1_BASE_ADDR					PERIPH_BASE_ADDR						/* APB1 Bus Domain Base Address		*/
#define APB2_BASE_ADDR					(PERIPH_BASE_ADDR + 0x00010000UL)		/* APB2 Bus Domain Base Address		*/
#define AHB1_BASE_ADDR					(PERIPH_BASE_ADDR + 0x00020000UL)		/* AHB1 Bus Domain Base Address		*/
#define AHB2_BASE_ADDR					(PERIPH_BASE_ADDR + 0x10000000UL)		/* AHB1 Bus Domain Base Address		*/

/*
 * APB1 Peripherals Base Addresses
 *
 */

#define TIM2_BASE_ADDR					(APB1_BASE_ADDR + 0x0000UL)
#define TIM3_BASE_ADDR					(APB1_BASE_ADDR + 0x0400UL)
#define TIM4_BASE_ADDR					(APB1_BASE_ADDR + 0x0800UL)
#define TIM5_BASE_ADDR					(APB1_BASE_ADDR + 0x0C00UL)
#define TIM6_BASE_ADDR					(APB1_BASE_ADDR + 0x1000UL)
#define TIM7_BASE_ADDR					(APB1_BASE_ADDR + 0x1400UL)
#define TIM12_BASE_ADDR					(APB1_BASE_ADDR + 0x1800UL)
#define TIM13_BASE_ADDR					(APB1_BASE_ADDR + 0x1C00UL)
#define TIM14_BASE_ADDR					(APB1_BASE_ADDR + 0x2000UL)


#define SPI2_BASE_ADDR					(APB1_BASE_ADDR + 0x3800UL)
#define SPI3_BASE_ADDR					(APB1_BASE_ADDR + 0x3C00UL)

#define USART2_BASE_ADDR				(APB1_BASE_ADDR + 0x4400UL)
#define USART3_BASE_ADDR				(APB1_BASE_ADDR + 0x4800UL)
#define UART4_BASE_ADDR					(APB1_BASE_ADDR + 0x4C00UL)
#define UART5_BASE_ADDR					(APB1_BASE_ADDR + 0x5000UL)
#define UART7_BASE_ADDR					(APB1_BASE_ADDR + 0x7800UL)
#define UART8_BASE_ADDR					(APB1_BASE_ADDR + 0x7C00UL)

#define I2C1_BASE_ADDR					(APB1_BASE_ADDR + 0x5400UL)
#define I2C2_BASE_ADDR					(APB1_BASE_ADDR + 0x5800UL)
#define I2C3_BASE_ADDR					(APB1_BASE_ADDR + 0x5C00UL)


/*
 * APB2 Peripherals Base Addresses
 *
 */


#define TIM1_BASE_ADDR					(APB2_BASE_ADDR + 0x0000UL)
#define TIM8_BASE_ADDR					(APB2_BASE_ADDR + 0x0400UL)
#define TIM9_BASE_ADDR					(APB2_BASE_ADDR + 0x4000UL)
#define TIM10_BASE_ADDR					(APB2_BASE_ADDR + 0x4400UL)
#define TIM11_BASE_ADDR					(APB2_BASE_ADDR + 0x4800UL)


#define USART1_BASE_ADDR				(APB2_BASE_ADDR + 0x1000UL)
#define USART6_BASE_ADDR				(APB2_BASE_ADDR + 0x1400UL)

#define SPI1_BASE_ADDR					(APB2_BASE_ADDR + 0x3000UL)
#define SPI4_BASE_ADDR					(APB2_BASE_ADDR + 0x3400UL)
#define SPI5_BASE_ADDR					(APB2_BASE_ADDR + 0x5000UL)
#define SPI6_BASE_ADDR					(APB2_BASE_ADDR + 0x5400UL)

#define SYSCFG_BASE_ADDR				(APB2_BASE_ADDR + 0x3800UL)

#define EXTI_BASE_ADDR					(APB2_BASE_ADDR + 0x3C00UL)

/*
 * AHB1 Peripherals Base Addresses
 *
 */

#define GPIOA_BASE_ADDR					(AHB1_BASE_ADDR + 0x0000UL)
#define GPIOB_BASE_ADDR					(AHB1_BASE_ADDR + 0x0400UL)
#define GPIOC_BASE_ADDR					(AHB1_BASE_ADDR + 0x0800UL)
#define GPIOD_BASE_ADDR					(AHB1_BASE_ADDR + 0x0C00UL)
#define GPIOE_BASE_ADDR					(AHB1_BASE_ADDR + 0x1000UL)
#define GPIOF_BASE_ADDR					(AHB1_BASE_ADDR + 0x1400UL)
#define GPIOG_BASE_ADDR					(AHB1_BASE_ADDR + 0x1800UL)
#define GPIOH_BASE_ADDR					(AHB1_BASE_ADDR + 0x1C00UL)
#define GPIOI_BASE_ADDR					(AHB1_BASE_ADDR + 0x2000UL)
#define GPIOJ_BASE_ADDR					(AHB1_BASE_ADDR + 0x2400UL)
#define GPIOK_BASE_ADDR					(AHB1_BASE_ADDR + 0x2800UL)

#define CRC_BASE_ADDR					(AHB1_BASE_ADDR + 0x3000UL)
#define RCC_BASE_ADDR					(AHB1_BASE_ADDR + 0x3800UL)

#define USB_OTG_HS_BASE_ADDR			(AHB1_BASE_ADDR + 0x20000UL)

/*
 * AHB2 Peripherals Base Addresses
 *
 */

#define USB_OTG_HS_BASE_ADDR			(AHB2_BASE_ADDR + 0x0000UL)


/*
 * Peripheral Structure Definitions
 *
 */

typedef struct
{
	__IO uint32_t MODER;
	__IO uint32_t OTYPER;
	__IO uint32_t OSPEEDR;
	__IO uint32_t PUPDR;
	__IO uint32_t IDR;
	__IO uint32_t ODR;
	__IO uint32_t BSRR;
	__IO uint32_t LCKR;
	__IO uint32_t AFR[2];
}GPIO_TypeDef_t;

typedef struct
{
	__IO uint32_t CR;
	__IO uint32_t PLLCFGR;
	__IO uint32_t CFGR;
	__IO uint32_t CIR;
	__IO uint32_t AHB1RSTR;
	__IO uint32_t AHB2RSTR;
	__IO uint32_t AHB3RSTR;
	__IO uint32_t RESERVED0;
	__IO uint32_t APB1RSTR;
	__IO uint32_t APB2RSTR;
	__IO uint32_t RESERVED1[2];
	__IO uint32_t AHB1ENR;
	__IO uint32_t AHB2ENR;
	__IO uint32_t AHB3ENR;
	__IO uint32_t RESERVED2;
	__IO uint32_t APB1ENR;
	__IO uint32_t APB2ENR;
	__IO uint32_t RESERVED3[2];
	__IO uint32_t AHB1LPENR;
	__IO uint32_t AHB2LPENR;
	__IO uint32_t AHB3LPENR;
	__IO uint32_t RESERVED4;
	__IO uint32_t APB1LPENR;
	__IO uint32_t APB2LPENR;
	__IO uint32_t RESERVED5[2];
	__IO uint32_t BDCR;
	__IO uint32_t CSR;
	__IO uint32_t RESERVED6[2];
	__IO uint32_t SSCGR;
	__IO uint32_t PLLI2SCFGR;
	__IO uint32_t PLLSAICFGR;
	__IO uint32_t DCKCFGR;
}RCC_TypeDef_t;

typedef struct
{
	__IO uint32_t MEMRMP;
	__IO uint32_t PMC;
	__IO uint32_t CR[4];
	__IO uint32_t CMPCR;

}SYSCFG_TypeDef_t;

typedef struct
{
	__IO uint32_t IMR;
	__IO uint32_t EMR;
	__IO uint32_t RTSR;
	__IO uint32_t FTSR;
	__IO uint32_t SWIER;
	__IO uint32_t PR;
}EXTI_TypeDef_t;

typedef struct
{
	__IO uint32_t CR1;
	__IO uint32_t CR2;
	__IO uint32_t SR;
	__IO uint32_t DR;
	__IO uint32_t CRCPR;
	__IO uint32_t RXCRCR;
	__IO uint32_t TXCRCR;
	__IO uint32_t I2SCFGR;
	__IO uint32_t I2SPR;
}SPI_TypeDef_t;

/**
  * @brief USB_OTG_Core_Registers
  */
typedef struct
{
  __IO uint32_t GOTGCTL;              /*!< USB_OTG Control and Status Register          000h */
  __IO uint32_t GOTGINT;              /*!< USB_OTG Interrupt Register                   004h */
  __IO uint32_t GAHBCFG;              /*!< Core AHB Configuration Register              008h */
  __IO uint32_t GUSBCFG;              /*!< Core USB Configuration Register              00Ch */
  __IO uint32_t GRSTCTL;              /*!< Core Reset Register                          010h */
  __IO uint32_t GINTSTS;              /*!< Core Interrupt Register                      014h */
  __IO uint32_t GINTMSK;              /*!< Core Interrupt Mask Register                 018h */
  __IO uint32_t GRXSTSR;              /*!< Receive Sts Q Read Register                  01Ch */
  __IO uint32_t GRXSTSP;              /*!< Receive Sts Q Read & POP Register            020h */
  __IO uint32_t GRXFSIZ;              /*!< Receive FIFO Size Register                   024h */
  __IO uint32_t DIEPTXF0_HNPTXFSIZ;   /*!< EP0 / Non Periodic Tx FIFO Size Register     028h */
  __IO uint32_t HNPTXSTS;             /*!< Non Periodic Tx FIFO/Queue Sts reg           02Ch */
  uint32_t Reserved30[2];             /*!< Reserved                                     030h */
  __IO uint32_t GCCFG;                /*!< General Purpose IO Register                  038h */
  __IO uint32_t CID;                  /*!< User ID Register                             03Ch */
  uint32_t  Reserved40[48];           /*!< Reserved                                0x40-0xFF */
  __IO uint32_t HPTXFSIZ;             /*!< Host Periodic Tx FIFO Size Reg               100h */
  __IO uint32_t DIEPTXF[0x0F];        /*!< dev Periodic Transmit FIFO                        */
} USB_OTG_GlobalTypeDef;

/**
  * @brief USB_OTG_device_Registers
  */
typedef struct
{
  __IO uint32_t DCFG;            /*!< dev Configuration Register   800h */
  __IO uint32_t DCTL;            /*!< dev Control Register         804h */
  __IO uint32_t DSTS;            /*!< dev Status Register (RO)     808h */
  uint32_t Reserved0C;           /*!< Reserved                     80Ch */
  __IO uint32_t DIEPMSK;         /*!< dev IN Endpoint Mask         810h */
  __IO uint32_t DOEPMSK;         /*!< dev OUT Endpoint Mask        814h */
  __IO uint32_t DAINT;           /*!< dev All Endpoints Itr Reg    818h */
  __IO uint32_t DAINTMSK;        /*!< dev All Endpoints Itr Mask   81Ch */
  uint32_t  Reserved20;          /*!< Reserved                     820h */
  uint32_t Reserved9;            /*!< Reserved                     824h */
  __IO uint32_t DVBUSDIS;        /*!< dev VBUS discharge Register  828h */
  __IO uint32_t DVBUSPULSE;      /*!< dev VBUS Pulse Register      82Ch */
  __IO uint32_t DTHRCTL;         /*!< dev threshold                830h */
  __IO uint32_t DIEPEMPMSK;      /*!< dev empty msk                834h */
  __IO uint32_t DEACHINT;        /*!< dedicated EP interrupt       838h */
  __IO uint32_t DEACHMSK;        /*!< dedicated EP msk             83Ch */
  uint32_t Reserved40;           /*!< dedicated EP mask            840h */
  __IO uint32_t DINEP1MSK;       /*!< dedicated EP mask            844h */
  uint32_t  Reserved44[15];      /*!< Reserved                 844-87Ch */
  __IO uint32_t DOUTEP1MSK;      /*!< dedicated EP msk             884h */
} USB_OTG_DeviceTypeDef;

/**
  * @brief USB_OTG_IN_Endpoint-Specific_Register
  */
typedef struct
{
  __IO uint32_t DIEPCTL;           /*!< dev IN Endpoint Control Reg    900h + (ep_num * 20h) + 00h */
  uint32_t Reserved04;             /*!< Reserved                       900h + (ep_num * 20h) + 04h */
  __IO uint32_t DIEPINT;           /*!< dev IN Endpoint Itr Reg        900h + (ep_num * 20h) + 08h */
  uint32_t Reserved0C;             /*!< Reserved                       900h + (ep_num * 20h) + 0Ch */
  __IO uint32_t DIEPTSIZ;          /*!< IN Endpoint Txfer Size         900h + (ep_num * 20h) + 10h */
  __IO uint32_t DIEPDMA;           /*!< IN Endpoint DMA Address Reg    900h + (ep_num * 20h) + 14h */
  __IO uint32_t DTXFSTS;           /*!< IN Endpoint Tx FIFO Status Reg 900h + (ep_num * 20h) + 18h */
  uint32_t Reserved18;             /*!< Reserved  900h+(ep_num*20h)+1Ch-900h+ (ep_num * 20h) + 1Ch */
} USB_OTG_INEndpointTypeDef;

/**
  * @brief USB_OTG_OUT_Endpoint-Specific_Registers
  */
typedef struct
{
  __IO uint32_t DOEPCTL;       /*!< dev OUT Endpoint Control Reg           B00h + (ep_num * 20h) + 00h */
  uint32_t Reserved04;         /*!< Reserved                               B00h + (ep_num * 20h) + 04h */
  __IO uint32_t DOEPINT;       /*!< dev OUT Endpoint Itr Reg               B00h + (ep_num * 20h) + 08h */
  uint32_t Reserved0C;         /*!< Reserved                               B00h + (ep_num * 20h) + 0Ch */
  __IO uint32_t DOEPTSIZ;      /*!< dev OUT Endpoint Txfer Size            B00h + (ep_num * 20h) + 10h */
  __IO uint32_t DOEPDMA;       /*!< dev OUT Endpoint DMA Address           B00h + (ep_num * 20h) + 14h */
  uint32_t Reserved18[2];      /*!< Reserved B00h + (ep_num * 20h) + 18h - B00h + (ep_num * 20h) + 1Ch */
} USB_OTG_OUTEndpointTypeDef;

/**
  * @brief USB_OTG_Host_Mode_Register_Structures
  */
typedef struct
{
  __IO uint32_t HCFG;             /*!< Host Configuration Register          400h */
  __IO uint32_t HFIR;             /*!< Host Frame Interval Register         404h */
  __IO uint32_t HFNUM;            /*!< Host Frame Nbr/Frame Remaining       408h */
  uint32_t Reserved40C;           /*!< Reserved                             40Ch */
  __IO uint32_t HPTXSTS;          /*!< Host Periodic Tx FIFO/ Queue Status  410h */
  __IO uint32_t HAINT;            /*!< Host All Channels Interrupt Register 414h */
  __IO uint32_t HAINTMSK;         /*!< Host All Channels Interrupt Mask     418h */
} USB_OTG_HostTypeDef;

/**
  * @brief USB_OTG_Host_Channel_Specific_Registers
  */
typedef struct
{
  __IO uint32_t HCCHAR;           /*!< Host Channel Characteristics Register    500h */
  __IO uint32_t HCSPLT;           /*!< Host Channel Split Control Register      504h */
  __IO uint32_t HCINT;            /*!< Host Channel Interrupt Register          508h */
  __IO uint32_t HCINTMSK;         /*!< Host Channel Interrupt Mask Register     50Ch */
  __IO uint32_t HCTSIZ;           /*!< Host Channel Transfer Size Register      510h */
  __IO uint32_t HCDMA;            /*!< Host Channel DMA Address Register        514h */
  uint32_t Reserved[2];           /*!< Reserved                                      */
} USB_OTG_HostChannelTypeDef;



#define GPIOA							( (GPIO_TypeDef_t *)(GPIOA_BASE_ADDR) )
#define GPIOB							( (GPIO_TypeDef_t *)(GPIOB_BASE_ADDR) )
#define GPIOC							( (GPIO_TypeDef_t *)(GPIOC_BASE_ADDR) )
#define GPIOD							( (GPIO_TypeDef_t *)(GPIOD_BASE_ADDR) )
#define GPIOE							( (GPIO_TypeDef_t *)(GPIOE_BASE_ADDR) )
#define GPIOF							( (GPIO_TypeDef_t *)(GPIOF_BASE_ADDR) )
#define GPIOG							( (GPIO_TypeDef_t *)(GPIOG_BASE_ADDR) )
#define GPIOH							( (GPIO_TypeDef_t *)(GPIOH_BASE_ADDR) )
#define GPIOI							( (GPIO_TypeDef_t *)(GPIOI_BASE_ADDR) )
#define GPIOJ							( (GPIO_TypeDef_t *)(GPIOJ_BASE_ADDR) )
#define GPIOK							( (GPIO_TypeDef_t *)(GPIOK_BASE_ADDR) )

#define RCC								( (RCC_TypeDef_t  *)(RCC_BASE_ADDR	) )

#define SYSCFG							((SYSCFG_TypeDef_t*)(SYSCFG_BASE_ADDR))

#define EXTI							((EXTI_TypeDef_t  *)(EXTI_BASE_ADDR ) )

#define SPI1							((SPI_TypeDef_t   *)(SPI1_BASE_ADDR ) )
#define SPI2							((SPI_TypeDef_t   *)(SPI2_BASE_ADDR ) )
#define SPI3							((SPI_TypeDef_t   *)(SPI3_BASE_ADDR ) )
#define SPI4							((SPI_TypeDef_t   *)(SPI4_BASE_ADDR ) )
#define SPI5							((SPI_TypeDef_t   *)(SPI5_BASE_ADDR ) )
#define SPI6							((SPI_TypeDef_t   *)(SPI6_BASE_ADDR ) )

/*
 * Bit Definitions
 *
 */

#define RCC_AHB1ENR_GPIOAEN_POS			(0U)
#define RCC_AHB1ENR_GPIOAEN_MSK			(0x1 << RCC_AHB1ENR_GPIOAEN_POS)
#define RCC_AHB1ENR_GPIOAEN				RCC_AHB1ENR_GPIOAEN_MSK

#define RCC_AHB1ENR_GPIOBEN_POS			(1U)
#define RCC_AHB1ENR_GPIOBEN_MSK			(0x1 << RCC_AHB1ENR_GPIOBEN_POS)
#define RCC_AHB1ENR_GPIOBEN				RCC_AHB1ENR_GPIOBEN_MSK

#define RCC_AHB1ENR_GPIOCEN_POS			(2U)
#define RCC_AHB1ENR_GPIOCEN_MSK			(0x1 << RCC_AHB1ENR_GPIOCEN_POS)
#define RCC_AHB1ENR_GPIOCEN				RCC_AHB1ENR_GPIOCEN_MSK

#define RCC_AHB1ENR_GPIODEN_POS			(3U)
#define RCC_AHB1ENR_GPIODEN_MSK			(0x1 << RCC_AHB1ENR_GPIODEN_POS)
#define RCC_AHB1ENR_GPIODEN				RCC_AHB1ENR_GPIODEN_MSK

#define RCC_AHB1ENR_GPIOEEN_POS			(4U)
#define RCC_AHB1ENR_GPIOEEN_MSK			(0x1 << RCC_AHB1ENR_GPIOEEN_POS)
#define RCC_AHB1ENR_GPIOEEN				RCC_AHB1ENR_GPIOEEN_MSK

#define RCC_AHB1ENR_GPIOFEN_POS			(5U)
#define RCC_AHB1ENR_GPIOFEN_MSK			(0x1 << RCC_AHB1ENR_GPIOFEN_POS)
#define RCC_AHB1ENR_GPIOFEN				RCC_AHB1ENR_GPIOFEN_MSK


#define RCC_APB2ENR_SYSCFGEN_POS		(14U)
#define RCC_APB2ENR_SYSCFGEN_MSK		(0x1 << RCC_APB2ENR_SYSCFGEN_POS)
#define RCC_APB2ENR_SYSCFGEN			RCC_APB2ENR_SYSCFGEN_MSK

#define RCC_APB2ENR_SPI1EN_POS			(12U)
#define RCC_APB2ENR_SPI1EN_MSK			(0x1 << RCC_APB2ENR_SPI1EN_POS)
#define RCC_APB2ENR_SPI1EN				RCC_APB2ENR_SPI1EN_MSK

#define RCC_APB1ENR_SPI2EN_POS			(14U)
#define RCC_APB1ENR_SPI2EN_MSK			(0x1 << RCC_APB1ENR_SPI2EN_POS)
#define RCC_APB1ENR_SPI2EN				RCC_APB1ENR_SPI2EN_MSK

#define RCC_APB1ENR_SPI3EN_POS			(15U)
#define RCC_APB1ENR_SPI3EN_MSK			(0x1 << RCC_APB1ENR_SPI3EN_POS)
#define RCC_APB1ENR_SPI3EN				RCC_APB1ENR_SPI3EN_MSK

#define RCC_APB2ENR_SPI4EN_POS			(13U)
#define RCC_APB2ENR_SPI4EN_MSK			(0x1 << RCC_APB2ENR_SPI4EN_POS)
#define RCC_APB2ENR_SPI4EN				RCC_APB2ENR_SPI4EN_MSK

#define RCC_APB2ENR_SPI5EN_POS			(20U)
#define RCC_APB2ENR_SPI5EN_MSK			(0x1 << RCC_APB2ENR_SPI5EN_POS)
#define RCC_APB2ENR_SPI5EN				RCC_APB2ENR_SPI5EN_MSK

#define RCC_APB2ENR_SPI6EN_POS			(21U)
#define RCC_APB2ENR_SPI6EN_MSK			(0x1 << RCC_APB2ENR_SPI6EN_POS)
#define RCC_APB2ENR_SPI6EN				RCC_APB2ENR_SPI6EN_MSK

#define SPI_SR_Busy						(7U)
#define SPI_SR_TxE						(1U)
#define SPI_SR_RNxE						(0U)

#define SPI_CR1_SPE						(6U)
#define SPI_CR1_DFF						(11U)

#define SPI_CR2_TXEIE					(7U)


/*
 * Flag Definitions
 *
 */

#define SPI_TxE_FLAG					(0x1U << SPI_SR_TxE)
#define SPI_Busy_FLAG					(0x1U << SPI_SR_Busy)
#define SPI_RxNE_FLAG					(0x1U << SPI_SR_RNxE)

#include "RCC.h"
#include "GPIO.h"
#include "EXTI.h"
#include "SPI.h"


#endif /* INC_STM32F407XX_H_ */
