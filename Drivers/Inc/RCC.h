/*
 * RCC.h
 *
 *  Created on: Jan 8, 2025
 *      Author: Yunus
 */

#ifndef INC_RCC_H_
#define INC_RCC_H_

#include "stm32f407xx.h"

/*
 * GPIO Clock Macros
 *
 */
#define RCC_GPIOA_CLK_ENABLE()				do{ uint32_t temp = 0;\
											SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN);\
											temp = READ_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN);\
											UNUSED(temp);\
											}while(0)

#define RCC_GPIOB_CLK_ENABLE()				do{ uint32_t temp = 0;\
											SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOBEN);\
											temp = READ_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOBEN);\
											UNUSED(temp);\
											}while(0)

#define RCC_GPIOC_CLK_ENABLE()				do{ uint32_t temp = 0;\
											SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOCEN);\
											temp = READ_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOCEN);\
											UNUSED(temp);\
											}while(0)

#define RCC_GPIOD_CLK_ENABLE()				do{ uint32_t temp = 0;\
											SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIODEN);\
											temp = READ_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIODEN);\
											UNUSED(temp);\
											}while(0)

#define RCC_GPIOE_CLK_ENABLE()				do{ uint32_t temp = 0;\
											SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOEEN);\
											temp = READ_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOEEN);\
											UNUSED(temp);\
											}while(0)

#define RCC_GPIOF_CLK_ENABLE()				do{ uint32_t temp = 0;\
											SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOFEN);\
											temp = READ_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOFEN);\
											UNUSED(temp);\
											}while(0)


#define RCC_GPIOA_CLK_DISABLE()				CLEAR_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN)
#define RCC_GPIOB_CLK_DISABLE()				CLEAR_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOBEN)
#define RCC_GPIOC_CLK_DISABLE()				CLEAR_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOCEN)
#define RCC_GPIOD_CLK_DISABLE()				CLEAR_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIODEN)
#define RCC_GPIOE_CLK_DISABLE()				CLEAR_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOEEN)
#define RCC_GPIOF_CLK_DISABLE()				CLEAR_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOFEN)

/*
 * SYSCFG Clock Macros
 *
 */

#define RCC_SYSCFG_CLK_ENABLE()				do{uint32_t temp = 0;\
											SET_BIT(RCC->APB2ENR,RCC_APB2ENR_SYSCFGEN);\
											temp = READ_BIT(RCC->APB2ENR, RCC_APB2ENR_SYSCFGEN);\
											UNUSED(temp);\
											}while(0)

#define RCC_SYSCFG_CLK_DISABLE()			CLEAR_BIT(RCC->APB2ENR, RCC_APB2ENR_SYSCFGEN)


/*
 * SPI Clock Macros
 *
 */
#define RCC_SPI1_CLK_ENABLE()				do{uint32_t temp = 0;\
											SET_BIT(RCC->APB2ENR,RCC_APB2ENR_SPI1EN);\
											temp = READ_BIT(RCC->APB2ENR,RCC_APB2ENR_SPI1EN);\
											UNUSED(temp);\
											}while(0)

#define RCC_SPI2_CLK_ENABLE()				do{uint32_t temp = 0;\
											SET_BIT(RCC->APB1ENR,RCC_APB1ENR_SPI2EN);\
											temp = READ_BIT(RCC->APB1ENR,RCC_APB1ENR_SPI2EN);\
											UNUSED(temp);\
											}while(0)

#define RCC_SPI3_CLK_ENABLE()				do{uint32_t temp = 0;\
											SET_BIT(RCC->APB1ENR,RCC_APB1ENR_SPI3EN);\
											temp = READ_BIT(RCC->APB1ENR,RCC_APB1ENR_SPI3EN);\
											UNUSED(temp);\
											}while(0)

#define RCC_SPI4_CLK_ENABLE()				do{uint32_t temp = 0;\
											SET_BIT(RCC->APB2ENR,RCC_APB2ENR_SPI4EN);\
											temp = READ_BIT(RCC->APB2ENR,RCC_APB2ENR_SPI4EN);\
											UNUSED(temp);\
											}while(0)

#define RCC_SPI5_CLK_ENABLE()				do{uint32_t temp = 0;\
											SET_BIT(RCC->APB2ENR,RCC_APB2ENR_SPI5EN);\
											temp = READ_BIT(RCC->APB2ENR,RCC_APB2ENR_SPI5EN);\
											UNUSED(temp);\
											}while(0)

#define RCC_SPI6_CLK_ENABLE()				do{uint32_t temp = 0;\
											SET_BIT(RCC->APB2ENR,RCC_APB2ENR_SPI6EN);\
											temp = READ_BIT(RCC->APB2ENR,RCC_APB2ENR_SPI6EN);\
											UNUSED(temp);\
											}while(0)

#define RCC_SPI1_CLK_DISABLE()				CLEAR_BIT(RCC->APB2ENR,RCC_APB2ENR_SPI1EN)
#define RCC_SPI2_CLK_DISABLE()				CLEAR_BIT(RCC->APB1ENR,RCC_APB1ENR_SPI2EN)
#define RCC_SPI3_CLK_DISABLE()				CLEAR_BIT(RCC->APB1ENR,RCC_APB1ENR_SPI3EN)
#define RCC_SPI4_CLK_DISABLE()				CLEAR_BIT(RCC->APB2ENR,RCC_APB2ENR_SPI4EN)
#define RCC_SPI5_CLK_DISABLE()				CLEAR_BIT(RCC->APB2ENR,RCC_APB2ENR_SPI5EN)
#define RCC_SPI6_CLK_DISABLE()				CLEAR_BIT(RCC->APB2ENR,RCC_APB2ENR_SPI6EN)

/*
 * OTGFS Clock Macros
 */

#define RCC_OTGFS_CLK_ENABLE()				do{uint32_t temp = 0;\
											SET_BIT(RCC->AHB2ENR,RCC_AHB2ENR_OTGFSEN);\
											temp = READ_BIT(RCC->AHB2ENR,RCC_APB2ENR_SPI1EN);\
											UNUSED(temp);\
											}while(0)

#define RCC_OTGFS_CLK_DISABLE()				CLEAR_BIT(RCC->AHB2ENR,RCC_AHB2ENR_OTGFSEN)



#define RCC_APB1ENR_PWREN					((uint32_t)(0x10000000))


#define RCC_CR_HSEON						((uint32_t)(0x00010000))
#define RCC_CR_HSERDY						((uint32_t)(0x00020000))

#define RCC_CR_PLLON						((uint32_t)(0x01000000))
#define RCC_CR_PLLRDY						((uint32_t)(0x02000000))

#define RCC_PLLCFGR_PLLM_POS				(0U)
#define RCC_PLLCFGR_PLLN_POS				(6U)
#define RCC_PLLCFGR_PLLP_POS				(16U)
#define RCC_PLLCFGR_PLLQ_POS				(24U)

#define RCC_PLLCFGR_PLLSRC_HSI				((uint32_t)(0x00000000))
#define RCC_PLLCFGR_PLLSRC_HSE				((uint32_t)(0x00400000))

#define RCC_CFGR_SW_HSI						((uint32_t)(0x00000000))		//HSI oscillator selected as system clock
#define RCC_CFGR_SW_HSE						((uint32_t)(0x00000001))		//HSE oscillator selected as system clock
#define RCC_CFGR_SW_PLL						((uint32_t)(0x00000002))		//PLL selected as system clock
#define RCC_CFGR_SW_POS						((uint32_t)(0x00000003))

#define RCC_CFGR_SWS_HSI					((uint32_t)(0x00000000))		//HSI oscillator selected as system clock
#define RCC_CFGR_SWS_HSE					((uint32_t)(0x00000004))		//HSE oscillator selected as system clock
#define RCC_CFGR_SWS_PLL					((uint32_t)(0x00000008))		//PLL selected as system clock
#define RCC_CFGR_SWS_POS					((uint32_t)(0x0000000c))



#define RCC_CFGR_HPRE_DIV1					((uint32_t)(0x00000000))
#define RCC_CFGR_HPRE_DIV2					((uint32_t)(0x00000080))
#define RCC_CFGR_HPRE_DIV4					((uint32_t)(0x00000090))
#define RCC_CFGR_HPRE_DIV8					((uint32_t)(0x000000a0))
#define RCC_CFGR_HPRE_DIV16					((uint32_t)(0x000000b0))
#define RCC_CFGR_HPRE_DIV64					((uint32_t)(0x000000c0))
#define RCC_CFGR_HPRE_DIV128				((uint32_t)(0x000000d0))
#define RCC_CFGR_HPRE_DIV256				((uint32_t)(0x000000e0))
#define RCC_CFGR_HPRE_DIV512				((uint32_t)(0x000000f0))


#define RCC_CFGR_PPRE1_DIV1					((uint32_t)(0x00000000)) 		//0xx
#define RCC_CFGR_PPRE1_DIV2					((uint32_t)(0x00001000))		//100
#define RCC_CFGR_PPRE1_DIV4					((uint32_t)(0x00001400))		//101
#define RCC_CFGR_PPRE1_DIV8					((uint32_t)(0x00001800))		//110
#define RCC_CFGR_PPRE1_DIV16				((uint32_t)(0x00001c00))		//111


#define RCC_CFGR_PPRE2_DIV1					((uint32_t)(0x00000000)) 		//0xx
#define RCC_CFGR_PPRE2_DIV2					((uint32_t)(0x00008000))		//100
#define RCC_CFGR_PPRE2_DIV4					((uint32_t)(0x0000a000))		//101
#define RCC_CFGR_PPRE2_DIV8					((uint32_t)(0x0000c000))		//110
#define RCC_CFGR_PPRE2_DIV16				((uint32_t)(0x0000e000))		//111


/*
 * HSE
 */
void RCC_SET_CLOCK();

#endif /* INC_RCC_H_ */
