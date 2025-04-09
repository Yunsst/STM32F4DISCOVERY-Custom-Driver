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

#endif /* INC_RCC_H_ */
