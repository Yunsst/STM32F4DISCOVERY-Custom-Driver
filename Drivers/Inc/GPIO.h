/*
 * GPIO.h
 *
 *  Created on: Jan 8, 2025
 *      Author: Yunus
 */

#ifndef INC_GPIO_H_
#define INC_GPIO_H_

#include "stm32f407xx.h"

#define GPIO_PIN_0				(uint16_t)(0x0001)
#define GPIO_PIN_1				(uint16_t)(0x0002)
#define GPIO_PIN_2				(uint16_t)(0x0004)
#define GPIO_PIN_3				(uint16_t)(0x0008)
#define GPIO_PIN_4				(uint16_t)(0x0010)
#define GPIO_PIN_5				(uint16_t)(0x0020)
#define GPIO_PIN_6				(uint16_t)(0x0040)
#define GPIO_PIN_7				(uint16_t)(0x0080)
#define GPIO_PIN_8				(uint16_t)(0x0100)
#define GPIO_PIN_9				(uint16_t)(0x0200)
#define GPIO_PIN_10				(uint16_t)(0x0400)
#define GPIO_PIN_11				(uint16_t)(0x0800)
#define GPIO_PIN_12				(uint16_t)(0x1000)
#define GPIO_PIN_13				(uint16_t)(0x2000)
#define GPIO_PIN_14				(uint16_t)(0x4000)
#define GPIO_PIN_15				(uint16_t)(0x8000)
#define GPIO_PIN_ALL			(uint16_t)(0xFFFF)

#define GPIO_MODE_INPUT			(0x0U)
#define GPIO_MODE_OUTPUT		(0x1U)
#define GPIO_MODE_AF			(0x2U)
#define GPIO_MODE_ANALOG		(0x3U)


#define GPIO_OTYPE_PP			(0x0U)
#define GPIO_OTYPE_OD			(0x1U)

#define GPIO_PUPD_NOPULL		(0x0U)
#define GPIO_PUPD_PULLUP		(0x1U)
#define GPIO_PUPD_PULLDOWN		(0x2U)

#define GPIO_SPEED_LOW			(0x0U)
#define GPIO_SPEED_MEDIUM		(0x1U)
#define GPIO_SPEED_HIGH			(0x2U)
#define GPIO_SPEED_VERY			(0x3U)


#define GPIO_AF0				(0x0U)
#define GPIO_AF1				(0x1U)
#define GPIO_AF2				(0x2U)
#define GPIO_AF3				(0x3U)
#define GPIO_AF4				(0x4U)
#define GPIO_AF5				(0x5U)
#define GPIO_AF6				(0x6U)
#define GPIO_AF7				(0x7U)
#define GPIO_AF8				(0x8U)
#define GPIO_AF9				(0x9U)
#define GPIO_AF10				(0xAU)
#define GPIO_AF11				(0xBU)
#define GPIO_AF12				(0xCU)
#define GPIO_AF13				(0xDU)
#define GPIO_AF14				(0xEU)
#define GPIO_AF15				(0xFU)



typedef enum
{
	GPIO_PIN_RESET = 0x0U,
	GPIO_PIN_SET

}GPIO_PinState_t;

typedef struct
{
	uint32_t pinNumber;
	uint32_t Mode;
	uint32_t Otype;
	uint32_t Pupd;
	uint32_t Speed;
	uint32_t Altarnate;

}GPIO_InitTypeDef_t;


void GPIO_Init(GPIO_TypeDef_t *GPIOx,GPIO_InitTypeDef_t *GPIO_ConfigStruct);
void GPIO_WritePin(GPIO_TypeDef_t *GPIOx, uint16_t pinNumber, GPIO_PinState_t pinState);
GPIO_PinState_t GPIO_ReadPin(GPIO_TypeDef_t *GPIOx, uint16_t pinNumber);
void GPIO_LockPin(GPIO_TypeDef_t *GPIOx, uint16_t pinNumber);
void GPIO_TogglePin(GPIO_TypeDef_t *GPIOx, uint16_t pinNumber);



#endif /* INC_GPIO_H_ */
