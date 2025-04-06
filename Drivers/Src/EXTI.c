/*
 * EXTI.c
 *
 *  Created on: Jan 15, 2025
 *      Author: Yunus
 */

#include "EXTI.h"

void EXTI_LineConfig(uint8_t PortSource, uint8_t EXTI_LineSource)
{
	uint32_t temp;

	temp = SYSCFG->CR[EXTI_LineSource >> 2U];
	temp &= ~(0xFU << (EXTI_LineSource & 0x3U) *4);
	temp = (PortSource << (EXTI_LineSource & 0x3U) * 4 );
	SYSCFG->CR[EXTI_LineSource >> 2U] = temp;
}

void EXTI_Init(EXTI_InitTypeDef_t *EXTI_InitStruct)
{
	uint32_t temp=0;

	temp = (uint32_t)EXTI_BASE_ADDR;

	EXTI->IMR &= ~(0x1U <<EXTI_InitStruct->EXTI_LineNumber);
	EXTI->EMR &= ~(0x1U <<EXTI_InitStruct->EXTI_LineNumber);

	if(EXTI_InitStruct->EXTI_LineCmd != DISABLE)
	{
		temp += EXTI_InitStruct->EXTI_Mode;

		*((__IO uint32_t*)temp) |= (0x1U << EXTI_InitStruct->EXTI_LineNumber);

		temp = (uint32_t)EXTI_BASE_ADDR;

		EXTI->RTSR &= ~(0x1U << EXTI_InitStruct->EXTI_LineNumber);
		EXTI->FTSR &= ~(0x1U << EXTI_InitStruct->EXTI_LineNumber);

		if(EXTI_InitStruct->TriggerSelection == EXTI_Trigger_RF)
		{
			EXTI->RTSR |= (0x1U << EXTI_InitStruct->EXTI_LineNumber);
			EXTI->FTSR |= (0x1U << EXTI_InitStruct->EXTI_LineNumber);

		}else
		{
			temp += EXTI_InitStruct->TriggerSelection;
			*((__IO uint32_t*)temp) |= (0x1U << EXTI_InitStruct->EXTI_LineNumber);

		}
	}
	else
	{
		temp = (uint32_t)EXTI_BASE_ADDR;

		temp += EXTI_InitStruct->EXTI_Mode;

		*((__IO uint32_t*)temp) &= (0x1U << EXTI_InitStruct->EXTI_LineNumber);

	}
}


void NVIC_EnableInterrupt(IRQNumber_TypeDef_t IRQNumber)
{
	uint32_t temp = 0;

	temp = *((IRQNumber >> 5U) + NVIC_ISER0);
	temp &= ~(0x1U << (IRQNumber &0x1FU));
	temp |= (0x1U << (IRQNumber &0x1FU));
	*((IRQNumber >> 5U) + NVIC_ISER0) = temp;
}

