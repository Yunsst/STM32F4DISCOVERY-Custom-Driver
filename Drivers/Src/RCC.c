/*
 * RCC.c
 *
 *  Created on: Jan 8, 2025
 *      Author: Yunus
 */


#include "RCC.h"


void RCC_SET_CLOCK()
{
	uint32_t temp = 0;

	temp = RCC->CR;
	temp |= RCC_CR_HSEON;
	RCC->CR = temp;

	while (!(RCC->CR & RCC_CR_HSERDY));

	temp = RCC->APB1ENR;
	temp |= RCC_APB1ENR_PWREN;
	RCC->APB1ENR = temp;

	temp = PWR->CR;
	temp |= (0x1 << 14U);											// VOS scale 1
	PWR->CR = temp;

	temp |= FLASH->ACR;
	temp |= FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN;
	temp &= ~FLASH_ACR_LATENCY_7WS; 								//Latency clear
	temp |= FLASH_ACR_LATENCY_5WS;
	FLASH->ACR |= temp;

	temp = RCC->CFGR;
	temp |= (RCC_CFGR_HPRE_DIV1) | (RCC_CFGR_PPRE1_DIV4) | (RCC_CFGR_PPRE2_DIV2);
	RCC->CFGR = temp;


	temp = RCC->PLLCFGR;
	temp &= ~(0x3F << RCC_PLLCFGR_PLLM_POS);									//PLLM Clear
	temp &= ~(0x1FF << RCC_PLLCFGR_PLLN_POS);									//PLLN Clear
	temp &= ~(0x3 << RCC_PLLCFGR_PLLP_POS);										//PLLP Clear
	temp &= ~(0xF << RCC_PLLCFGR_PLLQ_POS);										//PLLQ Clear
	temp |= (8 << RCC_PLLCFGR_PLLM_POS) | (336 << RCC_PLLCFGR_PLLN_POS) |\
			(0 << RCC_PLLCFGR_PLLP_POS) | (RCC_PLLCFGR_PLLSRC_HSE)		|\
			(7 << RCC_PLLCFGR_PLLQ_POS);
	RCC->PLLCFGR = temp;

	RCC->CR |= RCC_CR_PLLON; // PLL'i aç
	while (!(RCC->CR & RCC_CR_PLLRDY)); // PLL hazır olana kadar bekle

	RCC->CFGR &= ~RCC_CFGR_SW_POS;
	temp = RCC->CFGR;
	temp |= RCC_CFGR_SW_PLL;
	RCC->CFGR = temp;

	while ((RCC->CFGR & RCC_CFGR_SWS_POS) != RCC_CFGR_SWS_PLL);
}
