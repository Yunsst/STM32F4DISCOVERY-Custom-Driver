/*
 * OTGFS.c
 *
 *  Created on: Apr 7, 2025
 *      Author: Yunus
 */


void OTGFS_Init(USB_OTG_GlobalTypeDef *OTGFS_Core,OTGFS_InitTypeDef_t *OTGFS_ConfigStruct){

	OTGFS_Core->GAHBCFG |= 0x1U ; // GINTMSK == 1
	OTGFS_Core->GAHBCFG |= 0x3U << 7U; // TXFELVL & PTXFELVL == 1

	OTGFS_Core->GUSBCFG |= 0x3U << 8U; // SRPCAP & HNPCAP == 1
	OTGFS_Core->GUSBCFG |= 0x5U; // TOCAL = 5 (48 MHz için)


}

#include "OTGFS.h"
