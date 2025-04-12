/*
 * OTGFS.c
 *
 *  Created on: Apr 7, 2025
 *      Author: Yunus
 */
#include "OTGFS.h"

void OTGFS_Init(USB_OTG_GlobalTypeDef *OTGFS_Core,OTGFS_CfgTypeDef_t *OTGFS_ConfigStruct){


	OTGFS_Core->GCCFG |= OTGFS_GCCFG_PWRDWN; // USB PHY enable

	uint32_t temp = 0;

	temp = OTGFS_Core->GAHBCFG;
	temp |= (OTGFS_GAHBCFG_GINT) | (OTGFS_GAHBCFG_TXFELVL) | (OTGFS_GAHBCFG_PTXFELVL);

	OTGFS_Core->GAHBCFG = temp;

	temp = OTGFS_Core->GUSBCFG;

	temp |= (OTGFS_GUSBCFG_SRPCAP) | OTGFS_GUSBCFG_HNPCAP | (OTGFS_ConfigStruct->Mode);

	OTGFS_Core->GUSBCFG |= 0x3U << 8U; // SRPCAP & HNPCAP == 1
	OTGFS_Core->GUSBCFG |= 0x5U; // TOCAL = 5 (48 MHz için)


}


