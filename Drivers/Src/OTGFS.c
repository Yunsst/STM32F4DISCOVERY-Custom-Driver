/*
 * OTGFS.c
 *
 *  Created on: Apr 7, 2025
 *      Author: Yunus
 */
#include "OTGFS.h"

void OTGFS_Init(OTGFS_CfgTypeDef_t *OTGFS_ConfigStruct){

	uint32_t temp = 0;

	temp |= OTGFS_GCCFG_PWRDWN; // USB PHY enable

	USB_OTG_CORE->GCCFG= temp;

	temp = USB_OTG_CORE->GAHBCFG;
	temp |= (OTGFS_GAHBCFG_GINT) | (OTGFS_GAHBCFG_TXFELVL) | (OTGFS_GAHBCFG_PTXFELVL) ;

	USB_OTG_CORE->GAHBCFG = temp;


	temp = USB_OTG_CORE->GUSBCFG;
	temp &= ~(0xFU << 10);
	temp |= (OTGFS_GUSBCFG_SRPCAP) | OTGFS_GUSBCFG_HNPCAP | (OTGFS_ConfigStruct->Mode)|\
			(OTGFS_GUSBCFG_TOCAL_7) | (OTGFS_GUSBCFG_TRDT_9);

	USB_OTG_CORE->GUSBCFG = temp;


	temp = USB_OTG_CORE->GINTMSK;
	temp |= (OTGFS_GINTMSK_OTGINT) | (OTGFS_GINTMSK_MMISM) | (OTGFS_GINTMSK_REFLVLM);

	USB_OTG_CORE->GINTMSK = temp;

	temp = USB_OTG_CORE->GCCFG;
	temp |= (OTGFS_GCCFG_VBUSASEN) | (OTGFS_GCCFG_VBUSBSEN);

	USB_OTG_CORE->GCCFG = temp;

	USB_OTG_CORE->GINTSTS &= ~(0xFFFFFFFF);
	temp = USB_OTG_CORE->GINTSTS;

	if (temp & OTGFS_GINTSTS_CMOD){
		// Host
	}else{
		// Device
	}
}


