/*
 * OTGFS.c
 *
 *  Created on: Apr 7, 2025
 *      Author: Yunus
 */
#include "OTGFS.h"

void OTGFS_Init(OTGFS_CfgTypeDef_t *OTGFS_ConfigStruct){

	uint32_t temp = 0;

	UNUSED(OTGFS_CoreReset());

	temp = USB_OTG_CORE->GUSBCFG;
	temp |= OTGFS_ConfigStruct->Mode;
	USB_OTG_CORE->GUSBCFG = temp;

	temp |= OTGFS_GCCFG_PWRDWN; // USB PHY enable
	USB_OTG_CORE->GCCFG= temp;

	temp = USB_OTG_CORE->GAHBCFG;
	temp |= (OTGFS_GAHBCFG_GINT) | (OTGFS_GAHBCFG_TXFELVL) | (OTGFS_GAHBCFG_PTXFELVL) ;
	USB_OTG_CORE->GAHBCFG = temp;

	temp = USB_OTG_CORE->GUSBCFG;
	temp &= ~(0xFU << 10);
	temp |= (OTGFS_GUSBCFG_SRPCAP) | OTGFS_GUSBCFG_HNPCAP |\
			(OTGFS_GUSBCFG_TOCAL_7) | (OTGFS_GUSBCFG_TRDT_9);
	USB_OTG_CORE->GUSBCFG = temp;

	temp = USB_OTG_CORE->GINTMSK;
	temp |= (OTGFS_GINTMSK_OTGINT) | (OTGFS_GINTMSK_MMISM) | (OTGFS_GINTMSK_REFLVLM);
	USB_OTG_CORE->GINTMSK = temp;

	temp = USB_OTG_CORE->GCCFG;
	temp |= (OTGFS_GCCFG_VBUSASEN) | (OTGFS_GCCFG_VBUSBSEN);
	USB_OTG_CORE->GCCFG = temp;

	temp = USB_OTG_CORE->GINTSTS;

	if (temp & OTGFS_GINTSTS_CMOD){
		// Host
	}else{
		// Device
		OTGFS_DeviceInit();
	}
}


void OTGFS_DeviceInit(){
    uint32_t temp = 0;

    // 1. DCFG: Device speed ve NZLSOHSK ayarla
    temp = USB_OTG_DEVICE->DCFG;
    temp &= ~OTGFS_DCFG_DSPD_FULL;
    temp |= (OTGFS_DCFG_DSPD_FULL) | (OTGFS_DCFG_NZLSOHSK) ; // Full-speed
    USB_OTG_DEVICE->DCFG = temp;

    // 2. GINTMSK: Gerekli kesmeleri aç
    temp = USB_OTG_CORE->GINTMSK;
    temp |= (USB_OTG_GINTMSK_USBRST) |(USB_OTG_GINTMSK_ENUMDNEM) | (USB_OTG_GINTMSK_ESUSPM) |\
    		(USB_OTG_GINTMSK_USBSUSPM) | (USB_OTG_GINTMSK_SOFM);

    USB_OTG_CORE->GINTMSK = temp;

    // 3. GCCFG: VBUSBSEN bitini set et (B-device VBUS sensing)
    temp = USB_OTG_CORE->GCCFG;
    temp |= OTGFS_GCCFG_VBUSBSEN;
    USB_OTG_CORE->GCCFG = temp;

    // 4. USBRST interrupt bekle (reset algılanana kadar)
    while(!(USB_OTG_CORE->GINTSTS & USB_OTG_GINTMSK_USBRST));

    // 5. ENUMDNE interrupt bekle (enumeration tamamlanana kadar)
    while(!(USB_OTG_CORE->GINTSTS & USB_OTG_GINTMSK_ENUMDNEM));

    // 6. DSTS register'ından hız bilgisini oku
    uint32_t speed = (USB_OTG_DEVICE->DSTS & USB_OTG_DSTS_ENUMSPD_Msk) >> USB_OTG_DSTS_ENUMSPD_Pos;
    UNUSED(speed);
    // 7. Endpoint initialization (burada örnek verilmedi)
    // Control endpoint 0 için gerekli ayarları yapmalısınız
}


/**
  * @brief  Reset the USB Core (needed after USB clock settings change)
  */
static void OTGFS_CoreReset()
{
  __IO uint32_t count = 0U;

  /* Wait for AHB master IDLE state. */
  do
  {
    count++;
    if (count > 200000U)
    {
      return; // AHB IDLE beklerken timeout
    }
  } while ((USB_OTG_CORE->GRSTCTL & USB_OTG_GRSTCTL_AHBIDL) == 0U);

  /* Core Soft Reset */
  USB_OTG_CORE->GRSTCTL |= USB_OTG_GRSTCTL_CSRST;

  /* Reset işleminin tamamlanmasını bekle */
  count = 0U;
  do
  {
    count++;
    if (count > 200000U)
    {
      return; // Reset tamamlanırken timeout
    }
  } while ((USB_OTG_CORE->GRSTCTL & USB_OTG_GRSTCTL_CSRST) != 0U);

  /* Reset sonrası kısa bir bekleme */
  for(volatile uint32_t i = 0; i < 10000; i++);
}
