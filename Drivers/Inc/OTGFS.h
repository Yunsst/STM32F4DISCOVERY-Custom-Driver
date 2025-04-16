/*
 * OTGFS.h
 *
 *  Created on: Apr 7, 2025
 *      Author: Yunus
 */

#ifndef INC_OTGFS_H_
#define INC_OTGFS_H_

#include "stm32f407xx.h"

/********************  Bit definition for USB_OTG_GRSTCTL register  ********************/
#define USB_OTG_GRSTCTL_AHBIDL_Pos               (31U)
#define USB_OTG_GRSTCTL_AHBIDL_Msk               (0x1UL << USB_OTG_GRSTCTL_AHBIDL_Pos) /*!< 0x80000000 */
#define USB_OTG_GRSTCTL_AHBIDL                   USB_OTG_GRSTCTL_AHBIDL_Msk    /*!< AHB master idle */

#define USB_OTG_GRSTCTL_CSRST_Pos                (0U)
#define USB_OTG_GRSTCTL_CSRST_Msk                (0x1UL << USB_OTG_GRSTCTL_CSRST_Pos) /*!< 0x00000001 */
#define USB_OTG_GRSTCTL_CSRST                    USB_OTG_GRSTCTL_CSRST_Msk     /*!< Core soft reset          */
/********************  Bit definition for USB_OTG_GUSBCFG register  ********************/

#define USB_OTG_GUSBCFG_PHYSEL_Pos               (6U)
#define USB_OTG_GUSBCFG_PHYSEL_Msk               (0x1UL << USB_OTG_GUSBCFG_PHYSEL_Pos) /*!< 0x00000040 */
#define USB_OTG_GUSBCFG_PHYSEL                   USB_OTG_GUSBCFG_PHYSEL_Msk    /*!< USB 2.0 high-speed ULPI PHY or USB 1.1 full-speed serial transceiver select */

/********************  Bit definition for USB_OTG_GINTMSK register  ********************/
#define USB_OTG_GINTMSK_SOFM_Pos				 (3U)
#define USB_OTG_GINTMSK_SOFM_Msk				 (0x1UL << USB_OTG_GINTMSK_SOFM_Pos)
#define USB_OTG_GINTMSK_SOFM					 USB_OTG_GINTMSK_SOFM_Msk					// SOF

#define USB_OTG_GINTMSK_ESUSPM_Pos				 (10U)
#define USB_OTG_GINTMSK_ESUSPM_Msk				 (0x1UL << USB_OTG_GINTMSK_ESUSPM_Pos)
#define USB_OTG_GINTMSK_ESUSPM					 USB_OTG_GINTMSK_ESUSPM_Msk					// Early suspend

#define USB_OTG_GINTMSK_USBSUSPM_Pos			 (11U)
#define USB_OTG_GINTMSK_USBSUSPM_Msk			 (0x1UL << USB_OTG_GINTMSK_USBSUSPM_Pos)
#define USB_OTG_GINTMSK_USBSUSPM				 USB_OTG_GINTMSK_USBSUSPM_Msk				// USB suspend

#define USB_OTG_GINTMSK_USBRST_Pos				 (12U)
#define USB_OTG_GINTMSK_USBRST_Msk				 (0x1UL << USB_OTG_GINTMSK_USBRST_Pos)
#define USB_OTG_GINTMSK_USBRST					 USB_OTG_GINTMSK_USBRST_Msk					// USB reset

#define USB_OTG_GINTMSK_ENUMDNEM_Pos			 (13U)
#define USB_OTG_GINTMSK_ENUMDNEM_Msk			 (0x1UL << USB_OTG_GINTMSK_ENUMDNEM_Pos)
#define USB_OTG_GINTMSK_ENUMDNEM				 USB_OTG_GINTMSK_ENUMDNEM_Msk				// Enumeration done

/********************  Bit definition for USB_OTG_DSTS register  ********************/
#define USB_OTG_DSTS_ENUMSPD_Pos				 (1U)
#define USB_OTG_DSTS_ENUMSPD_Msk				 (0x3UL << USB_OTG_DSTS_ENUMSPD_Pos)
#define USB_OTG_DSTS_ENUMSPD					 USB_OTG_DSTS_ENUMSPD_Msk				// Enumeration done


#define OTGFS_MODE_HOST				((uint32_t)(0x20000000))
#define OTGFS_MODE_DEVICE			((uint32_t)(0x40000000))

/*
 * Bit Definitions
 */
#define OTGFS_GAHBCFG_GINT			((uint32_t)(0x00000001))
#define OTGFS_GAHBCFG_TXFELVL		((uint32_t)(0x00000080))
#define OTGFS_GAHBCFG_PTXFELVL		((uint32_t)(0x00000100))


#define OTGFS_GUSBCFG_TOCAL_7		((uint32_t)(0x00000007)) // TOCAL=7 1.75 bit
#define OTGFS_GUSBCFG_SRPCAP		((uint32_t)(0x00000100))
#define OTGFS_GUSBCFG_HNPCAP		((uint32_t)(0x00000200))
#define OTGFS_GUSBCFG_TRDT_9		((uint32_t)(0x00002400)) // 168 MHz için 0x9 değeri

#define OTGFS_GINTMSK_OTGINT		((uint32_t)(0x00000004))
#define OTGFS_GINTMSK_MMISM			((uint32_t)(0x00000002))
#define OTGFS_GINTMSK_REFLVLM		((uint32_t)(0x00000010))

#define OTGFS_GCCFG_PWRDWN			((uint32_t)(0x00010000))   // Power Down
#define OTGFS_GCCFG_VBUSASEN		((uint32_t)(0x00040000))
#define OTGFS_GCCFG_VBUSBSEN		((uint32_t)(0x00080000))

#define OTGFS_GINTSTS_CMOD			((uint32_t)(0x00000001))

#define OTGFS_DCFG_DSPD_FULL		((uint32_t)(0x00000003))  // FULL SPEED
#define OTGFS_DCFG_NZLSOHSK			((uint32_t)(0x00000004))  // NZLSOHSK == 1;



typedef struct
{
	uint32_t Mode;
	uint8_t Sof_enable;

}OTGFS_CfgTypeDef_t;

void OTGFS_Init(OTGFS_CfgTypeDef_t *OTGFS_ConfigStruct);
static void OTGFS_CoreReset();
void OTGFS_DeviceInit();

#endif /* INC_OTGFS_H_ */
