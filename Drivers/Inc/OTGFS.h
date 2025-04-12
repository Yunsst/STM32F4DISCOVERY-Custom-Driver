/*
 * OTGFS.h
 *
 *  Created on: Apr 7, 2025
 *      Author: Yunus
 */

#ifndef INC_OTGFS_H_
#define INC_OTGFS_H_

#include "stm32f407xx.h"

/********************  Bit definition for USB_OTG_GUSBCFG register  ********************/

#define USB_OTG_GUSBCFG_TOCAL_Pos                (0U)
#define USB_OTG_GUSBCFG_TOCAL_Msk                (0x7UL << USB_OTG_GUSBCFG_TOCAL_Pos) /*!< 0x00000007 */
#define USB_OTG_GUSBCFG_TOCAL                    USB_OTG_GUSBCFG_TOCAL_Msk     /*!< FS timeout calibration */
#define USB_OTG_GUSBCFG_TOCAL_0                  (0x1UL << USB_OTG_GUSBCFG_TOCAL_Pos) /*!< 0x00000001 */
#define USB_OTG_GUSBCFG_TOCAL_1                  (0x2UL << USB_OTG_GUSBCFG_TOCAL_Pos) /*!< 0x00000002 */
#define USB_OTG_GUSBCFG_TOCAL_2                  (0x4UL << USB_OTG_GUSBCFG_TOCAL_Pos) /*!< 0x00000004 */
#define USB_OTG_GUSBCFG_PHYSEL_Pos               (6U)
#define USB_OTG_GUSBCFG_PHYSEL_Msk               (0x1UL << USB_OTG_GUSBCFG_PHYSEL_Pos) /*!< 0x00000040 */
#define USB_OTG_GUSBCFG_PHYSEL                   USB_OTG_GUSBCFG_PHYSEL_Msk    /*!< USB 2.0 high-speed ULPI PHY or USB 1.1 full-speed serial transceiver select */
#define USB_OTG_GUSBCFG_SRPCAP_Pos               (8U)
#define USB_OTG_GUSBCFG_SRPCAP_Msk               (0x1UL << USB_OTG_GUSBCFG_SRPCAP_Pos) /*!< 0x00000100 */
#define USB_OTG_GUSBCFG_SRPCAP                   USB_OTG_GUSBCFG_SRPCAP_Msk    /*!< SRP-capable */
#define USB_OTG_GUSBCFG_HNPCAP_Pos               (9U)
#define USB_OTG_GUSBCFG_HNPCAP_Msk               (0x1UL << USB_OTG_GUSBCFG_HNPCAP_Pos) /*!< 0x00000200 */
#define USB_OTG_GUSBCFG_HNPCAP                   USB_OTG_GUSBCFG_HNPCAP_Msk    /*!< HNP-capable */
#define USB_OTG_GUSBCFG_TRDT_Pos                 (10U)
#define USB_OTG_GUSBCFG_TRDT_Msk                 (0xFUL << USB_OTG_GUSBCFG_TRDT_Pos) /*!< 0x00003C00 */
#define USB_OTG_GUSBCFG_TRDT                     USB_OTG_GUSBCFG_TRDT_Msk      /*!< USB turnaround time */
#define USB_OTG_GUSBCFG_TRDT_0                   (0x1UL << USB_OTG_GUSBCFG_TRDT_Pos) /*!< 0x00000400 */
#define USB_OTG_GUSBCFG_TRDT_1                   (0x2UL << USB_OTG_GUSBCFG_TRDT_Pos) /*!< 0x00000800 */
#define USB_OTG_GUSBCFG_TRDT_2                   (0x4UL << USB_OTG_GUSBCFG_TRDT_Pos) /*!< 0x00001000 */
#define USB_OTG_GUSBCFG_TRDT_3                   (0x8UL << USB_OTG_GUSBCFG_TRDT_Pos) /*!< 0x00002000 */
#define USB_OTG_GUSBCFG_PHYLPCS_Pos              (15U)
#define USB_OTG_GUSBCFG_PHYLPCS_Msk              (0x1UL << USB_OTG_GUSBCFG_PHYLPCS_Pos) /*!< 0x00008000 */
#define USB_OTG_GUSBCFG_PHYLPCS                  USB_OTG_GUSBCFG_PHYLPCS_Msk   /*!< PHY Low-power clock select */
#define USB_OTG_GUSBCFG_ULPIFSLS_Pos             (17U)
#define USB_OTG_GUSBCFG_ULPIFSLS_Msk             (0x1UL << USB_OTG_GUSBCFG_ULPIFSLS_Pos) /*!< 0x00020000 */
#define USB_OTG_GUSBCFG_ULPIFSLS                 USB_OTG_GUSBCFG_ULPIFSLS_Msk  /*!< ULPI FS/LS select               */
#define USB_OTG_GUSBCFG_ULPIAR_Pos               (18U)
#define USB_OTG_GUSBCFG_ULPIAR_Msk               (0x1UL << USB_OTG_GUSBCFG_ULPIAR_Pos) /*!< 0x00040000 */
#define USB_OTG_GUSBCFG_ULPIAR                   USB_OTG_GUSBCFG_ULPIAR_Msk    /*!< ULPI Auto-resume                */
#define USB_OTG_GUSBCFG_ULPICSM_Pos              (19U)
#define USB_OTG_GUSBCFG_ULPICSM_Msk              (0x1UL << USB_OTG_GUSBCFG_ULPICSM_Pos) /*!< 0x00080000 */
#define USB_OTG_GUSBCFG_ULPICSM                  USB_OTG_GUSBCFG_ULPICSM_Msk   /*!< ULPI Clock SuspendM             */
#define USB_OTG_GUSBCFG_ULPIEVBUSD_Pos           (20U)
#define USB_OTG_GUSBCFG_ULPIEVBUSD_Msk           (0x1UL << USB_OTG_GUSBCFG_ULPIEVBUSD_Pos) /*!< 0x00100000 */
#define USB_OTG_GUSBCFG_ULPIEVBUSD               USB_OTG_GUSBCFG_ULPIEVBUSD_Msk /*!< ULPI External VBUS Drive        */
#define USB_OTG_GUSBCFG_ULPIEVBUSI_Pos           (21U)
#define USB_OTG_GUSBCFG_ULPIEVBUSI_Msk           (0x1UL << USB_OTG_GUSBCFG_ULPIEVBUSI_Pos) /*!< 0x00200000 */
#define USB_OTG_GUSBCFG_ULPIEVBUSI               USB_OTG_GUSBCFG_ULPIEVBUSI_Msk /*!< ULPI external VBUS indicator    */
#define USB_OTG_GUSBCFG_TSDPS_Pos                (22U)
#define USB_OTG_GUSBCFG_TSDPS_Msk                (0x1UL << USB_OTG_GUSBCFG_TSDPS_Pos) /*!< 0x00400000 */
#define USB_OTG_GUSBCFG_TSDPS                    USB_OTG_GUSBCFG_TSDPS_Msk     /*!< TermSel DLine pulsing selection */
#define USB_OTG_GUSBCFG_PCCI_Pos                 (23U)
#define USB_OTG_GUSBCFG_PCCI_Msk                 (0x1UL << USB_OTG_GUSBCFG_PCCI_Pos) /*!< 0x00800000 */
#define USB_OTG_GUSBCFG_PCCI                     USB_OTG_GUSBCFG_PCCI_Msk      /*!< Indicator complement            */
#define USB_OTG_GUSBCFG_PTCI_Pos                 (24U)
#define USB_OTG_GUSBCFG_PTCI_Msk                 (0x1UL << USB_OTG_GUSBCFG_PTCI_Pos) /*!< 0x01000000 */
#define USB_OTG_GUSBCFG_PTCI                     USB_OTG_GUSBCFG_PTCI_Msk      /*!< Indicator pass through          */
#define USB_OTG_GUSBCFG_ULPIIPD_Pos              (25U)
#define USB_OTG_GUSBCFG_ULPIIPD_Msk              (0x1UL << USB_OTG_GUSBCFG_ULPIIPD_Pos) /*!< 0x02000000 */
#define USB_OTG_GUSBCFG_ULPIIPD                  USB_OTG_GUSBCFG_ULPIIPD_Msk   /*!< ULPI interface protect disable  */
#define USB_OTG_GUSBCFG_FHMOD_Pos                (29U)
#define USB_OTG_GUSBCFG_FHMOD_Msk                (0x1UL << USB_OTG_GUSBCFG_FHMOD_Pos) /*!< 0x20000000 */
#define USB_OTG_GUSBCFG_FHMOD                    USB_OTG_GUSBCFG_FHMOD_Msk     /*!< Forced host mode                */
#define USB_OTG_GUSBCFG_FDMOD_Pos                (30U)
#define USB_OTG_GUSBCFG_FDMOD_Msk                (0x1UL << USB_OTG_GUSBCFG_FDMOD_Pos) /*!< 0x40000000 */
#define USB_OTG_GUSBCFG_FDMOD                    USB_OTG_GUSBCFG_FDMOD_Msk     /*!< Forced peripheral mode          */
#define USB_OTG_GUSBCFG_CTXPKT_Pos               (31U)
#define USB_OTG_GUSBCFG_CTXPKT_Msk               (0x1UL << USB_OTG_GUSBCFG_CTXPKT_Pos) /*!< 0x80000000 */
#define USB_OTG_GUSBCFG_CTXPKT                   USB_OTG_GUSBCFG_CTXPKT_Msk    /*!< Corrupt Tx packet               */


#define OTGFS_MODE_HOST				((uint32_t)(0x20000000))
#define OTGFS_MODE_DEVICE			((uint32_t)(0x40000000))

/*
 * Bit Definitions
 */
#define OTGFS_GAHBCFG_GINT			((uint32_t)(0x00000001))
#define OTGFS_GAHBCFG_TXFELVL		((uint32_t)(0x00000040))
#define OTGFS_GAHBCFG_PTXFELVL		((uint32_t)(0x00000080))

//#define OTGFS_GUSBCFG_TOCAL			((uint32_t)(0x00000001))

#define OTGFS_GUSBCFG_SRPCAP		((uint32_t)(0x00000100))
#define OTGFS_GUSBCFG_HNPCAP		((uint32_t)(0x00000200))
//#define OTGFS_GUSBCFG_TRDT			((uint32_t)(0x00000001))


#define OTGFS_GCCFG_PWRDWN			((uint32_t)(0x00010000))   // Power Down


typedef struct
{
	uint32_t Mode;
	uint8_t Sof_enable;

}OTGFS_CfgTypeDef_t;

void OTGFS_Init(USB_OTG_GlobalTypeDef *OTGFSx,OTGFS_CfgTypeDef_t *OTGFS_ConfigStruct);

#endif /* INC_OTGFS_H_ */
