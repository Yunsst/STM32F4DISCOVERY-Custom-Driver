/*
 * OTGFS.h
 *
 *  Created on: Apr 7, 2025
 *      Author: Yunus
 */

#ifndef INC_OTGFS_H_
#define INC_OTGFS_H_

#include "stm32f407xx.h"

#define OTGFS_MODE_DEVICE		(0x0U)
#define OTGFS_MODE_HOST			(0x1U)

typedef struct
{
	uint32_t Mode;

}OTGFS_InitTypeDef_t;

void OTGFS_Init(USB_OTG_GlobalTypeDef *OTGFSx,OTGFS_InitTypeDef_t *OTGFS_ConfigStruct);

#endif /* INC_OTGFS_H_ */
