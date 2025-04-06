/*
 * SPI.h
 *
 *  Created on: Jan 16, 2025
 *      Author: Yunus
 */

#ifndef INC_SPI_H_
#define INC_SPI_H_

#include "stm32f407xx.h"

typedef enum
{
	SPI_BUS_FREE = 0x0U,
	SPI_BUS_BUSY_TX = 0x1U,
	SPI_BUS_BUSY_RX = 0x2U,
}SPI_BusStatus_t;

#define SPI_BAUDRATE_DIV2		((uint32_t)(0x00000000))
#define SPI_BAUDRATE_DIV4		((uint32_t)(0x00000008))
#define SPI_BAUDRATE_DIV8		((uint32_t)(0x00000010))
#define SPI_BAUDRATE_DIV16		((uint32_t)(0x00000018))
#define SPI_BAUDRATE_DIV32		((uint32_t)(0x00000020))
#define SPI_BAUDRATE_DIV64		((uint32_t)(0x00000028))
#define SPI_BAUDRATE_DIV128		((uint32_t)(0x00000030))
#define SPI_BAUDRATE_DIV256		((uint32_t)(0x00000038))


#define SPI_CPHA_FIRST			((uint32_t)(0x00000000))
#define SPI_CPHA_SECOND			((uint32_t)(0x00000001))


#define SPI_CPOL_LOW			((uint32_t)(0x00000000))
#define SPI_CPOL_HIGH			((uint32_t)(0x00000002))


#define SPI_DFF_8BITS			((uint32_t)(0x00000000))
#define SPI_DFF_16BITS			((uint32_t)(0x00000800))


#define SPI_MODE_MASTER			((uint32_t)(0x00000004))
#define SPI_MODE_SLAVE			((uint32_t)(0x00000000))


#define SPI_FRAMEFORMAT_MSB		((uint32_t)(0x00000000))
#define SPI_FRAMEFORMAT_LSB		((uint32_t)(0x00000080))


#define SPI_BUS_FullDuplex		((uint32_t)(0x00000000))
#define SPI_BUS_ReceiveOnly		((uint32_t)(0x00000400))
#define SPI_BUS_HalfDuplex_R	((uint32_t)(0x00000800))
#define SPI_BUS_HalfDuplex_T	((uint32_t)(0x00000C00))


#define SPI_SSM_CMD_DISABLED	((uint32_t)(0x00000000))
#define SPI_SSM_CMD_ENABLED		((uint32_t)(0x00000300))


typedef enum
{
	SPI_FLAG_RESET = 0x0U,
	SPI_FLAG_SET = !SPI_FLAG_RESET

}SPI_FlagStatus_t;

typedef struct
{
	uint32_t Mode;
	uint32_t CPHA;
	uint32_t CPOL;
	uint32_t BaudRate;
	uint32_t SSM_CMD;
	uint32_t DFF_Format;
	uint32_t BusConfig;
	uint32_t FrameFormat;

}SPI_InitTypeDef_t;

typedef struct __SPI_HandleTypeDef_t
{
	SPI_TypeDef_t *Instance;
	SPI_InitTypeDef_t Init;
	uint8_t *pTxDataAddr;
	uint16_t TxDataSize;
	uint8_t busStateTx;
	void(*TxISRFunction)(struct __SPI_HandleTypeDef_t *SPI_Handle);


}SPI_HandleTypeDef_t;

/*
 * SPI Function Declarations
 *
 */
void SPI_Init(SPI_HandleTypeDef_t *SPI_Handle);
void SPI_PeripCmd(SPI_HandleTypeDef_t *SPI_Handle, FunctionalState_t stateOfSPI);
void SPI_TransmitData(SPI_HandleTypeDef_t *SPI_Handle, uint8_t *pData, uint16_t sizeOfData);
void SPI_ReceiveData(SPI_HandleTypeDef_t *SPI_Handle, uint8_t *pBuffer, uint16_t sizeOfData);
void SPI_TransmitData_IT(SPI_HandleTypeDef_t *SPI_Handle, uint8_t *pData, uint16_t sizeOfData);
void SPI_InterruptHandler(SPI_HandleTypeDef_t *SPI_Handle);

SPI_FlagStatus_t SPI_GetFlagStatus(SPI_HandleTypeDef_t *SPI_Handle, uint16_t SPI_Flag);

#endif /* INC_SPI_H_ */
