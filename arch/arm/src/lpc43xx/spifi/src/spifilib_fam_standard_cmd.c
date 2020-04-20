/*
 * @brief Common Command Set Family driver
 *
 * @note
 * Copyright(C) NXP Semiconductors, 2014
 * All rights reserved.
 *
 * @par
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * LPC products.  This software is supplied "AS IS" without any warranties of
 * any kind, and NXP Semiconductors and its licenser disclaim any and
 * all warranties, express or implied, including all implied warranties of
 * merchantability, fitness for a particular purpose and non-infringement of
 * intellectual property rights.  NXP Semiconductors assumes no responsibility
 * or liability for the use of the software, conveys no license or rights under any
 * patent, copyright, mask work right, or any other intellectual property rights in
 * or to any products. NXP Semiconductors reserves the right to make changes
 * in the software without notification. NXP Semiconductors also makes no
 * representation or warranty that such application will be suitable for the
 * specified use without further testing or modification.
 *
 * @par
 * Permission to use, copy, modify, and distribute this software and its
 * documentation is hereby granted, under NXP Semiconductors' and its
 * licensor's relevant copyrights in the software, without fee, provided that it
 * is used in conjunction with NXP Semiconductors microcontrollers.  This
 * copyright, permission, and disclaimer notice must appear in all copies of
 * this code.
 */
#include <string.h>

#include "spifi/inc/spifilib_dev.h"
#include "spifi/inc/private/spifilib_chiphw.h"
#include <nuttx/board.h>
#include <arch/board/board.h>

/* need access to the device register Fx without importing the whole API */
extern SPIFI_ERR_T spifiDevRegister(SPIFI_FAM_NODE_T *pFamily, SPIFI_DEV_NODE_T *pDevData);

/** @defgroup LPCSPIFILIB_CONFIG_STANDARD_CMD_SET LPCSPIFILIB Common Command set driver
 * @ingroup LPCSPIFILIB_DRIVERS
 * This driver includes support for the following devices. (Clones within {} brackets).<br>
 * S25FL016K {W25Q16DV}<br>
 * S25FL032P<br>
 * S25FL064P<br>
 * S25FL129P 64k Sector<br>
 * S25FL129P 256k Sector {S25FL128S}<br>
 * S25FL164K<br>
 * S25FL256S<br>
 * S25FL512S<br>
 * MX25L1635E<br>
 * MX25L3235E<br>
 * MX25L6435E<br>
 * MX25L8035E<br>
 * W25Q32FV<br>
 * W25Q64FV<br>
 * W25Q80BV {W25Q80DV}<br>
 *
 * Driver Feature Specifics:<br>
 * - common command set<br>
 *
 * Optimization: Ram / code size optimizations can be enabled by enabling specific
 * devices in lieu of the default (all devices enabled). Change the setting of
 * SPIFI_DEVICE_ALL to 0 and enable specific devices by changing the setting
 * of the appropriate SPIFI_DEVICE_XXX to 1.
 * @{
 */

/**
 * @}
 */

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/
/**
 * @brief The following set of defines allows fine tuning where specific device support
 * can be pre determined and space is at a premium.  To Reduce code / iRam requirements
 * change the SPIFI_DEVICE_ALL to 0 and change the desired device definition(s) to 1
 */
//#define SPIFI_DEVICE_ALL                0		/**< Enables all devices in family */
//#define SPIFI_DEVICE_S25FL016K          0		/**< Enables Spansion S25FL016K device */
//#define SPIFI_DEVICE_S25FL032P          0		/**< Enables Spansion S25FL032P device */
//#define SPIFI_DEVICE_S25FL064P          0		/**< Enables Spansion S25FL064P device */
//#define SPIFI_DEVICE_S25FL129P_64K      0		/**< Enables Spansion S25FL129P (64K block) device */
//#define SPIFI_DEVICE_S25FL129P_256K     0		/**< Enables Spansion S25FL129P (256K block) device */
//#define SPIFI_DEVICE_S25FL164K          0		/**< Enables Spansion S25FL164K device */
//#define SPIFI_DEVICE_S25FL256S_64K      0		/**< Enables Spansion S25FL256S (64K block) device */
//#define SPIFI_DEVICE_S25FL256S_256K     0		/**< Enables Spansion S25FL256S (256K block) device */
//#define SPIFI_DEVICE_S25FL512S          0		/**< Enables Spansion S25FL512S device */
//#define SPIFI_DEVICE_MX25L1635E         0		/**< Enables Macronix MX25L1635E device */
//#define SPIFI_DEVICE_MX25L3235E         0		/**< Enables Macronix MX25L3235E device */
//#define SPIFI_DEVICE_MX25L8035E         0		/**< Enables Macronix MX25L8035E device */
//#define SPIFI_DEVICE_MX25L6435E         0		/**< Enables Macronix MX25L6435E device */
//#define SPIFI_DEVICE_W25Q32FV           0		/**< Enables Winbond W25Q32FV device */
//#define SPIFI_DEVICE_W25Q64FV           0		/**< Enables Winbond W25Q32V device */
//#define SPIFI_DEVICE_W25Q80BV           0		/**< Enables Winbond W25Q80BV device */

/* Required private data size for this family */
#define PRVDATASIZE 0

#define MAX_SINGLE_READ             16128

/* Command definitions. Only used commands are defined. */
#define CMD_0B_FAST_READ            0x0B		/**< Read Data bytes at Fast Speed */
#define CMD_BB_DIOR                 0xBB		/**< Dual IORead (all dual except op code( */
#define CMD_EB_QIOR                 0xEB		/**< Quad IORead (all quad except op code) */
#define CMD_0C_FAST_READ            0x0C		/**< Read Data (4B addr) bytes at Fast Speed */
#define CMD_BC_DIOR                 0xBC		/**< Dual IORead (4B addr) (all dual except op code( */
#define CMD_EC_QIOR                 0xEC		/**< Quad IORead (4B addr) (all quad except op code) */
#define CMD_06_WREN                 0x06		/**< Write Enable */
#define CMD_20_P4E                  0x20		/**< 4 KB Parameter Sector Erase */
#define CMD_C7_BE                   0xC7		/**< Bulk Erase */
#define CMD_D8_SE                   0xD8		/**< Sector Erase */
#define CMD_DC_SE					0xDC		/**< Sector erase (4B addr) */
#define CMD_02_PP                   0x02		/**< Page Programming */
#define CMD_12_PP                   0x12		/**< Page Programming (4B addr) */
#define CMD_05_RDSR1                0x05		/**< Read Status Register 1 */
#define CMD_35_RDSR2                0x35		/**< Read Status Register 2 */
#define CMD_33_RDSR3                0x33		/**< Read Status Register 3 */
#define CMD_01_WSR                  0x01		/**< Write Status Registers */
#define CMD_30_CSR                  0x30		/**< Reset the Erase and Program Fail Flag (SR5 and SR6) and restore normal operation) */
#define CMD_32_QPP                  0x32		/**< Quad Page Programming */
#define CMD_34_QPP                  0x34		/**< Quad Page Programming (4B addr) */
#define CMD_38_QPP_MACRONIX         0x38		/**< Quad Page Programming for 25L3235E */

/* Common status register definitions */
/* Status Register Write Disable,
   1 = Protects when W# is low,
   0 = No protection, even when W# is low */
#define STATUS_SRWD                   (1 << 7)

/* Block protect bits,
   Protects upper half of address range in 5 sizes */
#define STATUS_BPMASK                 (7 << 2)
/* Write Enable Latch,
   1 = Device accepts Write Status Register, program, or erase commands,
   0 = Ignores Write Status Register, program, or erase commands */
#define STATUS_WEL                    (1 << 1)
/* Write in Progress,
   1 = Device Busy. A Write Status Register, program, or erase,
   0 = Ready. Device is in standby mode and can accept commands. */
#define STATUS_WIP                    (1 << 0)

/* Virtual status bits
   (i.e moved to byte 4 so they don't conflict with bits in lower 3 bytes */
/* Programming Error Occurred,
   0 = No Error,
   1 = Error occurred */
#define STATUS_P_ERR                  (1 << 24)
/* Erase Error Occurred,
   0 = No Error,
   1 = Error occurred */
#define STATUS_W_ERR                  (1 << 25)

/* Functions used macro definitions */
/* Macros to control conditional use functions */
#define NEED_spifiDeviceDataGetStatusS25FL032P (SPIFI_DEVICE_ALL | \
												SPIFI_DEVICE_S25FL016K | \
												SPIFI_DEVICE_S25FL032P | \
												SPIFI_DEVICE_S25FL064P | \
												SPIFI_DEVICE_S25FL129P_64K | \
												SPIFI_DEVICE_S25FL129P_256K | \
												SPIFI_DEVICE_S25FL256S_64K | \
												SPIFI_DEVICE_S25FL256S_256K | \
												SPIFI_DEVICE_S25FL512S)

#define NEED_spifiDeviceDataGetStatusS25FL164K (SPIFI_DEVICE_ALL | \
												SPIFI_DEVICE_S25FL164K)

#define NEED_spifiDeviceDataGetStatusMX25L3235E (SPIFI_DEVICE_ALL |	\
												 SPIFI_DEVICE_MX25L1635E | \
												 SPIFI_DEVICE_MX25L3235E | \
												 SPIFI_DEVICE_MX25L8035E | \
												 SPIFI_DEVICE_MX25L6435E)

#define NEED_spifiDeviceDataGetStatusW25Q80BV (SPIFI_DEVICE_ALL | \
											   SPIFI_DEVICE_W25Q80BV | \
											   SPIFI_DEVICE_W25Q32FV | \
											   SPIFI_DEVICE_W25Q64FV)

#define NEED_spifiDeviceDataClearStatusNone (SPIFI_DEVICE_ALL |	\
											 SPIFI_DEVICE_W25Q80BV | \
											 SPIFI_DEVICE_W25Q32FV | \
											 SPIFI_DEVICE_W25Q64FV | \
											 SPIFI_DEVICE_MX25L1635E | \
											 SPIFI_DEVICE_MX25L3235E | \
											 SPIFI_DEVICE_MX25L8035E | \
											 SPIFI_DEVICE_MX25L6435E)

#define NEED_spifiDeviceDataClearStatusS25FL032P (SPIFI_DEVICE_ALL | \
												  SPIFI_DEVICE_S25FL016K | \
												  SPIFI_DEVICE_S25FL032P | \
												  SPIFI_DEVICE_S25FL064P | \
												  SPIFI_DEVICE_S25FL129P_64K | \
												  SPIFI_DEVICE_S25FL129P_256K |	\
												  SPIFI_DEVICE_S25FL164K | \
												  SPIFI_DEVICE_S25FL256S_64K | \
												  SPIFI_DEVICE_S25FL256S_256K |	\
												  SPIFI_DEVICE_S25FL512S)

#define NEED_spifiDeviceDataSetStatusS25FL032P (SPIFI_DEVICE_ALL |	\
												SPIFI_DEVICE_S25FL016K | \
												SPIFI_DEVICE_S25FL032P | \
												SPIFI_DEVICE_S25FL064P | \
												SPIFI_DEVICE_S25FL129P_64K | \
												SPIFI_DEVICE_S25FL129P_256K | \
												SPIFI_DEVICE_S25FL256S_64K | \
												SPIFI_DEVICE_S25FL256S_256K | \
												SPIFI_DEVICE_S25FL512S | \
												SPIFI_DEVICE_W25Q80BV |	\
												SPIFI_DEVICE_W25Q32FV |	\
												SPIFI_DEVICE_W25Q64FV)

#define NEED_spifiDeviceDataSetStatusS25FL164K (SPIFI_DEVICE_ALL |	\
												SPIFI_DEVICE_S25FL164K)

#define NEED_spifiDeviceDataSetStatusMX25L3235E (SPIFI_DEVICE_ALL |	\
												 SPIFI_DEVICE_MX25L1635E | \
												 SPIFI_DEVICE_MX25L3235E | \
												 SPIFI_DEVICE_MX25L8035E | \
												 SPIFI_DEVICE_MX25L6435E)

#define NEED_spifiDeviceDataSetOptsQuadModeBit6 (SPIFI_DEVICE_ALL |	\
												 SPIFI_DEVICE_MX25L1635E | \
												 SPIFI_DEVICE_MX25L3235E | \
												 SPIFI_DEVICE_MX25L8035E | \
												 SPIFI_DEVICE_MX25L6435E)

#define NEED_spifiDeviceDataSetOptsQuadModeBit9 (SPIFI_DEVICE_ALL |	\
												 SPIFI_DEVICE_S25FL016K | \
												 SPIFI_DEVICE_S25FL032P | \
												 SPIFI_DEVICE_S25FL064P | \
												 SPIFI_DEVICE_S25FL129P_64K | \
												 SPIFI_DEVICE_S25FL129P_256K | \
												 SPIFI_DEVICE_S25FL164K | \
												 SPIFI_DEVICE_S25FL256S_64K | \
												 SPIFI_DEVICE_S25FL256S_256K | \
												 SPIFI_DEVICE_S25FL512S | \
												 SPIFI_DEVICE_W25Q80BV | \
												 SPIFI_DEVICE_W25Q32FV | \
												 SPIFI_DEVICE_W25Q64FV)

#define NEED_spifiDeviceDataInitDeinit (SPIFI_DEVICE_ALL |	\
										SPIFI_DEVICE_S25FL016K | \
										SPIFI_DEVICE_S25FL032P | \
										SPIFI_DEVICE_S25FL064P | \
										SPIFI_DEVICE_S25FL129P_64K | \
										SPIFI_DEVICE_S25FL129P_256K | \
										SPIFI_DEVICE_S25FL256S_64K | \
										SPIFI_DEVICE_S25FL256S_256K | \
										SPIFI_DEVICE_S25FL512S | \
										SPIFI_DEVICE_W25Q80BV |	\
										SPIFI_DEVICE_W25Q32FV |	\
										SPIFI_DEVICE_W25Q64FV |	\
										SPIFI_DEVICE_MX25L1635E | \
										SPIFI_DEVICE_MX25L3235E | \
										SPIFI_DEVICE_MX25L8035E | \
										SPIFI_DEVICE_MX25L6435E)

#define NEED_spifiDeviceDataInitDeinitS25FL164K (SPIFI_DEVICE_ALL |	\
												 SPIFI_DEVICE_S25FL164K)

#define NEED_spifiDevice4BInitReadCommand (SPIFI_DEVICE_ALL |	\
												SPIFI_DEVICE_S25FL256S_64K | \
												SPIFI_DEVICE_S25FL256S_256K | \
												SPIFI_DEVICE_S25FL512S)

#define NEED_spifiDeviceInitReadCommand (SPIFI_DEVICE_ALL |	\
										 SPIFI_DEVICE_S25FL016K | \
										 SPIFI_DEVICE_S25FL032P | \
										 SPIFI_DEVICE_S25FL064P | \
										 SPIFI_DEVICE_S25FL129P_64K | \
										 SPIFI_DEVICE_S25FL129P_256K | \
										 SPIFI_DEVICE_S25FL164K | \
										 SPIFI_DEVICE_W25Q80BV | \
										 SPIFI_DEVICE_W25Q32FV | \
										 SPIFI_DEVICE_W25Q64FV | \
										 SPIFI_DEVICE_MX25L1635E | \
										 SPIFI_DEVICE_MX25L3235E | \
										 SPIFI_DEVICE_MX25L8035E | \
										 SPIFI_DEVICE_MX25L6435E)

#define NEED_spifiDeviceInitWriteCommand (SPIFI_DEVICE_ALL |	\
										  SPIFI_DEVICE_S25FL016K | \
										  SPIFI_DEVICE_S25FL032P | \
										  SPIFI_DEVICE_S25FL064P | \
										  SPIFI_DEVICE_S25FL129P_64K | \
										  SPIFI_DEVICE_S25FL129P_256K |	\
										  SPIFI_DEVICE_S25FL164K | \
										  SPIFI_DEVICE_W25Q80BV | \
										  SPIFI_DEVICE_W25Q32FV | \
										  SPIFI_DEVICE_W25Q64FV | \
										  SPIFI_DEVICE_MX25L8035E)

#define NEED_spifiDevice4BInitWriteCommand (SPIFI_DEVICE_ALL |	\
											SPIFI_DEVICE_S25FL256S_64K | \
											SPIFI_DEVICE_S25FL256S_256K |	\
											SPIFI_DEVICE_S25FL512S)

#define NEED_spifiDeviceInitWriteCommandMacronix (SPIFI_DEVICE_ALL |	\
												  SPIFI_DEVICE_MX25L1635E |	\
												  SPIFI_DEVICE_MX25L3235E |	\
												  SPIFI_DEVICE_MX25L6435E)

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/* Software write Enable */
static void spifiPrvSetWREN(LPC_SPIFI_CHIPHW_T *pSpifiCtrlAddr)
{
	spifi_HW_SetCmd(pSpifiCtrlAddr,
					(SPIFI_CMD_OPCODE(CMD_06_WREN) |
					 SPIFI_CMD_FIELDFORM(SPIFI_FIELDFORM_ALL_SERIAL) |
					 SPIFI_CMD_FRAMEFORM(SPIFI_FRAMEFORM_OP)));

	spifi_HW_WaitCMD(pSpifiCtrlAddr);
}

/* Wait for device to complete operation (I.e not busy) */
static void spifiPrvWaitUnBusy(const SPIFI_HANDLE_T *pHandle)
{
	/* Device wait for device to be ready */
	while ((pHandle->pFamFx->devGetStatus(pHandle) & STATUS_WIP) != 0) {}
}

static void spifiPrvSetQuadModeBitPosition(const SPIFI_HANDLE_T *pHandle, uint32_t bitPosition, uint8_t enQuadMode)
{
	uint32_t statusRegs;

	/* Read the device specific status bytes */
	statusRegs = pHandle->pFamFx->devGetStatus(pHandle);

	/* Set / clear bit x */
	if (enQuadMode) {
		statusRegs |= (1 << bitPosition);
	}
	else {
		statusRegs &= ~(1 << bitPosition);
	}

	/* Write status back out */
	pHandle->pFamFx->devSetStatus(pHandle, statusRegs);
}

/* Checks to see if the device is writable and not busy */
static SPIFI_ERR_T spifiPrvCheckWriteState(const SPIFI_HANDLE_T *pHandle)
{
	uint32_t stat;

	/* Get status */
	stat = pHandle->pFamFx->devGetStatus(pHandle);

	/* Exit if blocks are locked or WIP in progress */
	if ((stat & STATUS_BPMASK) != 0) {
		return SPIFI_ERR_LOCKED;
	}
	else if ((stat & STATUS_WIP) != 0) {
		return SPIFI_ERR_BUSY;
	}

	return SPIFI_ERR_NONE;
}

/*****************************************************************************
 * Semi-Private functions
 * Functions may be assigned to SPIFI_DEVICE_DATA_T function pointers.
 ****************************************************************************/

/* Read the status bytes for the following device(s)... */
#if NEED_spifiDeviceDataGetStatusS25FL032P
static uint32_t spifiDeviceDataGetStatusS25FL032P(const SPIFI_HANDLE_T *pHandle)
{
	static const uint8_t spifiCmdOp[2] = {CMD_05_RDSR1, CMD_35_RDSR2};
	uint32_t statusReg = 0;
	LPC_SPIFI_CHIPHW_T *pSpifiCtrlAddr = (LPC_SPIFI_CHIPHW_T *) pHandle->pInfoData->spifiCtrlAddr;

	uint32_t index;

	/* Read the status bytes needed */
	for (index = 0; index < (sizeof(spifiCmdOp) / sizeof(spifiCmdOp[0])); ++index) {

		spifi_HW_SetCmd(pSpifiCtrlAddr,
						(SPIFI_CMD_OPCODE(spifiCmdOp[index]) |
						 SPIFI_CMD_DATALEN(1) |
						 SPIFI_CMD_FIELDFORM(SPIFI_FIELDFORM_ALL_SERIAL) |
						 SPIFI_CMD_FRAMEFORM(SPIFI_FRAMEFORM_OP)));

		statusReg |= (spifi_HW_GetData8(pSpifiCtrlAddr) << (8 * index));

		/* Wait for command to complete */
		spifi_HW_WaitCMD(pSpifiCtrlAddr);
	}

	/* Move the error bits to generic location */
	if (statusReg & (1 << 5)) {
		statusReg |= STATUS_W_ERR;
	}
	if (statusReg & (1 << 6)) {
		statusReg |= STATUS_P_ERR;
	}

	/* Finally remove the error bits from the original location */
	statusReg &= ~(3 << 5);

	return statusReg;
}

#endif

/* Read the status bytes for the following device(s)... */
#if NEED_spifiDeviceDataGetStatusS25FL164K
static uint32_t spifiDeviceDataGetStatusS25FL164K(const SPIFI_HANDLE_T *pHandle)
{
	static const uint8_t spifiCmdOp[3] = {CMD_05_RDSR1, CMD_35_RDSR2, CMD_33_RDSR3};
	uint32_t statusRegs = 0;
	LPC_SPIFI_CHIPHW_T *pSpifiCtrlAddr = (LPC_SPIFI_CHIPHW_T *) pHandle->pInfoData->spifiCtrlAddr;
	uint32_t idx;

	for (idx = 0; idx < sizeof(spifiCmdOp) / sizeof(spifiCmdOp[0]); ++idx) {
		spifi_HW_SetCmd(pSpifiCtrlAddr,
						(SPIFI_CMD_OPCODE(spifiCmdOp[idx]) |
						 SPIFI_CMD_DATALEN(1) |
						 SPIFI_CMD_FIELDFORM(SPIFI_FIELDFORM_ALL_SERIAL) |
						 SPIFI_CMD_FRAMEFORM(SPIFI_FRAMEFORM_OP)));

		statusRegs |= (spifi_HW_GetData8(pSpifiCtrlAddr) << (8 * idx));

		/* Wait for command to complete */
		spifi_HW_WaitCMD(pSpifiCtrlAddr);
	}

	return statusRegs;
}

#endif

/* Read the status bytes for the following device(s)... */
#if NEED_spifiDeviceDataGetStatusMX25L3235E
static uint32_t spifiDeviceDataGetStatusMX25L3235E(const SPIFI_HANDLE_T *pHandle)
{
	uint32_t statRegister;
	LPC_SPIFI_CHIPHW_T *pSpifiCtrlAddr = (LPC_SPIFI_CHIPHW_T *) pHandle->pInfoData->spifiCtrlAddr;

	spifi_HW_SetCmd(pSpifiCtrlAddr,
					(SPIFI_CMD_OPCODE(CMD_05_RDSR1) |
					 SPIFI_CMD_DATALEN(1) |
					 SPIFI_CMD_FIELDFORM(SPIFI_FIELDFORM_ALL_SERIAL) |
					 SPIFI_CMD_FRAMEFORM(SPIFI_FRAMEFORM_OP)));

	statRegister = spifi_HW_GetData8(pSpifiCtrlAddr);

	/* Wait for command to complete */
	spifi_HW_WaitCMD(pSpifiCtrlAddr);

	return statRegister;
}

#endif

/* Read the status bytes for the following device(s)... */
#if NEED_spifiDeviceDataGetStatusW25Q80BV
static uint32_t spifiDeviceDataGetStatusW25Q80BV(const SPIFI_HANDLE_T *pHandle)
{
	static const uint8_t spifiCmdOp[2] = {CMD_05_RDSR1, CMD_35_RDSR2};
	uint32_t statusReg = 0;
	LPC_SPIFI_CHIPHW_T *pSpifiCtrlAddr = (LPC_SPIFI_CHIPHW_T *) pHandle->pInfoData->spifiCtrlAddr;

	uint32_t index;

	/* Read the status bytes needed */
	for (index = 0; index < (sizeof(spifiCmdOp) / sizeof(spifiCmdOp[0])); ++index) {

		spifi_HW_SetCmd(pSpifiCtrlAddr,
						(SPIFI_CMD_OPCODE(spifiCmdOp[index]) |
						 SPIFI_CMD_DATALEN(1) |
						 SPIFI_CMD_FIELDFORM(SPIFI_FIELDFORM_ALL_SERIAL) |
						 SPIFI_CMD_FRAMEFORM(SPIFI_FRAMEFORM_OP)));

		statusReg |= (spifi_HW_GetData8(pSpifiCtrlAddr) << (8 * index));

		/* Wait for command to complete */
		spifi_HW_WaitCMD(pSpifiCtrlAddr);
	}

	return statusReg;
}

#endif

/* Software clear status for the following device(s) */
#if NEED_spifiDeviceDataClearStatusNone
static void spifiDeviceDataClearStatusNone(const SPIFI_HANDLE_T *pHandle)
{
	/* Do nothing */
}

#endif

/* Software clear status for the following device(s) */
#if NEED_spifiDeviceDataClearStatusS25FL032P
static void spifiDeviceDataClearStatusS25FL032P(const SPIFI_HANDLE_T *pHandle)
{
	LPC_SPIFI_CHIPHW_T *pSpifiCtrlAddr = (LPC_SPIFI_CHIPHW_T *) pHandle->pInfoData->spifiCtrlAddr;

	spifi_HW_SetCmd(pSpifiCtrlAddr,
					(SPIFI_CMD_OPCODE(CMD_30_CSR) |
					 SPIFI_CMD_FIELDFORM(SPIFI_FIELDFORM_ALL_SERIAL) |
					 SPIFI_CMD_FRAMEFORM(SPIFI_FRAMEFORM_OP)));

	spifi_HW_WaitCMD(pSpifiCtrlAddr);
}

#endif

/* Write Status / Config Register for the following device(s) */
#if NEED_spifiDeviceDataSetStatusS25FL032P
static void spifiDeviceDataSetStatusS25FL032P(const SPIFI_HANDLE_T *pHandle, uint32_t status)
{
	LPC_SPIFI_CHIPHW_T *pSpifiCtrlAddr = (LPC_SPIFI_CHIPHW_T *) pHandle->pInfoData->spifiCtrlAddr;

	spifiPrvSetWREN(pSpifiCtrlAddr);
	spifi_HW_SetCmd(pSpifiCtrlAddr,
					(SPIFI_CMD_OPCODE(CMD_01_WSR) |
					 SPIFI_CMD_DATALEN(2) |
					 SPIFI_CMD_DOUT(1) |
					 SPIFI_CMD_FIELDFORM(SPIFI_FIELDFORM_ALL_SERIAL) |
					 SPIFI_CMD_FRAMEFORM(SPIFI_FRAMEFORM_OP)));

	/* Write the data out. Don't worry about restoring error bits as they are read only anyway */
	spifi_HW_SetData8(pSpifiCtrlAddr, status);
	spifi_HW_SetData8(pSpifiCtrlAddr, (status >> 8));

	/* Wait for Controller to finish command */
	spifi_HW_WaitCMD(pSpifiCtrlAddr);

	/* Wait for flash controller to finish command */
	spifiPrvWaitUnBusy(pHandle);
}

#endif

/* Write Status1, 2 and 3 Register for the following device(s) */
#if NEED_spifiDeviceDataSetStatusS25FL164K
static void spifiDeviceDataSetStatusS25FL164K(const SPIFI_HANDLE_T *pHandle, uint32_t status)
{
	LPC_SPIFI_CHIPHW_T *pSpifiCtrlAddr = (LPC_SPIFI_CHIPHW_T *) pHandle->pInfoData->spifiCtrlAddr;

	spifiPrvSetWREN(pSpifiCtrlAddr);
	spifi_HW_SetCmd(pSpifiCtrlAddr,
					(SPIFI_CMD_OPCODE(CMD_01_WSR) |
					 SPIFI_CMD_DATALEN(3) |
					 SPIFI_CMD_DOUT(1) |
					 SPIFI_CMD_FIELDFORM(SPIFI_FIELDFORM_ALL_SERIAL) |
					 SPIFI_CMD_FRAMEFORM(SPIFI_FRAMEFORM_OP)));

	/* Write the data out */
	spifi_HW_SetData8(pSpifiCtrlAddr, status);
	spifi_HW_SetData8(pSpifiCtrlAddr, (status >> 8));
	spifi_HW_SetData8(pSpifiCtrlAddr, (status >> 16));

	/* Wait for Controller to finish command */
	spifi_HW_WaitCMD(pSpifiCtrlAddr);

	/* Wait for flash controller to finish command */
	spifiPrvWaitUnBusy(pHandle);
}

#endif

/* Write Status / Config Register for the following device(s)*/
#if NEED_spifiDeviceDataSetStatusMX25L3235E
static void spifiDeviceDataSetStatusMX25L3235E(const SPIFI_HANDLE_T *pHandle, uint32_t status)
{
	LPC_SPIFI_CHIPHW_T *pSpifiCtrlAddr = (LPC_SPIFI_CHIPHW_T *) pHandle->pInfoData->spifiCtrlAddr;

	spifiPrvSetWREN(pSpifiCtrlAddr);
	spifi_HW_SetCmd(pSpifiCtrlAddr,
					(SPIFI_CMD_OPCODE(CMD_01_WSR) |
					 SPIFI_CMD_DATALEN(1) |
					 SPIFI_CMD_DOUT(1) |
					 SPIFI_CMD_FIELDFORM(SPIFI_FIELDFORM_ALL_SERIAL) |
					 SPIFI_CMD_FRAMEFORM(SPIFI_FRAMEFORM_OP)));

	/* Write the data out */
	spifi_HW_SetData8(pSpifiCtrlAddr, status);

	/* Wait for Controller to finish command */
	spifi_HW_WaitCMD(pSpifiCtrlAddr);

	/* Wait for flash controller to finish command */
	spifiPrvWaitUnBusy(pHandle);
}

#endif

/* Function to change bit 6 of status/config register for the following device(s) */
#if NEED_spifiDeviceDataSetOptsQuadModeBit6
static SPIFI_ERR_T spifiDeviceDataSetOptsQuadModeBit6(const SPIFI_HANDLE_T *pHandle, uint32_t opts, uint32_t enMode)
{
	/* Do not attempt to set bit if option is not supported */
	if (opts & (SPIFI_CAP_QUAD_READ | SPIFI_CAP_QUAD_WRITE)) {
		spifiPrvSetQuadModeBitPosition(pHandle, 6, enMode);
	}
	return SPIFI_ERR_NONE;
}

#endif

/* Function to change bit 9 of status/config register for the following device(s) */
#if NEED_spifiDeviceDataSetOptsQuadModeBit9
static SPIFI_ERR_T spifiDeviceDataSetOptsQuadModeBit9(const SPIFI_HANDLE_T *pHandle, uint32_t opts, uint32_t enMode)
{
	/* Do not attempt to set bit if option is not supported */
	if (opts & (SPIFI_CAP_QUAD_READ | SPIFI_CAP_QUAD_WRITE)) {
		spifiPrvSetQuadModeBitPosition(pHandle, 9, enMode);
	}
	return SPIFI_ERR_NONE;
}

#endif

/* Initialize SPIFI device for the following device(s) */
#if NEED_spifiDeviceDataInitDeinit
static SPIFI_ERR_T spifiDeviceDataInitDeinit(const SPIFI_HANDLE_T *pHandle, uint32_t init)
{
	return SPIFI_ERR_NONE;
}

#endif

/* Initialize SPIFI device for the following device(s) */
#if NEED_spifiDeviceDataInitDeinitS25FL164K
static SPIFI_ERR_T spifiDeviceDataInitDeinitS25FL164K(const SPIFI_HANDLE_T *pHandle, uint32_t init)
{
	uint32_t status;

	/* Disable variable read latency */
	if (init) {
		status = pHandle->pFamFx->devGetStatus(pHandle);
		status &= ~(0xf << 16);	/* Latency control bits for this part */
		pHandle->pFamFx->devSetStatus(pHandle, status);
	}

	return SPIFI_ERR_NONE;
}

#endif

/* Function to return spifi controller read cmd */
#if NEED_spifiDeviceInitReadCommand
static void spifiDeviceInitReadCommand(const SPIFI_HANDLE_T *pHandle, uint8_t enable,
								uint32_t *cmd, uint32_t *iData)
{
	if (iData) {
		*iData = 0xFF;
	}

	if (pHandle->pInfoData->opts & SPIFI_CAP_QUAD_READ) {
		*cmd =
			(SPIFI_CMD_OPCODE(CMD_EB_QIOR) |
			 SPIFI_CMD_DOUT(0) |
			 SPIFI_CMD_INTER(3) |
			 SPIFI_CMD_FIELDFORM(SPIFI_FIELDFORM_SERIAL_OPCODE) |
			 SPIFI_CMD_FRAMEFORM(SPIFI_FRAMEFORM_OP_3ADDRESS));
	}
	else if (pHandle->pInfoData->opts & SPIFI_CAP_DUAL_READ) {
		*cmd =
			(SPIFI_CMD_OPCODE(CMD_BB_DIOR) |
			 SPIFI_CMD_DOUT(0) |
			 SPIFI_CMD_INTER(1) |
			 SPIFI_CMD_FIELDFORM(SPIFI_FIELDFORM_SERIAL_OPCODE) |
			 SPIFI_CMD_FRAMEFORM(SPIFI_FRAMEFORM_OP_3ADDRESS));
	}
	/* Default to single lane mode if no other modes enabled */
	else {
		*cmd =
			(SPIFI_CMD_OPCODE(CMD_0B_FAST_READ) |
			 SPIFI_CMD_DOUT(0) |
			 SPIFI_CMD_INTER(1) |
			 SPIFI_CMD_FIELDFORM(SPIFI_FIELDFORM_ALL_SERIAL) |
			 SPIFI_CMD_FRAMEFORM(SPIFI_FRAMEFORM_OP_3ADDRESS));
	}
}

#endif

/* Function to return spifi controller read cmd */
#if NEED_spifiDevice4BInitReadCommand
static void spifiDevice4BInitReadCommand(const SPIFI_HANDLE_T *pHandle, uint8_t enable,
								uint32_t *cmd, uint32_t *iData)
{
	if (iData) {
		*iData = 0xFF;
	}

	if (pHandle->pInfoData->opts & SPIFI_CAP_QUAD_READ) {
		*cmd =
			(SPIFI_CMD_OPCODE(CMD_EC_QIOR) |
			 SPIFI_CMD_DOUT(0) |
			 SPIFI_CMD_INTER(3) |
			 SPIFI_CMD_FIELDFORM(SPIFI_FIELDFORM_SERIAL_OPCODE) |
			 SPIFI_CMD_FRAMEFORM(SPIFI_FRAMEFORM_OP_4ADDRESS));
	}
	else if (pHandle->pInfoData->opts & SPIFI_CAP_DUAL_READ) {
		*cmd =
			(SPIFI_CMD_OPCODE(CMD_BC_DIOR) |
			 SPIFI_CMD_DOUT(0) |
			 SPIFI_CMD_INTER(1) |
			 SPIFI_CMD_FIELDFORM(SPIFI_FIELDFORM_SERIAL_OPCODE) |
			 SPIFI_CMD_FRAMEFORM(SPIFI_FRAMEFORM_OP_4ADDRESS));
	}
	/* Default to single lane mode if no other modes enabled */
	else {
		*cmd =
			(SPIFI_CMD_OPCODE(CMD_0C_FAST_READ) |
			 SPIFI_CMD_DOUT(0) |
			 SPIFI_CMD_INTER(1) |
			 SPIFI_CMD_FIELDFORM(SPIFI_FIELDFORM_ALL_SERIAL) |
			 SPIFI_CMD_FRAMEFORM(SPIFI_FRAMEFORM_OP_4ADDRESS));
	}
}

#endif

/* Function to return spifi controller write cmd */
#if NEED_spifiDeviceInitWriteCommand
static void spifiDeviceInitWriteCommand(const SPIFI_HANDLE_T *pHandle, uint32_t *cmd)
{
	if (pHandle->pInfoData->opts & SPIFI_CAP_QUAD_WRITE) {
		*cmd = (SPIFI_CMD_OPCODE(CMD_32_QPP) |
				SPIFI_CMD_DOUT(1) |
				SPIFI_CMD_FIELDFORM(SPIFI_FIELDFORM_SERIAL_OPCODE_ADDRESS) |
				SPIFI_CMD_FRAMEFORM(SPIFI_FRAMEFORM_OP_3ADDRESS));
	}
	else {
		*cmd = (SPIFI_CMD_OPCODE(CMD_02_PP) |
				SPIFI_CMD_DOUT(1) |
				SPIFI_CMD_FIELDFORM(SPIFI_FIELDFORM_ALL_SERIAL) |
				SPIFI_CMD_FRAMEFORM(SPIFI_FRAMEFORM_OP_3ADDRESS));
	}
}

#endif

#if NEED_spifiDevice4BInitWriteCommand
static void spifiDevice4BInitWriteCommand(const SPIFI_HANDLE_T *pHandle, uint32_t *cmd)
{
	if (pHandle->pInfoData->opts & SPIFI_CAP_QUAD_WRITE) {
		*cmd = (SPIFI_CMD_OPCODE(CMD_34_QPP) |
				SPIFI_CMD_DOUT(1) |
				SPIFI_CMD_FIELDFORM(SPIFI_FIELDFORM_SERIAL_OPCODE_ADDRESS) |
				SPIFI_CMD_FRAMEFORM(SPIFI_FRAMEFORM_OP_4ADDRESS));
	}
	else {
		*cmd = (SPIFI_CMD_OPCODE(CMD_12_PP) |
				SPIFI_CMD_DOUT(1) |
				SPIFI_CMD_FIELDFORM(SPIFI_FIELDFORM_ALL_SERIAL) |
				SPIFI_CMD_FRAMEFORM(SPIFI_FRAMEFORM_OP_4ADDRESS));
	}
}

#endif

/* Function to return spifi controller write cmd */
#if NEED_spifiDeviceInitWriteCommandMacronix
static void spifiDeviceInitWriteCommandMacronix(const SPIFI_HANDLE_T *pHandle, uint32_t *cmd)
{
	if (pHandle->pInfoData->opts & SPIFI_CAP_QUAD_WRITE) {
		*cmd = (SPIFI_CMD_OPCODE(CMD_38_QPP_MACRONIX) |
				SPIFI_CMD_DOUT(1) |
				SPIFI_CMD_FIELDFORM(SPIFI_FIELDFORM_SERIAL_OPCODE) |
				SPIFI_CMD_FRAMEFORM(SPIFI_FRAMEFORM_OP_3ADDRESS));
	}
	else {
		*cmd = (SPIFI_CMD_OPCODE(CMD_02_PP) |
				SPIFI_CMD_DOUT(1) |
				SPIFI_CMD_FIELDFORM(SPIFI_FIELDFORM_ALL_SERIAL) |
				SPIFI_CMD_FRAMEFORM(SPIFI_FRAMEFORM_OP_3ADDRESS));
	}
}

#endif

/* Function to indicate configuration error */
static void spifiDeviceFxError(void)
{
	while (1) {}
}

/* Function to return Fx* for initialization */
static deviceInitDeInitFx spifiDeviceAssignFxInitDeInit(SPIFI_HANDLE_T *pHandle)
{
	if (pHandle->pInfoData->pDeviceData->initDeInitFxId == FX_spifiDeviceDataInitDeinitS25FL164K) {
#if NEED_spifiDeviceDataInitDeinitS25FL164K
		return spifiDeviceDataInitDeinitS25FL164K;
#endif
	}
	else if (pHandle->pInfoData->pDeviceData->initDeInitFxId == FX_spifiDeviceDataInitDeinit ) {
#if NEED_spifiDeviceDataInitDeinit
		return spifiDeviceDataInitDeinit;
#endif
	}
	return (deviceInitDeInitFx) spifiDeviceFxError;
}

/* Function to return Fx* for clearing status */
static devClearStatusFx spifiDeviceAssignFxClearStatus(SPIFI_HANDLE_T *pHandle)
{
	/* Initialize the device clearStatus Fx* */
	if (pHandle->pInfoData->pDeviceData->clearStatusFxId == FX_spifiDeviceDataClearStatusS25FL032P) {
#if NEED_spifiDeviceDataClearStatusS25FL032P
		return spifiDeviceDataClearStatusS25FL032P;
#endif
	}
	else if (pHandle->pInfoData->pDeviceData->clearStatusFxId == FX_spifiDeviceDataClearStatusNone) {
#if NEED_spifiDeviceDataClearStatusNone
		return spifiDeviceDataClearStatusNone;
#endif
	}
	return (devClearStatusFx) spifiDeviceFxError;
}

/* Function to return Fx* for getting status */
static devGetStatusFx spifiDeviceAssignFxGetStatus(SPIFI_HANDLE_T *pHandle)
{
	/* Initialize the device getStatus Fx* */
	if (pHandle->pInfoData->pDeviceData->getStatusFxId == FX_spifiDeviceDataGetStatusS25FL032P) {
#if NEED_spifiDeviceDataGetStatusS25FL032P
		return spifiDeviceDataGetStatusS25FL032P;
#endif
	}
	else if (pHandle->pInfoData->pDeviceData->getStatusFxId == FX_spifiDeviceDataGetStatusS25FL164K) {
#if NEED_spifiDeviceDataGetStatusS25FL164K
		return spifiDeviceDataGetStatusS25FL164K;
#endif
	}
	else if (pHandle->pInfoData->pDeviceData->getStatusFxId == FX_spifiDeviceDataGetStatusMX25L3235E) {
#if NEED_spifiDeviceDataGetStatusMX25L3235E
		return spifiDeviceDataGetStatusMX25L3235E;
#endif
	}
	else if (pHandle->pInfoData->pDeviceData->getStatusFxId == FX_spifiDeviceDataGetStatusW25Q80BV) {
#if NEED_spifiDeviceDataGetStatusW25Q80BV
		return spifiDeviceDataGetStatusW25Q80BV;
#endif
	}
	return (devGetStatusFx) spifiDeviceFxError;
}

/* Function for returning Fx* for setting status */
static devSetStatusFx spifiDeviceAssignFxSetStatus(SPIFI_HANDLE_T *pHandle)
{
	/* Initialize the device setStatus Fx* */
	if (pHandle->pInfoData->pDeviceData->setStatusFxId == FX_spifiDeviceDataSetStatusS25FL032P) {
#if NEED_spifiDeviceDataSetStatusS25FL032P
		return spifiDeviceDataSetStatusS25FL032P;
#endif
	}
	else if (pHandle->pInfoData->pDeviceData->setStatusFxId == FX_spifiDeviceDataSetStatusS25FL164K) {
#if NEED_spifiDeviceDataSetStatusS25FL164K
		return spifiDeviceDataSetStatusS25FL164K;
#endif
	}
	else if (pHandle->pInfoData->pDeviceData->setStatusFxId == FX_spifiDeviceDataSetStatusMX25L3235E) {
#if NEED_spifiDeviceDataSetStatusMX25L3235E
		return spifiDeviceDataSetStatusMX25L3235E;
#endif
	}
	return (devSetStatusFx) spifiDeviceFxError;
}

/* Function for returning Fx* for setting options */
static devSetOptsFx spifiDeviceAssignFxSetOptions(SPIFI_HANDLE_T *pHandle)
{
	/* Initialize the device setOptions Fx* */
	if (pHandle->pInfoData->pDeviceData->setOptionsFxId == FX_spifiDeviceDataSetOptsQuadModeBit9) {
#if NEED_spifiDeviceDataSetOptsQuadModeBit9
		return spifiDeviceDataSetOptsQuadModeBit9;
#endif
	}
	else if (pHandle->pInfoData->pDeviceData->setOptionsFxId == FX_spifiDeviceDataSetOptsQuadModeBit6) {
#if NEED_spifiDeviceDataSetOptsQuadModeBit6
		return spifiDeviceDataSetOptsQuadModeBit6;
#endif
	}
	return (devSetOptsFx) spifiDeviceFxError;
}

/* Function for returning Fx* for getting spifi controller read cmd */
static devGetReadCmdFx spifiDeviceAssignFxReadCmd(SPIFI_HANDLE_T *pHandle)
{
	/* Initialize the device getReadCmd Fx* */
	if (pHandle->pInfoData->pDeviceData->getReadCmdFxId == FX_spifiDeviceInitReadCommand) {
#if NEED_spifiDeviceInitReadCommand
		return spifiDeviceInitReadCommand;
#endif
	}
	else if (pHandle->pInfoData->pDeviceData->getReadCmdFxId == FX_spifiDevice4BInitReadCommand) {
#if NEED_spifiDevice4BInitReadCommand
		return spifiDevice4BInitReadCommand;
#endif
	}
	return (devGetReadCmdFx) spifiDeviceFxError;
}

/* Function for returning Fx* for getting spifi controller write cmd */
static devGetWriteCmdFx spifiDeviceAssignFxWriteCmd(SPIFI_HANDLE_T *pHandle)
{
	/* Initialize the device getWriteCmd Fx* */
	if (pHandle->pInfoData->pDeviceData->getWriteCmdFxId == FX_spifiDeviceInitWriteCommand) {
#if NEED_spifiDeviceInitWriteCommand
		return spifiDeviceInitWriteCommand;
#endif
	}
	else if (pHandle->pInfoData->pDeviceData->getWriteCmdFxId == FX_spifiDevice4BInitWriteCommand) {
	#if NEED_spifiDevice4BInitWriteCommand
			return spifiDevice4BInitWriteCommand;
	#endif
	}
	else if (pHandle->pInfoData->pDeviceData->getWriteCmdFxId == FX_spifiDeviceInitWriteCommandMacronix) {
#if NEED_spifiDeviceInitWriteCommandMacronix
		return spifiDeviceInitWriteCommandMacronix;
#endif
	}
	return (devGetWriteCmdFx) spifiDeviceFxError;
}

/*****************************************************************************
 * Semi-Private family functions
 * Functions may be assigned to SPIFI_FAM_FX_T function pointers.
 ****************************************************************************/
/* Converts a device status to an OR'ed API status */
static uint32_t spifiFamFxGetDeviceStatus(const SPIFI_HANDLE_T *pHandle, uint8_t clearStatus)
{
	uint32_t devStat;
	uint32_t status = 0;

	/* Read device status word */
	devStat = pHandle->pFamFx->devGetStatus(pHandle);

	/* Convert to standard status values */
	if ((devStat & (STATUS_P_ERR | STATUS_W_ERR)) != 0) {
		if ((devStat & STATUS_P_ERR) != 0) {
			status |= SPIFI_STAT_PROGERR;
		}
		if ((devStat & STATUS_W_ERR) != 0) {
			status |= SPIFI_STAT_ERASEERR;
		}

		/* Only clear status if necessary */
		if (clearStatus) {
			pHandle->pFamFx->devClearStatus(pHandle);
		}
	}
	if ((devStat & STATUS_BPMASK) != 0) {
		if ((devStat & STATUS_BPMASK) == STATUS_BPMASK) {
			status |= SPIFI_STAT_FULLLOCK;
		}
		else {
			status |= SPIFI_STAT_PARTLOCK;
		}
	}
	if ((devStat & STATUS_WIP) != 0) {
		status |= SPIFI_STAT_BUSY;
	}

	return status;
}

/* lock/ unlock commands */
static SPIFI_ERR_T spifiFamFxLockDeviceCmd(const SPIFI_HANDLE_T *pHandle, SPIFI_PCMD_LOCK_UNLOCK_T cmd, uint32_t data)
{
	SPIFI_ERR_T status = SPIFI_ERR_NOTSUPPORTED;

	if ((cmd == SPIFI_PCMD_UNLOCK_DEVICE) || (cmd == SPIFI_PCMD_LOCK_DEVICE)) {
		uint32_t stat;

		/* Get current status */
		stat = pHandle->pFamFx->devGetStatus(pHandle);

		if (cmd == SPIFI_PCMD_UNLOCK_DEVICE) {
			/* Clear lock bits only if they are locked */
			if ((stat & STATUS_BPMASK) != 0) {
				stat &= ~STATUS_BPMASK;
				/* Write updated value back to status register */
				pHandle->pFamFx->devSetStatus(pHandle, stat);
			}
		}
		else {
			/* Clear lock bits only if they are locked */
			if ((stat & STATUS_BPMASK) != STATUS_BPMASK) {
				stat |= STATUS_BPMASK;
				/* Write updated value back to status register */
				pHandle->pFamFx->devSetStatus(pHandle, stat);
			}
		}
		status = SPIFI_ERR_NONE;
	}

	return status;
}

/* Bulk Erase*/
static SPIFI_ERR_T spifiFamFxEraseAll(const SPIFI_HANDLE_T *pHandle)
{
	SPIFI_ERR_T status;
	LPC_SPIFI_CHIPHW_T *pSpifiCtrlAddr = (LPC_SPIFI_CHIPHW_T *) pHandle->pInfoData->spifiCtrlAddr;

	status = spifiPrvCheckWriteState(pHandle);
	if (status == SPIFI_ERR_NONE) {
		spifiPrvSetWREN(pSpifiCtrlAddr);
		spifi_HW_SetCmd(pSpifiCtrlAddr,
						(SPIFI_CMD_OPCODE(CMD_C7_BE) |
						 SPIFI_CMD_FIELDFORM(SPIFI_FIELDFORM_ALL_SERIAL) |
						 SPIFI_CMD_FRAMEFORM(SPIFI_FRAMEFORM_OP)));

		spifi_HW_WaitCMD(pSpifiCtrlAddr);

		/* Device wait for device to become ready */
		spifiPrvWaitUnBusy(pHandle);
	}

	return status;
}

/* Erase a block by block number */
static SPIFI_ERR_T spifiFamFxEraseBlock(const SPIFI_HANDLE_T *pHandle, uint32_t blockNum)
{
	uint16_t stat;
	uint32_t addr;
	SPIFI_ERR_T status = SPIFI_ERR_RANGE;
	LPC_SPIFI_CHIPHW_T *pSpifiCtrlAddr = (LPC_SPIFI_CHIPHW_T *) pHandle->pInfoData->spifiCtrlAddr;

	if (blockNum < pHandle->pInfoData->numBlocks) {
		status = spifiPrvCheckWriteState(pHandle);
		if (status == SPIFI_ERR_NONE) {
			addr = blockNum * pHandle->pInfoData->blockSize;
			/* Only clear status if necessary */
			pHandle->pFamFx->devClearStatus(pHandle);

			spifiPrvSetWREN(pSpifiCtrlAddr);

			spifi_HW_SetAddr(pSpifiCtrlAddr, addr);
			if (pHandle->pInfoData->pDeviceData->caps & SPIFI_CAP_4BYTE_ADDR) {
				spifi_HW_SetCmd(pSpifiCtrlAddr,
											(SPIFI_CMD_OPCODE(CMD_DC_SE) |
											 SPIFI_CMD_FIELDFORM(SPIFI_FIELDFORM_ALL_SERIAL) |
											 SPIFI_CMD_FRAMEFORM(SPIFI_FRAMEFORM_OP_4ADDRESS)));
			}
			else { /* Setup for a 3 Byte address erase */
				spifi_HW_SetCmd(pSpifiCtrlAddr,
							(SPIFI_CMD_OPCODE(CMD_D8_SE) |
							 SPIFI_CMD_FIELDFORM(SPIFI_FIELDFORM_ALL_SERIAL) |
							 SPIFI_CMD_FRAMEFORM(SPIFI_FRAMEFORM_OP_3ADDRESS)));
			}
			spifi_HW_WaitCMD(pSpifiCtrlAddr);

			/* If blocking is disabled, exit now */
			if ((pHandle->pInfoData->opts & SPIFI_OPT_NOBLOCK) == 0) {
				/* Device wait for device to become ready */
				spifiPrvWaitUnBusy(pHandle);

				/* Read status and check error bits */
				stat = spifiFamFxGetDeviceStatus(pHandle, 0);
				if ((stat & SPIFI_STAT_ERASEERR) != 0) {
					status = SPIFI_ERR_ERASEERR;
				}
			}
		}
	}

	return status;
}

/* Erase a block by sub-block number */
static SPIFI_ERR_T spifiFamFxEraseSubBlock(const SPIFI_HANDLE_T *pHandle, uint32_t subBlockNum)
{
	uint16_t stat;
	uint32_t addr;
	SPIFI_ERR_T status = SPIFI_ERR_RANGE;
	LPC_SPIFI_CHIPHW_T *pSpifiCtrlAddr = (LPC_SPIFI_CHIPHW_T *) pHandle->pInfoData->spifiCtrlAddr;

	if (subBlockNum < pHandle->pInfoData->numSubBlocks) {
		status = spifiPrvCheckWriteState(pHandle);
		if (status == SPIFI_ERR_NONE) {
			addr = subBlockNum * pHandle->pInfoData->subBlockSize;
			/* Only clear status if necessary */
			pHandle->pFamFx->devClearStatus(pHandle);

			spifiPrvSetWREN(pSpifiCtrlAddr);

			spifi_HW_SetAddr(pSpifiCtrlAddr, addr);

			spifi_HW_SetCmd(pSpifiCtrlAddr,
						(SPIFI_CMD_OPCODE(CMD_20_P4E) |
						 SPIFI_CMD_FIELDFORM(SPIFI_FIELDFORM_ALL_SERIAL) |
						 SPIFI_CMD_FRAMEFORM(SPIFI_FRAMEFORM_OP_3ADDRESS)));

			spifi_HW_WaitCMD(pSpifiCtrlAddr);

			/* If blocking is disabled, exit now */
			if ((pHandle->pInfoData->opts & SPIFI_OPT_NOBLOCK) == 0) {
				/* Device wait for device to become ready */
				spifiPrvWaitUnBusy(pHandle);

				/* Read status and check error bits */
				stat = spifiFamFxGetDeviceStatus(pHandle, 0);
				if ((stat & SPIFI_STAT_ERASEERR) != 0) {
					status = SPIFI_ERR_ERASEERR;
				}
			}
		}
	}

	return status;
}

/* Program a region */
static SPIFI_ERR_T spifiFamFxPageProgram(const SPIFI_HANDLE_T *pHandle,
										 uint32_t addr,
										 const uint32_t *writeBuff,
										 uint32_t bytes)
{
	uint16_t stat;
	uint8_t *writeBuff8;
	SPIFI_ERR_T status = SPIFI_ERR_PAGESIZE;
	LPC_SPIFI_CHIPHW_T *pSpifiCtrlAddr = (LPC_SPIFI_CHIPHW_T *) pHandle->pInfoData->spifiCtrlAddr;
	uint32_t cmdOnlyValue;
	uint32_t dwords;

	if (bytes <= pHandle->pInfoData->pageSize) {
		status = spifiPrvCheckWriteState(pHandle);
		if (status == SPIFI_ERR_NONE) {
			/* Get the program cmd value for this device */
			pHandle->pFamFx->devGetWriteCmd(pHandle, &cmdOnlyValue);

			/* Get the number of dwords to write */
			dwords = bytes >> 2;

			/* process by bytes if amount isn't even number of dwords */
			if (bytes & 0x3) {

				writeBuff8 = (uint8_t *) writeBuff;

				/* Only clear status if the device requires it and set write enable*/
				pHandle->pFamFx->devClearStatus(pHandle);
				spifiPrvSetWREN(pSpifiCtrlAddr);

				spifi_HW_SetAddr(pSpifiCtrlAddr, addr);
				spifi_HW_SetCmd(pSpifiCtrlAddr, cmdOnlyValue | SPIFI_CMD_DATALEN(bytes));
				/* Write data */
				while (bytes) {
					spifi_HW_SetData8(pSpifiCtrlAddr, *writeBuff8);
					++writeBuff8;
					--bytes;
				}
				spifi_HW_WaitCMD(pSpifiCtrlAddr);
			}
			else if (dwords) {
				uint32_t cmdValue = cmdOnlyValue | SPIFI_CMD_DATALEN(dwords << 2);

				/* Only clear status if the device requires it and set write enable */
				pHandle->pFamFx->devClearStatus(pHandle);
				spifiPrvSetWREN(pSpifiCtrlAddr);

				/* Set address and increment for any remaining */
				spifi_HW_SetAddr(pSpifiCtrlAddr, addr);

				/* Finally send command and write the data */
				spifi_HW_SetCmd(pSpifiCtrlAddr, cmdValue);
				while (dwords) {
					spifi_HW_SetData32(pSpifiCtrlAddr, *writeBuff);
					++writeBuff;
					--dwords;
				}
				spifi_HW_WaitCMD(pSpifiCtrlAddr);
			}
		}

		/* If block is disabled, exit now */
		if ((pHandle->pInfoData->opts & SPIFI_OPT_NOBLOCK) == 0) {
			/* Device wait for device to become ready */
			spifiPrvWaitUnBusy(pHandle);

			/* Read status and check error bits */
			stat = spifiFamFxGetDeviceStatus(pHandle, 0);
			if ((stat & SPIFI_STAT_PROGERR) != 0) {
				status = SPIFI_ERR_PROGERR;
			}
		}
	}

	return status;
}

/* Read a region */
static SPIFI_ERR_T spifiFamFxReadDevice(const SPIFI_HANDLE_T *pHandle,
										uint32_t addr,
										uint32_t *readBuff,
										uint32_t bytes)
{
	uint8_t *readBuff8 = (uint8_t *) readBuff;
	SPIFI_ERR_T status = SPIFI_ERR_RANGE;
	LPC_SPIFI_CHIPHW_T *pSpifiCtrlAddr = (LPC_SPIFI_CHIPHW_T *) pHandle->pInfoData->spifiCtrlAddr;
	uint32_t cmdOnlyValue;
	uint32_t cmdValue;
	uint32_t dwords;

	/* Limit read to controller data limit in bytes */
	if (bytes <= pHandle->pInfoData->maxReadSize) {
		/* Get the number of dwords to read */
		dwords = bytes >> 2;
		bytes -= (dwords << 2);

		/* Get the command value to program the SPIFI controller */
		pHandle->pFamFx->devGetReadCmd(pHandle, 0, &cmdOnlyValue, NULL);
		if (dwords) {
			cmdValue = cmdOnlyValue | SPIFI_CMD_DATALEN(dwords << 2);

			/* Specify the intermediate data byte (turn off). */
			spifi_HW_SetIDATA(pSpifiCtrlAddr, 0xFF);

			/* Set the address and increment for any remaining bytes */
			spifi_HW_SetAddr(pSpifiCtrlAddr, addr);
			addr += (dwords << 2);

			spifi_HW_SetCmd(pSpifiCtrlAddr, cmdValue);
			while (dwords) {
				*readBuff = spifi_HW_GetData32(pSpifiCtrlAddr);
				++readBuff;
				--dwords;
			}
			spifi_HW_WaitCMD(pSpifiCtrlAddr);
		}

		if (bytes) {
			readBuff8 = (uint8_t *) readBuff;
			cmdValue = cmdOnlyValue | SPIFI_CMD_DATALEN(bytes);

			/* Specify the intermediate data byte (turn off). */
			spifi_HW_SetIDATA(pSpifiCtrlAddr, 0xFF);

			spifi_HW_SetAddr(pSpifiCtrlAddr, addr);
			spifi_HW_SetCmd(pSpifiCtrlAddr, cmdValue);

			/* Read data */
			while (bytes) {
				*readBuff8 = spifi_HW_GetData8(pSpifiCtrlAddr);
				++readBuff8;
				--bytes;
			}
			spifi_HW_WaitCMD(pSpifiCtrlAddr);
		}
		status = SPIFI_ERR_NONE;
	}

	return status;
}

/* Enable or disable software write protect state */
static SPIFI_ERR_T spifiFamFxResetDevice(const SPIFI_HANDLE_T *pHandle)
{
	return SPIFI_ERR_NOTSUPPORTED;
}

/* Setup a device */
static SPIFI_ERR_T spifiFamFxDeviceSetup(SPIFI_HANDLE_T *pHandle, uint32_t spifiCtrlAddr, uint32_t baseAddr)
{
	/* Common Command Set family function table */
	static  SPIFI_FAM_FX_T fxTable;

	fxTable.lockCmd = spifiFamFxLockDeviceCmd;
	fxTable.eraseAll = spifiFamFxEraseAll;
	fxTable.eraseBlock = spifiFamFxEraseBlock;
	fxTable.eraseSubBlock = spifiFamFxEraseSubBlock;
	fxTable.pageProgram = spifiFamFxPageProgram;
	fxTable.read = spifiFamFxReadDevice;
	fxTable.reset = spifiFamFxResetDevice;
	fxTable.getStatus = spifiFamFxGetDeviceStatus;
	fxTable.subBlockCmd = NULL;	/* Use generic handler in spifilib_dev_common.c */

	/* Initialize the device specific function pointers */
	fxTable.devInitDeInit = spifiDeviceAssignFxInitDeInit(pHandle);
	fxTable.devClearStatus = spifiDeviceAssignFxClearStatus(pHandle);
	fxTable.devGetStatus = spifiDeviceAssignFxGetStatus(pHandle);
	fxTable.devSetStatus = spifiDeviceAssignFxSetStatus(pHandle);
	fxTable.devSetOpts = spifiDeviceAssignFxSetOptions(pHandle);
	fxTable.devGetReadCmd = spifiDeviceAssignFxReadCmd(pHandle);
	fxTable.devGetWriteCmd = spifiDeviceAssignFxWriteCmd(pHandle);

	/* save pointer to family function table */
	pHandle->pFamFx = &fxTable;

	return SPIFI_ERR_NONE;
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/
SPIFI_FAM_NODE_T *spifi_REG_FAMILY_CommonCommandSet(void)
{
	/* Variables declared static so they will persist after function returns. */
	/* All members are assigned at run-time so that position independent code
	   will know the address */
	static SPIFI_DEV_NODE_T devListBase = {0};	/* List base to hold devices */
	static SPIFI_FAM_NODE_T devFamily;			/* Family node to hold family descriptor */
	static SPIFI_FAM_DESC_T famDesc;			/* Family descriptor (holds all info about family) */
	static uint32_t devCount = 0;				/* Variable to keep track of # registered devices */

	/* Protect against multiple calls to register the same family */
	if (devCount) {
		return NULL;
	}

	/* Make sure that the base list is empty and the count reflects 0 */
	devListBase.pNext = NULL;
	devCount = 0;

	/* Store the device specific info so it can be returned */
	famDesc.pFamName = "Common SPIFI Command Set";

	/* Save the pointer to the device list and count */
	famDesc.pDevList = &devListBase;
	famDesc.pDevCount = &devCount;

	famDesc.prvContextSize = 0;					/* Reserve space for private data (this family doesn't need any)*/
	famDesc.pPrvDevGetID = NULL;			/* Use the generic readID routine */
	famDesc.pPrvDevSetup = spifiFamFxDeviceSetup;		/* Provide Fx to handle setup */

	/* Save the descriptor in the handle */
	devFamily.pDesc = &famDesc;

	/* Begin Winbond devices */
	#if SPIFI_DEVICE_ALL || SPIFI_DEVICE_W25Q80BV
	/* Add support for W25Q80BV */
	{
		static const SPIFI_DEVICE_DATA_T pData = {
			"W25Q80BV",
			{{0xEF, 0x40, 0x14}, 0, {0}},	/* JEDEC ID, extCount, ext data  */
			(SPIFI_CAP_DUAL_READ | SPIFI_CAP_QUAD_READ | SPIFI_CAP_QUAD_WRITE | SPIFI_CAP_FULLLOCK |
			 SPIFI_CAP_NOBLOCK | SPIFI_CAP_SUBBLKERASE),
			16,						/* # of blocks */
			0x10000,				/* block size */
			256,					/* # of sub-blocks */
			0x1000,					/* sub-block size */
			0x100,					/* page size */
			MAX_SINGLE_READ,					/* max single read bytes */
			104,				/* max clock rate in MHz */
			104,				/* max read clock rate in MHz */
			104,				/* max high speed read clock rate in MHz */
			104,				/* max program clock rate in MHz */
			104,				/* max high speed program clock rate in MHz */
			FX_spifiDeviceDataInitDeinit,	/* (Fx Id) use generic deviceInit / deInit */
			FX_spifiDeviceDataClearStatusNone,	/* (Fx Id) Does not have persistent status */
			FX_spifiDeviceDataGetStatusW25Q80BV,	/* (Fx Id) getStatus */
			FX_spifiDeviceDataSetStatusS25FL032P,	/* (Fx Id) setStatus (uses S25FL032P variant) */
			FX_spifiDeviceDataSetOptsQuadModeBit9,		/* (Fx Id) to set/clr options */
			FX_spifiDeviceInitReadCommand,	/* (Fx Id) to get memoryMode Cmd */
			FX_spifiDeviceInitWriteCommand	/* (Fx Id) to get program Cmd */
		};
		static SPIFI_DEV_NODE_T data;			/* Create persistent node */

		data.pDevData = &pData;					/* save the data in the node */
		spifiDevRegister(&devFamily, &data);	/* Register the new device */
	}
	#endif
	#if SPIFI_DEVICE_ALL || SPIFI_DEVICE_W25Q64FV
	/* Add support for W25Q64FV */
	{
		static const SPIFI_DEVICE_DATA_T pData = {
			"W25Q64FV",
			{{0xEF, 0x40, 0x17}, 0, {0}},	/* JEDEC ID, extCount, ext data  */
			(SPIFI_CAP_DUAL_READ | SPIFI_CAP_QUAD_READ  | SPIFI_CAP_QUAD_WRITE | SPIFI_CAP_FULLLOCK |
			 SPIFI_CAP_NOBLOCK | SPIFI_CAP_SUBBLKERASE),
			128,						/* # of blocks */
			0x10000,				/* block size */
			2048,					/* # of sub-blocks */
			0x1000,					/* sub-block size */
			0x100,					/* page size */
			MAX_SINGLE_READ,					/* max single read bytes */
			104,				/* max clock rate in MHz */
			104,				/* max read clock rate in MHz */
			104,				/* max high speed read clock rate in MHz */
			104,				/* max program clock rate in MHz */
			104,				/* max high speed program clock rate in MHz */
			FX_spifiDeviceDataInitDeinit,	/* (Fx Id) use generic deviceInit / deInit */
			FX_spifiDeviceDataClearStatusNone,	/* (Fx Id) Does not have persistent status */
			FX_spifiDeviceDataGetStatusW25Q80BV,	/* (Fx Id) getStatus */
			FX_spifiDeviceDataSetStatusS25FL032P,	/* (Fx Id) setStatus (uses S25FL032P variant) */
			FX_spifiDeviceDataSetOptsQuadModeBit9,	/* (Fx Id) to set/clr options */
			FX_spifiDeviceInitReadCommand,	/* (Fx Id) to get memoryMode Cmd */
			FX_spifiDeviceInitWriteCommand		/* (Fx Id) to get program Cmd */
		};
		static SPIFI_DEV_NODE_T data;			/* Create persistent node */

		data.pDevData = &pData;					/* save the data in the node */
		spifiDevRegister(&devFamily, &data);	/* Register the new device */
	}
	#endif
	#if SPIFI_DEVICE_ALL || SPIFI_DEVICE_W25Q32FV
	/* Add support for W25Q32FV */
	{
		static const SPIFI_DEVICE_DATA_T pData = {
			"W25Q32FV",
			{{0xEF, 0x40, 0x16}, 0, {0}},	/* JEDEC ID, extCount, ext data  */
			(SPIFI_CAP_DUAL_READ | SPIFI_CAP_QUAD_READ | SPIFI_CAP_QUAD_WRITE | SPIFI_CAP_FULLLOCK |
			 SPIFI_CAP_NOBLOCK | SPIFI_CAP_SUBBLKERASE),
			64,						/* # of blocks */
			0x10000,				/* block size */
			1024,					/* # of sub-blocks */
			0x1000,					/* sub-block size */
			0x100,					/* page size */
			MAX_SINGLE_READ,					/* max single read bytes */
			104,				/* max clock rate in MHz */
			104,				/* max read clock rate in MHz */
			104,				/* max high speed read clock rate in MHz */
			104,				/* max program clock rate in MHz */
			104,				/* max high speed program clock rate in MHz */
			FX_spifiDeviceDataInitDeinit,	/* (Fx Id) use generic deviceInit / deInit */
			FX_spifiDeviceDataClearStatusNone,	/* (Fx Id) Does not have persistent status */
			FX_spifiDeviceDataGetStatusW25Q80BV,	/* (Fx Id) getStatus */
			FX_spifiDeviceDataSetStatusS25FL032P,	/* (Fx Id) setStatus (uses S25FL032P variant) */
			FX_spifiDeviceDataSetOptsQuadModeBit9,	/* (Fx Id) to set/clr options */
			FX_spifiDeviceInitReadCommand,	/* (Fx Id) to get memoryMode Cmd */
			FX_spifiDeviceInitWriteCommand		/* (Fx Id) to get program Cmd */
		};
		static SPIFI_DEV_NODE_T data;			/* Create persistent node */

		data.pDevData = &pData;					/* save the data in the node */
		spifiDevRegister(&devFamily, &data);	/* Register the new device */
	}
	#endif
	/* Begin Spansion devices */
	#if SPIFI_DEVICE_ALL || SPIFI_DEVICE_S25FL512S
	/* Add support for S25FL512S 256K Sector */
	{
		static const SPIFI_DEVICE_DATA_T pData = {
			"S25FL512S",
			{{0x01, 0x02, 0x20}, 0, {0}},	/* JEDEC ID, extCount, ext data  */
			(SPIFI_CAP_4BYTE_ADDR | SPIFI_CAP_DUAL_READ  | SPIFI_CAP_QUAD_READ | SPIFI_CAP_QUAD_WRITE | SPIFI_CAP_FULLLOCK | SPIFI_CAP_NOBLOCK),
			256,						/* # of blocks */
			0x40000,				/* block size */
			0,						/* # of sub-blocks (Does NOT support full sub-block erase) */
			0,						/* sub-block size */
			512,					/* page size */
			MAX_SINGLE_READ,					/* max single read bytes */
			80,				/* max clock rate in MHz */
			104,				/* max read clock rate in MHz */
			80,				/* max high speed read clock rate in MHz */
			104,				/* max program clock rate in MHz */
			80,				/* max high speed program clock rate in MHz */
			FX_spifiDeviceDataInitDeinit,	/* (Fx Id) use generic deviceInit / deInit */
			FX_spifiDeviceDataClearStatusS25FL032P,	/* (Fx Id) has persistent bits in status register */
			FX_spifiDeviceDataGetStatusS25FL032P,	/*  (Fx Id) getStatus */
			FX_spifiDeviceDataSetStatusS25FL032P,	/* (Fx Id) setStatus */
			FX_spifiDeviceDataSetOptsQuadModeBit9,	/* (Fx Id) to set/clr options */
			FX_spifiDevice4BInitReadCommand,	/* (Fx Id) to get memoryMode Cmd */
			FX_spifiDevice4BInitWriteCommand	/* (Fx Id) to get program Cmd */
		};
		static SPIFI_DEV_NODE_T data;			/* Create persistent node */

		data.pDevData = &pData;					/* save the data in the node */
		spifiDevRegister(&devFamily, &data);	/* Register the new device */
	}
	#endif
	#if SPIFI_DEVICE_ALL || SPIFI_DEVICE_S25FL256S_256K
	/* Add support for S25FL256S 256K Sector */
	{
		static const SPIFI_DEVICE_DATA_T pData = {
			"S25FL256S 256kSec",
			{{0x01, 0x02, 0x19}, 2, {0x4D, 0x0}},	/* JEDEC ID, extCount, ext data  */
			(SPIFI_CAP_4BYTE_ADDR | SPIFI_CAP_DUAL_READ | SPIFI_CAP_QUAD_READ | SPIFI_CAP_QUAD_WRITE | SPIFI_CAP_FULLLOCK | SPIFI_CAP_NOBLOCK),
			128,						/* # of blocks */
			0x40000,				/* block size */
			0,						/* # of sub-blocks (Does NOT support full sub-block erase) */
			0,						/* sub-block size */
			256,					/* page size */
			MAX_SINGLE_READ,					/* max single read bytes */
			80,				/* max clock rate in MHz */
			104,				/* max read clock rate in MHz */
			80,				/* max high speed read clock rate in MHz */
			104,				/* max program clock rate in MHz */
			80,				/* max high speed program clock rate in MHz */
			FX_spifiDeviceDataInitDeinit,	/* (Fx Id) use generic deviceInit / deInit */
			FX_spifiDeviceDataClearStatusS25FL032P,	/* (Fx Id) has persistent bits in status register */
			FX_spifiDeviceDataGetStatusS25FL032P,	/*  (Fx Id) getStatus */
			FX_spifiDeviceDataSetStatusS25FL032P,	/* (Fx Id) setStatus */
			FX_spifiDeviceDataSetOptsQuadModeBit9,	/* (Fx Id) to set/clr options */
			FX_spifiDevice4BInitReadCommand,	/* (Fx Id) to get memoryMode Cmd */
			FX_spifiDevice4BInitWriteCommand	/* (Fx Id) to get program Cmd */
		};
		static SPIFI_DEV_NODE_T data;			/* Create persistent node */

		data.pDevData = &pData;					/* save the data in the node */
		spifiDevRegister(&devFamily, &data);	/* Register the new device */
	}
	#endif
	#if SPIFI_DEVICE_ALL || SPIFI_DEVICE_S25FL256S_64K
	/* Add support for S25FL256S 64k sector */
	{
		static const SPIFI_DEVICE_DATA_T pData = {
			"S25FL256S 64kSec",
			{{0x01, 0x02, 0x19}, 2, {0x4D, 0x01}},	/* JEDEC ID, extCount, ext data  */
			(SPIFI_CAP_4BYTE_ADDR | SPIFI_CAP_DUAL_READ | SPIFI_CAP_QUAD_READ | SPIFI_CAP_QUAD_WRITE | SPIFI_CAP_FULLLOCK | SPIFI_CAP_NOBLOCK),
			512,					/* # of blocks */
			0x10000,				/* block size */
			0,						/* # of sub-blocks (Does NOT support full sub-block erase) */
			0,						/* sub-block size 0x1000 */
			256,					/* page size */
			MAX_SINGLE_READ,					/* max single read bytes */
			80,				/* max clock rate in MHz */
			104,				/* max read clock rate in MHz */
			80,				/* max high speed read clock rate in MHz */
			104,				/* max program clock rate in MHz */
			80,				/* max high speed program clock rate in MHz */
			FX_spifiDeviceDataInitDeinit,	/* (Fx Id) use generic deviceInit / deInit */
			FX_spifiDeviceDataClearStatusS25FL032P,	/* (Fx Id) has persistent bits in status register */
			FX_spifiDeviceDataGetStatusS25FL032P,	/* (Fx Id) getStatus */
			FX_spifiDeviceDataSetStatusS25FL032P,	/* (Fx Id) setStatus */
			FX_spifiDeviceDataSetOptsQuadModeBit9,	/* (Fx Id) to set/clr options */
			FX_spifiDevice4BInitReadCommand,	/* (Fx Id) to get memoryMode Cmd */
			FX_spifiDevice4BInitWriteCommand	/* (Fx Id) to get program Cmd */
		};
		static SPIFI_DEV_NODE_T data;			/* Create persistent node */

		data.pDevData = &pData;					/* save the data in the node */
		spifiDevRegister(&devFamily, &data);	/* Register the new device */
	}
	#endif
	#if SPIFI_DEVICE_ALL || SPIFI_DEVICE_S25FL164K
	/* Add support for S25FL164K */
	{
		static const SPIFI_DEVICE_DATA_T pData = {
			"S25FL164K",
			{{0x01, 0x40, 0x17}, 0, {0}},	/* JEDEC ID, extCount, ext data  */
			(SPIFI_CAP_DUAL_READ | SPIFI_CAP_QUAD_READ | SPIFI_CAP_FULLLOCK | SPIFI_CAP_NOBLOCK | SPIFI_CAP_SUBBLKERASE),	/* does NOT support Quad Write */
			128,					/* # of blocks */
			0x10000,				/* block size */
			2048,					/* # of sub-blocks */
			0x1000,					/* sub-block size */
			256,					/* page size */
			MAX_SINGLE_READ,					/* max single read bytes */
			50,				/* max clock rate in MHz */
			97,				/* max read clock rate in MHz */
			97,				/* max high speed read clock rate in MHz */
			97,				/* max program clock rate in MHz */
			97,				/* max high speed program clock rate in MHz */
			FX_spifiDeviceDataInitDeinitS25FL164K,	/* (Fx Id) device init / deInit */
			FX_spifiDeviceDataClearStatusNone,	/* (Fx Id) No persistent status */
			FX_spifiDeviceDataGetStatusS25FL164K,	/* (Fx Id) getStatus */
			FX_spifiDeviceDataSetStatusS25FL164K,	/* (Fx Id) setStatus */
			FX_spifiDeviceDataSetOptsQuadModeBit9,	/* (Fx Id) to set/clr options */
			FX_spifiDeviceInitReadCommand,	/* (Fx Id) to get memoryMode Cmd */
			FX_spifiDeviceInitWriteCommand	/* (Fx Id) to get program Cmd */
		};
		static SPIFI_DEV_NODE_T data;			/* Create persistent node */

		data.pDevData = &pData;					/* save the data in the node */
		spifiDevRegister(&devFamily, &data);	/* Register the new device */
	}
	#endif
	#if SPIFI_DEVICE_ALL || SPIFI_DEVICE_S25FL129P_256K
	/* Add support for S25FL129P 256K Sector. Clone: S25FL128S */
	{
		static const SPIFI_DEVICE_DATA_T pData = {
			"S25FL129P 256kSec",
			{{0x01, 0x20, 0x18}, 2, {0x4D, 0x0}},	/* JEDEC ID, extCount, ext data  */
			(SPIFI_CAP_DUAL_READ | SPIFI_CAP_QUAD_READ | SPIFI_CAP_QUAD_WRITE | SPIFI_CAP_FULLLOCK | SPIFI_CAP_NOBLOCK),
			64,						/* # of blocks */
			0x40000,				/* block size */
			0,						/* # of sub-blocks (Does NOT support full sub-block erase) */
			0,						/* sub-block size */
			256,					/* page size */
			MAX_SINGLE_READ,					/* max single read bytes */
			80,				/* max clock rate in MHz */
			104,				/* max read clock rate in MHz */
			80,				/* max high speed read clock rate in MHz */
			104,				/* max program clock rate in MHz */
			80,				/* max high speed program clock rate in MHz */
			FX_spifiDeviceDataInitDeinit,	/* (Fx Id) use generic deviceInit / deInit */
			FX_spifiDeviceDataClearStatusS25FL032P,	/* (Fx Id) has persistent bits in status register */
			FX_spifiDeviceDataGetStatusS25FL032P,	/*  (Fx Id) getStatus */
			FX_spifiDeviceDataSetStatusS25FL032P,	/* (Fx Id) setStatus */
			FX_spifiDeviceDataSetOptsQuadModeBit9,	/* (Fx Id) to set/clr options */
			FX_spifiDeviceInitReadCommand,	/* (Fx Id) to get memoryMode Cmd */
			FX_spifiDeviceInitWriteCommand	/* (Fx Id) to get program Cmd */
		};
		static SPIFI_DEV_NODE_T data;			/* Create persistent node */

		data.pDevData = &pData;					/* save the data in the node */
		spifiDevRegister(&devFamily, &data);	/* Register the new device */
	}
	#endif
	#if SPIFI_DEVICE_ALL || SPIFI_DEVICE_S25FL129P_64K
	/* Add support for S25FL129P 64k sector */
	{
		static const SPIFI_DEVICE_DATA_T pData = {
			"S25FL129P 64kSec",
			{{0x01, 0x20, 0x18}, 2, {0x4D, 0x01}},	/* JEDEC ID, extCount, ext data  */
			(SPIFI_CAP_DUAL_READ | SPIFI_CAP_QUAD_READ | SPIFI_CAP_QUAD_WRITE | SPIFI_CAP_FULLLOCK | SPIFI_CAP_NOBLOCK),
			256,					/* # of blocks */
			0x10000,				/* block size */
			0,						/* # of sub-blocks (Does NOT support full sub-block erase) */
			0,						/* sub-block size 0x1000 */
			256,					/* page size */
			MAX_SINGLE_READ,					/* max single read bytes */
			80,				/* max clock rate in MHz */
			104,				/* max read clock rate in MHz */
			80,				/* max high speed read clock rate in MHz */
			104,				/* max program clock rate in MHz */
			80,				/* max high speed program clock rate in MHz */
			FX_spifiDeviceDataInitDeinit,	/* (Fx Id) use generic deviceInit / deInit */
			FX_spifiDeviceDataClearStatusS25FL032P,	/* (Fx Id) has persistent bits in status register */
			FX_spifiDeviceDataGetStatusS25FL032P,	/* (Fx Id) getStatus */
			FX_spifiDeviceDataSetStatusS25FL032P,	/* (Fx Id) setStatus */
			FX_spifiDeviceDataSetOptsQuadModeBit9,	/* (Fx Id) to set/clr options */
			FX_spifiDeviceInitReadCommand,	/* (Fx Id) to get memoryMode Cmd */
			FX_spifiDeviceInitWriteCommand	/* (Fx Id) to get program Cmd */
		};
		static SPIFI_DEV_NODE_T data;			/* Create persistent node */

		data.pDevData = &pData;					/* save the data in the node */
		spifiDevRegister(&devFamily, &data);	/* Register the new device */
	}
	#endif
	#if SPIFI_DEVICE_ALL || SPIFI_DEVICE_S25FL064P
	/* Add support for S25FL064P */
	{
		static const SPIFI_DEVICE_DATA_T pData = {
			"S25FL064P",
			{{0x01, 0x02, 0x16}, 1, {0x4d}},	/* JEDEC ID, extCount, ext data  */
			(SPIFI_CAP_DUAL_READ | SPIFI_CAP_QUAD_READ | SPIFI_CAP_QUAD_WRITE | SPIFI_CAP_FULLLOCK | SPIFI_CAP_NOBLOCK),	/* Capabilities */
			128,					/* # of blocks */
			0x10000,				/* block size */
			0,						/* # of sub-blocks  (Does NOT support full sub-block erase) */
			0,						/* sub-block size  0x1000 */
			256,					/* page size */
			MAX_SINGLE_READ,					/* max single read bytes */
			80,				/* max clock rate in MHz */
			104,				/* max read clock rate in MHz */
			80,				/* max high speed read clock rate in MHz */
			104,				/* max program clock rate in MHz */
			80,				/* max high speed program clock rate in MHz */
			FX_spifiDeviceDataInitDeinit,	/* (Fx Id) use generic deviceInit / deInit */
			FX_spifiDeviceDataClearStatusS25FL032P,	/* (Fx Id) has persistent bits in status register */
			FX_spifiDeviceDataGetStatusS25FL032P,	/* (Fx Id) getStatus */
			FX_spifiDeviceDataSetStatusS25FL032P,	/* (Fx Id) setStatus */
			FX_spifiDeviceDataSetOptsQuadModeBit9,	/* (Fx Id) to set/clr options */
			FX_spifiDeviceInitReadCommand,	/* (Fx Id) to get memoryMode Cmd */
			FX_spifiDeviceInitWriteCommand	/* (Fx Id) to get program Cmd */
		};
		static SPIFI_DEV_NODE_T data;			/* Create persistent node */

		data.pDevData = &pData;					/* save the data in the node */
		spifiDevRegister(&devFamily, &data);	/* Register the new device */
	}
	#endif
	#if SPIFI_DEVICE_ALL || SPIFI_DEVICE_S25FL032P
	/* Add support for S25FL032P */
	{
		static const SPIFI_DEVICE_DATA_T pData = {
			"S25FL032P",
			{{0x01, 0x02, 0x15}, 0, {0}},	/* JEDEC ID, extCount, ext data  */
			(SPIFI_CAP_DUAL_READ | SPIFI_CAP_QUAD_READ | SPIFI_CAP_QUAD_WRITE | SPIFI_CAP_FULLLOCK | SPIFI_CAP_NOBLOCK),	/* Capabilities */
			64,						/* # of blocks */
			0x10000,				/* block size */
			0,						/* # of sub-blocks  (Does NOT support full sub-block erase) */
			0,						/* sub-block size  0x1000 */
			256,					/* page size */
			MAX_SINGLE_READ,					/* max single read bytes */
			80,				/* max clock rate in MHz */
			104,				/* max read clock rate in MHz */
			80,				/* max high speed read clock rate in MHz */
			104,				/* max program clock rate in MHz */
			80,				/* max high speed program clock rate in MHz */
			FX_spifiDeviceDataInitDeinit,	/* (Fx Id) use generic deviceInit / deInit */
			FX_spifiDeviceDataClearStatusS25FL032P,	/* (Fx Id) has persistent bits in status register */
			FX_spifiDeviceDataGetStatusS25FL032P,	/* (Fx Id) getStatus */
			FX_spifiDeviceDataSetStatusS25FL032P,	/* (Fx Id) setStatus */
			FX_spifiDeviceDataSetOptsQuadModeBit9,	/* Fx* to set/clr options */
			FX_spifiDeviceInitReadCommand,	/* (Fx Id) to get memoryMode Cmd */
			FX_spifiDeviceInitWriteCommand	/* (Fx Id) to get program Cmd */
		};
		static SPIFI_DEV_NODE_T data;			/* Create persistent node */

		data.pDevData = &pData;					/* save the data in the node */
		spifiDevRegister(&devFamily, &data);	/* Register the new device */
	}
	#endif
	#if SPIFI_DEVICE_ALL || SPIFI_DEVICE_S25FL016K
	/* Add support for S25FL016K */
	{
		static const SPIFI_DEVICE_DATA_T pData = {
			"S25FL016K",
			{{0xef, 0x40, 0x15}, 0, {0}},	/* JEDEC ID, extCount, ext data  */
			(SPIFI_CAP_DUAL_READ | SPIFI_CAP_QUAD_READ | SPIFI_CAP_QUAD_WRITE | SPIFI_CAP_FULLLOCK |
			 SPIFI_CAP_NOBLOCK | SPIFI_CAP_SUBBLKERASE),																							/* Capabilities */
			32,						/* # of blocks */
			0x10000,				/* block size */
			512,					/* # of sub-blocks  (Does NOT support full sub-block erase)*/
			0x1000,					/* sub-block size  0x1000 */
			256,					/* page size */
			MAX_SINGLE_READ,					/* max single read bytes */
			80,				/* max clock rate in MHz */
			104,				/* max read clock rate in MHz */
			80,				/* max high speed read clock rate in MHz */
			104,				/* max program clock rate in MHz */
			80,				/* max high speed program clock rate in MHz */
			FX_spifiDeviceDataInitDeinit,	/* (Fx Id) use generic deviceInit / deInit */
			FX_spifiDeviceDataClearStatusNone,	/* (Fx Id) Does not have persistent status */
			FX_spifiDeviceDataGetStatusS25FL032P,	/* (Fx Id) getStatus */
			FX_spifiDeviceDataSetStatusS25FL032P,	/* (Fx Id) setStatus */
			FX_spifiDeviceDataSetOptsQuadModeBit9,	/* (Fx Id) to set/clr options */
			FX_spifiDeviceInitReadCommand,	/* (Fx Id) to get memoryMode Cmd */
			FX_spifiDeviceInitWriteCommand	/* (Fx Id) to get program Cmd */
		};
		static SPIFI_DEV_NODE_T data;			/* Create persistent node */

		data.pDevData = &pData;					/* save the data in the node */
		spifiDevRegister(&devFamily, &data);	/* Register the new device */
	}
	#endif
	/* Begin Maxronix devices */
	#if SPIFI_DEVICE_ALL || SPIFI_DEVICE_MX25L8035E
	/* Add support for MX25L8035E */
	{
		static const SPIFI_DEVICE_DATA_T pData = {
			"MX25L8035E",
			{{0xC2, 0x20, 0x14}, 0, {0}},	/* JEDEC ID, extCount, ext data  */
			(SPIFI_CAP_DUAL_READ | SPIFI_CAP_QUAD_READ | SPIFI_CAP_QUAD_WRITE | SPIFI_CAP_NOBLOCK |
			 SPIFI_CAP_SUBBLKERASE),																						/* capabilities */
			16,						/* # of blocks */
			0x10000,				/* block size */
			256,					/* # of sub-blocks */
			0x1000,					/* sub-block size */
			256,					/* page size */
			MAX_SINGLE_READ,					/* max single read bytes */
			80,				/* max clock rate in MHz */
			108,				/* max read clock rate in MHz */
			108,				/* max high speed read clock rate in MHz */
			104,				/* max program clock rate in MHz */
			104,				/* max high speed program clock rate in MHz */
			FX_spifiDeviceDataInitDeinit,	/* (Fx Id) use generic deviceInit / deInit */
			FX_spifiDeviceDataClearStatusNone,	/* (Fx Id) no persistent status */
			FX_spifiDeviceDataGetStatusMX25L3235E,	/* (Fx Id) getStatus */
			FX_spifiDeviceDataSetStatusMX25L3235E,	/* (Fx Id) setStatus */
			FX_spifiDeviceDataSetOptsQuadModeBit6,	/* (Fx Id) to set/clr options */
			FX_spifiDeviceInitReadCommand,	/* (Fx Id) to get memoryMode Cmd */
			FX_spifiDeviceInitWriteCommandMacronix	/* (Fx Id) to get program Cmd */
		};
		static SPIFI_DEV_NODE_T data;			/* Create persistent node */

		data.pDevData = &pData;					/* save the data in the node */
		spifiDevRegister(&devFamily, &data);	/* Register the new device */
	}
	#endif
	#if SPIFI_DEVICE_ALL || SPIFI_DEVICE_MX25L6435E
	/* Add support for MX25L6435E */
	{
		static const SPIFI_DEVICE_DATA_T pData = {
			"MX25L6435E",
			{{0xC2, 0x20, 0x17}, 0, {0}},	/* JEDEC ID, extCount, ext data  */
			(SPIFI_CAP_DUAL_READ | SPIFI_CAP_QUAD_READ | SPIFI_CAP_QUAD_WRITE | SPIFI_CAP_NOBLOCK |
			 SPIFI_CAP_SUBBLKERASE),																						/* capabilities */
			128,					/* # of blocks */
			0x10000,				/* block size */
			2048,					/* # of sub-blocks */
			0x1000,					/* sub-block size */
			256,					/* page size */
			MAX_SINGLE_READ,					/* max single read bytes */
			80,				/* max clock rate in MHz */
			104,				/* max read clock rate in MHz */
			86,				/* max high speed read clock rate in MHz */
			104,				/* max program clock rate in MHz */
			104,				/* max high speed program clock rate in MHz */
			FX_spifiDeviceDataInitDeinit,	/* (Fx Id) use generic deviceInit / deInit */
			FX_spifiDeviceDataClearStatusNone,	/* (Fx Id) no persistent status */
			FX_spifiDeviceDataGetStatusMX25L3235E,	/* (Fx Id) getStatus function */
			FX_spifiDeviceDataSetStatusMX25L3235E,	/* (Fx Id) setStatus function */
			FX_spifiDeviceDataSetOptsQuadModeBit6,		/* (Fx Id) to set/clr options */
			FX_spifiDeviceInitReadCommand,	/* (Fx Id) to get memoryMode Cmd */
			FX_spifiDeviceInitWriteCommandMacronix	/* (Fx Id) to get program Cmd */
		};
		static SPIFI_DEV_NODE_T data;			/* Create persistent node */

		data.pDevData = &pData;					/* save the data in the node */
		spifiDevRegister(&devFamily, &data);	/* Register the new device */
	}
	#endif
	#if SPIFI_DEVICE_ALL || SPIFI_DEVICE_MX25L3235E
	/* Add support for MX25L3235E */
	{
		static const SPIFI_DEVICE_DATA_T pData = {
			"MX25L3235E",
			{{0xC2, 0x20, 0x16}, 0, {0}},	/* JEDEC ID, extCount, ext data  */
			(SPIFI_CAP_DUAL_READ | SPIFI_CAP_QUAD_READ | SPIFI_CAP_QUAD_WRITE | SPIFI_CAP_NOBLOCK |
			 SPIFI_CAP_SUBBLKERASE),																						/* capabilities */
			64,						/* # of blocks */
			0x10000,				/* block size */
			1024,					/* # of sub-blocks */
			0x1000,					/* sub-block size */
			256,					/* page size */
			MAX_SINGLE_READ,		/* max single read bytes */
			80,				/* max clock rate in MHz */
			104,				/* max read clock rate in MHz */
			86,				/* max high speed read clock rate in MHz */
			104,				/* max program clock rate in MHz */
			104,				/* max high speed program clock rate in MHz */
			FX_spifiDeviceDataInitDeinit,	/* (Fx Id) use generic deviceInit / deInit */
			FX_spifiDeviceDataClearStatusNone,		/* (Fx Id) no persistent status */
			FX_spifiDeviceDataGetStatusMX25L3235E,	/* (Fx Id) getStatus function */
			FX_spifiDeviceDataSetStatusMX25L3235E,	/* (Fx Id) setStatus function */
			FX_spifiDeviceDataSetOptsQuadModeBit6,	/* (Fx Id) to set/clr options */
			FX_spifiDeviceInitReadCommand,	/* (Fx Id) to get memoryMode Cmd */
			FX_spifiDeviceInitWriteCommandMacronix	/* (Fx Id) to get program Cmd */
		};
		static SPIFI_DEV_NODE_T data;			/* Create persistent node */

		data.pDevData = &pData;					/* save the data in the node */
		spifiDevRegister(&devFamily, &data);	/* Register the new device */
	}
	#endif
	#if SPIFI_DEVICE_ALL || SPIFI_DEVICE_MX25L1635E
	/* Add support for MX25L1635E */
	{
		static const SPIFI_DEVICE_DATA_T pData = {
			"MX25L1635E",
			{{0xC2, 0x25, 0x15}, 0, {0}},	/* JEDEC ID, extCount, ext data  */
			(SPIFI_CAP_DUAL_READ | SPIFI_CAP_QUAD_READ | SPIFI_CAP_QUAD_WRITE | SPIFI_CAP_NOBLOCK |
			 SPIFI_CAP_SUBBLKERASE),																						/* capabilities */
			32,						/* # of blocks */
			0x10000,				/* block size */
			512,					/* # of sub-blocks */
			0x1000,					/* sub-block size */
			256,					/* page size */
			MAX_SINGLE_READ,		/* max single read bytes */
			80,				/* max clock rate in MHz */
			104,				/* max read clock rate in MHz */
			86,				/* max high speed read clock rate in MHz */
			104,				/* max program clock rate in MHz */
			104,				/* max high speed program clock rate in MHz */
			FX_spifiDeviceDataInitDeinit,	/* (Fx Id) use generic deviceInit / deInit */
			FX_spifiDeviceDataClearStatusNone,		/* (Fx Id) no persistent status */
			FX_spifiDeviceDataGetStatusMX25L3235E,	/* (Fx Id) getStatus function */
			FX_spifiDeviceDataSetStatusMX25L3235E,	/* (Fx Id) setStatus function */
			FX_spifiDeviceDataSetOptsQuadModeBit6,	/* (Fx Id) to set/clr options */
			FX_spifiDeviceInitReadCommand,	/* (Fx Id) to get memoryMode Cmd */
			FX_spifiDeviceInitWriteCommandMacronix	/* (Fx Id) to get program Cmd */
		};
		static SPIFI_DEV_NODE_T data;			/* Create persistent node */

		data.pDevData = &pData;					/* save the data in the node */
		spifiDevRegister(&devFamily, &data);	/* Register the new device */
	}
	#endif

	/* finally return the family device structure */
	return &devFamily;
}
