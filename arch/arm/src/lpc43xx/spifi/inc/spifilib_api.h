/****************************************************************************
 * arch/arm/src/lpc43xx/spifi/inc/spifilib_api.h
 *
 * Copyright(C) NXP Semiconductors, 2014
 * All rights reserved.
 *
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * LPC products.  This software is supplied "AS IS" without any warranties of
 * any kind, and NXP Semiconductors and its licenser disclaim any and
 * all warranties, express or implied, including all implied warranties of
 * merchantability, fitness for a particular purpose and non-infringement of
 * intellectual property rights.  NXP Semiconductors assumes no
 * responsibility or liability for the use of the software, conveys no
 * license or rights under any patent, copyright, mask work right, or any
 * other intellectual property rights in or to any products. NXP
 * Semiconductors reserves the right to make changes in the software without
 * notification. NXP Semiconductors also makes no representation or warranty
 * that such application will be suitable for the specified use without
 * further testing or modification.
 *
 * Permission to use, copy, modify, and distribute this software and its
 * documentation is hereby granted, under NXP Semiconductors' and its
 * licensor's relevant copyrights in the software, without fee, provided
 * that it is used in conjunction with NXP Semiconductors microcontrollers.
 * This copyright, permission, and disclaimer notice must appear in all
 * copies of this code.
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_LPC43XX_SPIFI_INC_SPIFILIB_API_H
#define __ARCH_ARM_SRC_LPC43XX_SPIFI_INC_SPIFILIB_API_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "spifilib_dev.h"

#ifdef __cplusplus
extern "C"
{
#endif

/* LPCSPIFILIB_API LPCSPIFILIB common API functions
 *
 * These LPCSPIFILIB functions provide an abstracted interface to
 * the LPCSPIFILIB functions. The device API is a private API which should
 * only used to interface with the LPCSPIFILIB core library.
 */

/* LPCSPIFILIB_CMNAPI LPCSPIFILIB library support functions
 * Library support functions are not tied to any specific LPCSPIFILIB device.
 */

/* Report the SPIFILIB version
 * return     SPIFI library version in format MMmm where MM is major number
 * and mm is minor number.
 */

uint16_t spifiGetLibVersion(void);

/* Initialize the SPIFILIB driver
 * spifiCtrlAddr  : Base address of SPIFI controller
 * reset          : true to reset the SPIFI controller, or false to not reset
 * return         SPIFI library error code
 * NOTE This function should be called prior to any other SPIFILIB functions.
 * In most cases, a reset isn't needed. Before calling this function, all
 * board specific functions related to the SPIFI interface must be setup and
 * the SPIFI clock must be enabled. If booting from SPIFI FLASH, this will
 * already be done. If not booting from SPIFI FLASH, the SPIFI FLASH pin
 * muxing and SPIFI controller clock need to be enabled prior to this call.
 */

SPIFI_ERR_T spifiInit(uint32_t spifiCtrlAddr, uint8_t reset);

/* Register a SPIFILIB family driver
 * regFx        : A function which returns persistent device specific data
 *                structure.
 * return       Handle to device specific data structure.
 * NOTE This function should be called prior to calling
 * spifiGetHandleMemSize() or spifiInitDevice().
 */

SPIFI_FAM_NODE_T *spifiRegisterFamily(SPIFI_FAM_NODE_T *(*regFx)(void));

/* Converts a SPIFILIB error code into a meaningful string
 * errCode     : Error code to get string pointer to
 * return       Pointer to string for the passed error code
 */

const char *spifiReturnErrString(SPIFI_ERR_T errCode);

/* Return the number of registered device families in this driver
 * return       number of registered device families in this driver
 */

uint32_t spifiGetSuppFamilyCount(void);

/* Return the driver device family name for a specific index
 * index        : Index (0 - n) where n = number of families returned
 *                    by spifiGetSuppFamilyCount() -1
 * return       a string pointer to the generic device name
 * NOTE Can be used with the spifiGetSuppFamilyCount() to get a list of
 * device families the library is configured for.
 */

const char *spifiGetSuppFamilyName(uint32_t index);

/* Detect and return memory needed for device handle at passed address
 * spifiCtrlAddr  : Base address of SPIFI controller
 * return         The size in bytes this device needs for the call to
 *                InitDevice().
 * If no supported device is detected 0 will be returned.
 * NOTE Selects the first matching device in the library.
 */

uint32_t spifiGetHandleMemSize(uint32_t spifiCtrlAddr);

/* Initialize driver and hardware for a specific device
 * pMem          : Pointer to a 32-bit aligned buffer with a size returned
 *                 from spifiGetHandleMemSize()
 * sizePMem      : Size of the buffer in bytes pass in pMem
 * spifiCtrlAddr : Base address of SPIFI controller
 * baseAddr      : Base address of device
 * return   Returns a pointer to a device handle if successful,
 *          or NULL on an error.
 */

SPIFI_HANDLE_T *spifiInitDevice(void *pMem, uint32_t sizePMem,
                                uint32_t spifiCtrlAddr, uint32_t baseAddr);

/* Set or unset driver options
 * pHandle      : Pointer to a LPCSPIFILIB device handle
 * options      : Options to set or unset, an OR'ed value of SPIFI_OPT_xxx
 *                values
 * (example #SPIFI_OPT_USE_QUAD | #SPIFI_OPT_NOBLOCK)
 * set          : true to set the passed options, false to clear them
 * return       Nothing
 * NOTE Only options that are supported in the capabilities of the driver
 * can be set or unset.
 */

SPIFI_ERR_T spifiDevSetOpts(SPIFI_HANDLE_T *pHandle,
                            uint32_t options, uint8_t set);

/* LPCSPIFILIB_DEVAPI LPCSPIFILIB library device functions
 * Device functions are used to perform LPCSPIFILIB device operations.
 */

/* Add device to family driver
 * pFamily      : Pointer to a SPIFI_DEV_FAMILY_T family handle
 * pDevData     : Pointer to a persistent SPIFI_DEV_DATA_T device structure
 * return       A SPIFI_ERR_T error code (SPIFI_ERR_NONE for no errors)
 * NOTE This function MUST be called prior to spifiGetHandleMemSize() or
 *       spifiInitDevice()
 */

SPIFI_ERR_T spifiDevRegister(const SPIFI_FAM_NODE_T *pFamily,
                             SPIFI_DEV_NODE_T *pDevData);

/* Returns the number of supported devices within a family
 * pFamily     : Pointer to a SPIFI_DEV_FAMILY_T family handle
 * return      The number of registered devices.
 */

static INLINE uint32_t spifiDevGetCount(const SPIFI_FAM_NODE_T *pFamily)
{
	return *(pFamily->pDesc->pDevCount);
}

/* Enumerates the friendly names of supported devices
 * pContext     : Pointer to a SPIFI_DEV_ENUMERATOR_T context structure
 * reset        : 0 enumerates next device, 1 resets list to beginning
 *                 and returns first device
 * return       A friendly string representing the device,
 *              NULL when list has been exhausted.
 */

const char *spifiDevEnumerateName(SPIFI_DEV_ENUMERATOR_T *pContext,
                                  uint8_t reset);

/* Initialize a detected LPCSPIFILIB device
 * pHandle      : Pointer to a LPCSPIFILIB device handle
 * return       A SPIFI_NO_* error code (SPIFI_ERR_NONE is no errors)
 */

SPIFI_ERR_T spifiDevInit(const SPIFI_HANDLE_T *pHandle);

/* De-initialize a detected LPCSPIFILIB device
 * pHandle      : Pointer to a LPCSPIFILIB device handle
 * return       A SPIFI_NO_* error code (SPIFI_ERR_NONE is no errors)
 */

SPIFI_ERR_T spifiDevDeInit(const SPIFI_HANDLE_T *pHandle);

/* Sets or clears memory mode
 * pHandle      : Pointer to a LPCSPIFILIB device handle
 * enMMode      : true to enable memory mode, false to disable
 * return       A SPIFI_NO_* error code (SPIFI_ERR_NONE is no errors)
 * NOTE	Enter memory mode to enable direct read access for Execute in
 * place code and memory mapped data. Memory mode must be disabled
 * for most operations.
 */

SPIFI_ERR_T spifiDevSetMemMode(const SPIFI_HANDLE_T *pHandle,
                               uint8_t enMMode);

/* Return status of memory mode
 * pSpifi       : Base address of SPIFI controller
 * return       state of memory mode (false = off, true = on)
 */

uint8_t spifiDevGetMemoryMode(const SPIFI_HANDLE_T *pSpifi);

/* Full LPCSPIFILIB device unlock
 * pHandle      : Pointer to a LPCSPIFILIB device handle
 * return       A SPIFI_NO_* error code (SPIFI_ERR_NONE is no errors)
 */

static INLINE SPIFI_ERR_T spifiDevUnlockDevice(const SPIFI_HANDLE_T *pHandle)
{
  return pHandle->pFamFx->lockCmd(pHandle, SPIFI_PCMD_UNLOCK_DEVICE, 0);
}

/* Full LPCSPIFILIB device lock
 * pHandle      : Pointer to a LPCSPIFILIB device handle
 * return       A SPIFI_NO_* error code (SPIFI_ERR_NONE is no errors)
 */

static INLINE SPIFI_ERR_T spifiDevLockDevice(const SPIFI_HANDLE_T *pHandle)
{
  return pHandle->pFamFx->lockCmd(pHandle, SPIFI_PCMD_LOCK_DEVICE, 0);
}

/* Unlock a single device block
 * pHandle      : Pointer to a LPCSPIFILIB device handle
 * block        : Block number to unlock
 * return       A SPIFI_NO_* error code (SPIFI_ERR_NONE is no errors)
 */

static INLINE SPIFI_ERR_T spifiDevUnlockBlock(const SPIFI_HANDLE_T *pHandle,
                                              uint32_t block)
{
  return pHandle->pFamFx->lockCmd(pHandle, SPIFI_PCMD_UNLOCK_BLOCK, block);
}

/* Lock a single device block
 * pHandle      : Pointer to a LPCSPIFILIB device handle
 * block        : Block number to lock
 * return       A SPIFI_NO_* error code (SPIFI_ERR_NONE is no errors)
 */

static INLINE SPIFI_ERR_T spifiDevLockBlock(const SPIFI_HANDLE_T *pHandle,
                                            uint32_t block)
{
  return pHandle->pFamFx->lockCmd(pHandle, SPIFI_PCMD_LOCK_BLOCK, block);
}

/* Full LPCSPIFILIB device erase
 * pHandle      : Pointer to a LPCSPIFILIB device handle
 * return       A SPIFI_NO_* error code (SPIFI_ERR_NONE is no errors)
 */

static INLINE SPIFI_ERR_T spifiDevEraseAll(const SPIFI_HANDLE_T *pHandle)
{
  return pHandle->pFamFx->eraseAll(pHandle);
}

/* Erase a sub-block
 * pHandle      : Pointer to a LPCSPIFILIB device handle
 * blknum       : Sub-block number to erase
 * return       A SPIFI_NO_* error code (SPIFI_ERR_NONE is no errors)
 */

static INLINE
SPIFI_ERR_T spifiDevEraseSubBlock(const SPIFI_HANDLE_T *pHandle,
                                  uint32_t blknum)
{
  return pHandle->pFamFx->eraseSubBlock(pHandle, blknum);
}

/* Program up to a page of data at an address
 * pHandle      : Pointer to a LPCSPIFILIB device handle
 * addr         : LPCSPIFILIB device address to start write at
 * writeBuff    : Address of buffer to write, must be 32-bit aligned
 * bytes        : Number of bytes to write, must not exceed page length
 * return       A SPIFI_NO_* error code (SPIFI_ERR_NONE is no errors)
 * NOTE	Only use this function to program data up to the page size.
 */

static INLINE SPIFI_ERR_T spifiDevPageProgram(const SPIFI_HANDLE_T *pHandle,
                                              uint32_t addr,
                                              uint32_t *writeBuff,
                                              uint32_t bytes)
{
  return pHandle->pFamFx->pageProgram(pHandle, addr, writeBuff, bytes);
}

/* Read data from a LPCSPIFILIB device
 * pHandle      : Pointer to a LPCSPIFILIB device handle
 * addr         : LPCSPIFILIB device address to read from
 * readBuff     : Address of buffer to fill, must be 32-bit aligned
 * bytes        : Number of bytes to read
 * return       A SPIFI_NO_* error code (SPIFI_ERR_NONE is no errors)
 * NOTE	Maximum read size is limited to the max single read size
 */

static INLINE SPIFI_ERR_T spifiDevRead(const SPIFI_HANDLE_T *pHandle,
                                       uint32_t addr,
                                       uint32_t *readBuff, uint32_t bytes)
{
  return pHandle->pFamFx->read(pHandle, addr, readBuff, bytes);
}

/* Reset the device
 * pHandle     : Pointer to a LPCSPIFILIB device handle
 * return       Nothing
 * NOTE	Will set the device into read mode
 */

static INLINE void spifiDevReset(const SPIFI_HANDLE_T *pHandle)
{
  pHandle->pFamFx->reset(pHandle);
}

/* Returns a string pointer to the generic device family name
 * pHandle      : Pointer to a LPCSPIFILIB device handle
 * return a string pointer to the generic device family name
 */

static INLINE
const char *spifiDevGetDeviceName(const SPIFI_HANDLE_T *pHandle)
{
  return pHandle->pInfoData->pDevName;
}

#define spifiDevGetFamilyName spifiDevGetDeviceName /* Deprecated!  Do NOT use for new development */

/* Returns information on the device
 * pHandle      : Pointer to a LPCSPIFILIB device handle
 * infoId       : Info to get about the device
 * return       Return value varies per selected function
 */

uint32_t spifiDevGetInfo(const SPIFI_HANDLE_T *pHandle,
                         SPIFI_INFO_ID_T infoId);

/* LPCSPIFILIB_HELPAPI LPCSPIFILIB library helper functions */

/* Returns the starting address of a block number
 * pHandle      : Pointer to a LPCSPIFILIB device handle
 * blockNum     : Block number to get starting address for
 * return       The starting address for the block,
 *               or 0xFFFFFFFF if the block number if invalid
 */

uint32_t spifiGetAddrFromBlock(const SPIFI_HANDLE_T *pHandle,
                               uint32_t blockNum);

/* Returns the starting address of a sub-block number
 * pHandle      : Pointer to a LPCSPIFILIB device handle
 * subBlockNum  : Sub-block number to get starting address for
 * return       The starting address for the sub-block,
 *              or 0xFFFFFFFF if the block number if invalid
 */

uint32_t spifiGetAddrFromSubBlock(const SPIFI_HANDLE_T *pHandle,
                                  uint32_t subBlockNum);

/* Returns the block number the passed address is located in
 * pHandle    : Pointer to a LPCSPIFILIB device handle
 * addr       : Address to get block number for
 * return    The block number the passed address is in,
 *            0xFFFFFFFF is the address is invalid
 */

uint32_t spifiGetBlockFromAddr(const SPIFI_HANDLE_T *pHandle,
                               uint32_t addr);

/* Returns the sub-block number the passed address is located in
 * pHandle : Pointer to a LPCSPIFILIB device handle
 * addr    : Address to get sub-block number for
 * return   The sub-block number the passed address is in,
 *          0xFFFFFFFF is the address is invalid
 */

uint32_t spifiGetSubBlockFromAddr(const SPIFI_HANDLE_T *pHandle,
                                  uint32_t addr);

/* Returns the first sub-block for a block
 * pHandle      : Pointer to a LPCSPIFILIB device handle
 * blockNum     : Block number to get first sub-block for
 * return The first sub-block number in passed block,
 *         0xFFFFFFFF if the block number if invalid
 */

uint32_t spifiGetSubBlockFromBlock(const SPIFI_HANDLE_T *pHandle,
                                   uint32_t blockNum);

/* Program the device with the passed buffer
 * pHandle      : Pointer to a LPCSPIFILIB device handle
 * addr         : LPCSPIFILIB device address to start write at
 * writeBuff    : Address of buffer to write, must be 32-bit aligned
 * bytes        : Number of bytes to write
 * return A SPIFI_ERR_xxx error code (SPIFI_ERR_NONE is no errors)
 * NOTE This function has no size limit. This function only works in blocking
 * mode.
 */

SPIFI_ERR_T spifiProgram(const SPIFI_HANDLE_T *pHandle, uint32_t addr,
                         const uint32_t *writeBuff, uint32_t bytes);

/* Read the device into the passed buffer
 * pHandle      : Pointer to a LPCSPIFILIB device handle
 * addr         : LPCSPIFILIB device address to start read at
 * readBuff     : Address of buffer to read into, must be 32-bit aligned
 * bytes        : Number of bytes to read
 * return       A SPIFI_ERR_xxx error code (SPIFI_ERR_NONE is no errors)
 * NOTE	This function has no size limit. Optionally, the device can be placed
 * into memory mode and accessed directly via memory mapped reads without
 * using this function. This function only works in blocking mode.
 */

SPIFI_ERR_T spifiRead(const SPIFI_HANDLE_T *pHandle, uint32_t addr,
                      uint32_t *readBuff, uint32_t bytes);

/* Erase multiple blocks
 * pHandle      : Pointer to a LPCSPIFILIB device handle
 * firstBlock   : First block number to erase
 * numBlocks    : Number of blocks to erase
 * return A SPIFI_ERR_xxx error code (SPIFI_ERR_NONE is no errors)
 * NOTE	If any of the specified params are invalid, the operation is aborted
 * before any sectors are erased. This function only works in blocking mode.
 */

SPIFI_ERR_T spifiErase(const SPIFI_HANDLE_T *pHandle,
                       uint32_t firstBlock, uint32_t numBlocks);

/* Erase multiple blocks by address range
 * pHandle      : Pointer to a LPCSPIFILIB device handle
 * firstAddr    : Starting address range for block erase
 * lastAddr     : Ending address range for block erase
 * return A SPIFI_ERR_xxx error code (SPIFI_ERR_NONE is no errors)
 * NOTE This function will erase blocks inside the passed address
 * range if and only if the address range is valid.
 * This function only works in blocking mode.
 */

SPIFI_ERR_T spifiEraseByAddr(const SPIFI_HANDLE_T *pHandle,
                             uint32_t firstAddr, uint32_t lastAddr);

#ifdef __cplusplus
}
#endif

#endif /* __ARCH_ARM_SRC_LPC43XX_SPIFI_INC_SPIFILIB_API_H */
