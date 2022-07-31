/****************************************************************************
 * arch/arm/src/lpc43xx/spifi/src/spifilib_dev_common.c
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
 * licensor's relevant copyrights in the software, without fee, provided that
 * it is used in conjunction with NXP Semiconductors microcontrollers.  This
 * copyright, permission, and disclaimer notice must appear in all copies of
 * this code.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "spifi/inc/spifilib_api.h"
#include "spifi/inc/private/spifilib_chiphw.h"

/****************************************************************************
 * Private types
 ****************************************************************************/

/* Declare the version numbers */

#define LIBRARY_VERSION_MAJOR (1)
#define LIBRARY_VERSION_MINOR (03)

/* device node count and linked list header */

static uint32_t famCount = 0;
static SPIFI_FAM_NODE_T famListHead =
{
  0
};

/* Generic device OP Codes */

#define SPIFI_OP_CODE_RDID          0x9F

/* Number of supported devices */

#define NUMSUPPDEVS (sizeof(pPrvDevs) / sizeof(SPIFI_DEVDESC_T *))

/* Mapped error strings to error codes */

static const char *spifiErrStrings[SPIFI_ERR_LASTINDEX] =
{
  "No error",
  "Device is busy",
  "General error",
  "Capability not supported",
  "Alignment error",
  "Device is locked",
  "Program error",
  "Erase error",
  "Program region not blank",
  "Page size exceeded",
  "Validation error",
  "Range exceeded",
  "Not Allowed in Memory Mode"
};

static const char noName[] = "Invalid index";

/****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/****************************************************************************
 * Private functions
 ****************************************************************************/

static uint8_t spifiPrvCheckExtendedMatch(SPIFI_DEV_NODE_T *pNode,
                                          SPIFI_DEVICE_ID_T *pID)
{
  uint32_t x;

  if (pID->extCount != pNode->pDevData->id.extCount)
    {
      return 0;
    }

  if (pNode->pDevData->id.extCount)
    {
      for (x = 0; x < pID->extCount; ++x)
        {
          if (pNode->pDevData->id.extId[x] != pID->extId[x])
            {
              return 0;
            }
        }
    }

  return 1;
}

static SPIFI_DEV_NODE_T *spifiPrvFindDeviceMatch(SPIFI_DEV_NODE_T *pHead,
                                                 SPIFI_DEVICE_ID_T *pID,
                                                 uint8_t checkExtended)
{
  SPIFI_DEV_NODE_T *pNode;

  /* search the list looking for a match. Skip over head node since
   * it is a dummy node and NEVER contains data
   */

  for (pNode = pHead->pNext; pNode != NULL; pNode = pNode->pNext)
    {
      /* Manufacturer and part match? */

      if ((pID->mfgId[0] == pNode->pDevData->id.mfgId[0]) &&
          (pID->mfgId[1] == pNode->pDevData->id.mfgId[1]) &&
          (pID->mfgId[2] == pNode->pDevData->id.mfgId[2]))
        {
          /* If extended data check it */

          uint8_t matchFound = 1;
          if (checkExtended)
            {
              matchFound = spifiPrvCheckExtendedMatch(pNode, pID);
            }

          /* Match, time to exit */

          if (matchFound)
            {
              return pNode;
            }
        }
    }

  return NULL;
}

/* Read Identification */

static void spifiPrvDevGetID(uint32_t spifiAddr, SPIFI_DEVICE_ID_T *pID)
{
  uint8_t idx;
  LPC_SPIFI_CHIPHW_T *pSpifiCtrlAddr = (LPC_SPIFI_CHIPHW_T *) spifiAddr;

  /* Read ID command, plus read 3 bytes on data */

  spifi_HW_SetCmd(pSpifiCtrlAddr,
                 (SPIFI_CMD_OPCODE(SPIFI_OP_CODE_RDID) |
                  SPIFI_CMD_DATALEN(3 + pID->extCount) |
                  SPIFI_CMD_FIELDFORM(SPIFI_FIELDFORM_ALL_SERIAL) |
                  SPIFI_CMD_FRAMEFORM(SPIFI_FRAMEFORM_OP)));

  /* Get info from the device */

  pID->mfgId[0] = spifi_HW_GetData8(pSpifiCtrlAddr);  /* Manufacturers ID */
  pID->mfgId[1] = spifi_HW_GetData8(pSpifiCtrlAddr);  /* Memory Type */
  pID->mfgId[2] = spifi_HW_GetData8(pSpifiCtrlAddr);  /* Memory Capacity */

  /* Read the specified number of extended bytes */

  for (idx = 0; idx < pID->extCount; ++idx)
    {
      pID->extId[idx] = spifi_HW_GetData8(pSpifiCtrlAddr);
    }

  spifi_HW_WaitCMD(pSpifiCtrlAddr);
}

/* Detect if this device exists at the passed base address, returns 0 if the
 * device doesn't exist of the required memory allocation size for the device
 * context if the device exists.
 */

static SPIFI_DEV_NODE_T *spifiPrvDevDetect(uint32_t spifiCtrlAddr,
                                           SPIFI_FAM_NODE_T *familyNode)
{
  SPIFI_DEV_NODE_T *devNode;
  uint32_t idx;
  SPIFI_DEVICE_ID_T id;
  SPIFI_DEVICE_ID_T idVerify;
  void (*pPrvspifiPrvDevGetID)(uint32_t baseAddr,
                               SPIFI_DEVICE_ID_T *pID) = spifiPrvDevGetID;

  /* Do not ask for extended ID information yet */

  id.extCount = 0;
  idVerify.extCount = 0;

  /* If the family has a specific readID routine, use it instead */

  if (familyNode->pDesc->pPrvDevGetID)
    {
      pPrvspifiPrvDevGetID = familyNode->pDesc->pPrvDevGetID;
    }

  /* Read device ID three times to validate.
   * First read on a hard reset isn't reliable
   */

  pPrvspifiPrvDevGetID(spifiCtrlAddr, &id);
  pPrvspifiPrvDevGetID(spifiCtrlAddr, &id);
  pPrvspifiPrvDevGetID(spifiCtrlAddr, &idVerify);

  /* Compare both reads to make sure they match.
   * If any byte doesn't compare, abort.
   */

  for (idx = 0; idx < sizeof(id.mfgId); ++idx)
    {
      if (id.mfgId[idx] != idVerify.mfgId[idx])
        {
          return NULL;
        }
    }

  /* Find match for 3 bytes.
   *  If found, check to see if there is extended id information
   */

  devNode = spifiPrvFindDeviceMatch(familyNode->pDesc->pDevList, &id, 0);
  if ((devNode) && (devNode->pDevData->id.extCount))
    {
      /* read ID + extended ID data */

      id.extCount = devNode->pDevData->id.extCount;
      pPrvspifiPrvDevGetID(spifiCtrlAddr, &id);

      /* Now get the node that matches JEDEC and extended data */

      devNode = spifiPrvFindDeviceMatch(familyNode->pDesc->pDevList, &id, 1);
    }

  return devNode;
}

/* Detect first SPIFI FLASH device at the passed base address */

static SPIFI_FAM_NODE_T *spifiPrvPartDetect(uint32_t spifiCtrlAddr,
                                            SPIFI_DEV_NODE_T * *devData)
{
  SPIFI_FAM_NODE_T *pNode;

  /* Loop through the library and check for detected devices.
   * skip over head node because it is NEVER used.
   */

  for (pNode = famListHead.pNext; pNode != NULL; pNode = pNode->pNext)
    {
      /* Match at this index */

      if ((*devData = spifiPrvDevDetect(spifiCtrlAddr, pNode)) != NULL)
        {
          return pNode;
        }
    }

  return NULL;
}

static uint32_t spifiPrvCalculateHandleSize(SPIFI_FAM_NODE_T *devData)
{
  /* This is the size needed for the device context instance by the driver */

  return sizeof(SPIFI_HANDLE_T) +
         sizeof(SPIFI_INFODATA_T) +
         devData->pDesc->prvContextSize;
}

static void *spifiPrvMemset(void *bufPtr,
                            uint8_t value,
                            uint32_t count)
{
  uint8_t *dest = (uint8_t *) bufPtr;
  uint32_t index;

  for (index = 0; index < count; ++index)
    {
      dest[index] = value;
    }

  return bufPtr;
}

static void spifiPrvInitContext(SPIFI_DEV_ENUMERATOR_T *pContext,
                                SPIFI_FAM_NODE_T *pFamily)
{
  /* Save the new family passed */

  pContext->pFamily = pFamily;

  /* Save pointer to device or NULL if No devices */

  if (pFamily)
    {
      pContext->pDevice = pFamily->pDesc->pDevList->pNext;
    }
  else
    {
      pContext->pDevice = NULL;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

SPIFI_FAM_NODE_T *spifiRegisterFamily(SPIFI_FAM_NODE_T *(*regFx)(void))
{
  SPIFI_FAM_NODE_T *pFam;

  /* Get the family node from the user */

  pFam = regFx();

  /* If not a valid family return NULL and don't process */

  if (!pFam)
    {
      return NULL;
    }

  /* Insert the node into the beginning of the list */

  pFam->pNext = famListHead.pNext;
  famListHead.pNext = pFam;

  /* update the count of known families */

  ++famCount;

  /* Return handle */

  return pFam;
}

/* register a device (i.e append to the list of known devices) */

SPIFI_ERR_T spifiDevRegister(const SPIFI_FAM_NODE_T *pDevFamily,
                             SPIFI_DEV_NODE_T *pDevData)
{
  /* insert into the beginning of the list */

  pDevData->pNext = pDevFamily->pDesc->pDevList->pNext;
  pDevFamily->pDesc->pDevList->pNext = pDevData;

  /* update the number of devices in the list */

  (*pDevFamily->pDesc->pDevCount) += 1;

  /* Nothing to do here yet */

  return SPIFI_ERR_NONE;
}

/* enumerate the friendly names of supported devices */

const char *spifiDevEnumerateName(SPIFI_DEV_ENUMERATOR_T *pContext,
                                  uint8_t reset)
{
  const char *retValue = NULL;

  /* If user requested reset, point back to the beginning of the list */

  if (reset)
    {
      /* Initialize the device list from new family */

      spifiPrvInitContext(pContext, famListHead.pNext);
    }

  /* Now get the friendly name of the current device and increment to the
   * next device.
   */

  if (pContext->pDevice)
    {
      /* Retrieve friendly name */

      retValue = pContext->pDevice->pDevData->pDevName;

      /* Point at next device */

      pContext->pDevice = pContext->pDevice->pNext;

      /* Point at next family if at end of device list */

      if (!pContext->pDevice)
        {
          /* Initialize the device list from new family */

          spifiPrvInitContext(pContext, pContext->pFamily->pNext);
        }
    }

  return retValue;
}

/* Report the library version number */

uint16_t spifiGetLibVersion(void)
{
  return (LIBRARY_VERSION_MAJOR << 8) | LIBRARY_VERSION_MINOR;
}

/* Initialize the SPIFILIB driver */

SPIFI_ERR_T spifiInit(uint32_t spifiCtrlAddr, uint8_t reset)
{
  LPC_SPIFI_CHIPHW_T *pSpifiCtrlAddr = (LPC_SPIFI_CHIPHW_T *) spifiCtrlAddr;

  if (reset)
    {
      /* Reset controller */

      spifi_HW_ResetController(pSpifiCtrlAddr);

      /* Set intermediate data and memcmd registers. */

      spifi_HW_SetIDATA(pSpifiCtrlAddr, 0x0);
      spifi_HW_SetMEMCMD(pSpifiCtrlAddr, 0);

      spifi_HW_ResetController(pSpifiCtrlAddr);

      /* Setup SPIFI controller */

      spifi_HW_SetCtrl(pSpifiCtrlAddr,
                      (SPIFI_CTRL_TO(1000) |
                       SPIFI_CTRL_CSHI(15) |
                       SPIFI_CTRL_RFCLK(1) |
                       SPIFI_CTRL_FBCLK(1)));
    }

  /* Nothing to do here yet */

  return SPIFI_ERR_NONE;
}

/* performs device specific initialization */

SPIFI_ERR_T spifiDevInit(const SPIFI_HANDLE_T *pHandle)
{
  SPIFI_ERR_T retValue = SPIFI_ERR_NONE;

  /* call device specific initialization if provided */

  pHandle->pFamFx->devInitDeInit(pHandle, 1);

  /* make sure the controller is not in memMode */

  spifiDevSetMemMode(pHandle, 0);

  return retValue;
}

/* performs device specific de-initialization */

SPIFI_ERR_T spifiDevDeInit(const SPIFI_HANDLE_T *pHandle)
{
  SPIFI_ERR_T retValue = SPIFI_ERR_NONE;

  /* call device specific de-init if provided */

  pHandle->pFamFx->devInitDeInit(pHandle, 0);

  /* make sure the controller is in memMode */

  spifiDevSetMemMode(pHandle, 1);

  return retValue;
}

/* Converts a SPIFILIB error code into a meaningful string */

const char *spifiReturnErrString(SPIFI_ERR_T errCode)
{
  if (((unsigned int) errCode) < SPIFI_ERR_LASTINDEX)
    {
      return spifiErrStrings[errCode];
    }

  return noName;
}

/* Returns information on the device */

uint32_t spifiDevGetInfo(const SPIFI_HANDLE_T *pHandle,
                         SPIFI_INFO_ID_T infoId)
{
  uint32_t val = 0;

  /* Don't use switch statement to prevent including clib helpers */

  if (infoId == SPIFI_INFO_BASE_ADDRESS)
    {
      val = pHandle->pInfoData->baseAddr;
    }
  else if (infoId == SPIFI_INFO_DEVSIZE)
    {
      val = pHandle->pInfoData->numBlocks * pHandle->pInfoData->blockSize;
    }
  else if (infoId == SPIFI_INFO_ERASE_BLOCKS)
    {
      val = pHandle->pInfoData->numBlocks;
    }
  else if (infoId == SPIFI_INFO_ERASE_BLOCKSIZE)
    {
      val = pHandle->pInfoData->blockSize;
    }
  else if (infoId == SPIFI_INFO_ERASE_SUBBLOCKS)
    {
      val = pHandle->pInfoData->numSubBlocks;
    }
  else if (infoId == SPIFI_INFO_ERASE_SUBBLOCKSIZE)
    {
      val = pHandle->pInfoData->subBlockSize;
    }
  else if (infoId == SPIFI_INFO_PAGESIZE)
    {
      val = pHandle->pInfoData->pageSize;
    }
  else if (infoId == SPIFI_INFO_MAXREADSIZE)
    {
      val = pHandle->pInfoData->maxReadSize;
    }
  else if (infoId == SPIFI_INFO_MAXCLOCK)
    {
      val = (pHandle->pInfoData->pDeviceData->maxClkRate * 1000000);
    }
  else if (infoId == SPIFI_INFO_MAX_READ_CLOCK)
    {
      val = (pHandle->pInfoData->pDeviceData->maxReadRate * 1000000);
    }
  else if (infoId == SPIFI_INFO_MAX_HSREAD_CLOCK)
    {
      val = (pHandle->pInfoData->pDeviceData->maxHSReadRate * 1000000);
    }
  else if (infoId == SPIFI_INFO_MAX_PROG_CLOCK)
    {
        val = (pHandle->pInfoData->pDeviceData->maxProgramRate * 1000000);
    }
  else if (infoId == SPIFI_INFO_MAX_HSPROG_CLOCK)
    {
      val = (pHandle->pInfoData->pDeviceData->maxHSProgramRate * 1000000);
    }
  else if (infoId == SPIFI_INFO_CAPS)
    {
      val = pHandle->pInfoData->pDeviceData->caps;
    }
  else if (infoId == SPIFI_INFO_STATUS)
    {
        val = pHandle->pFamFx->getStatus(pHandle, 1);
    }
  else if (infoId == SPIFI_INFO_STATUS_RETAIN)
    {
      val = pHandle->pFamFx->getStatus(pHandle, 0);
    }
  else if (infoId == SPIFI_INFO_OPTIONS)
    {
      val = pHandle->pInfoData->opts;
    }

  return val;
}

/* Returns status of memory mode */

uint8_t spifiDevGetMemoryMode(const SPIFI_HANDLE_T *pHandle)
{
  LPC_SPIFI_CHIPHW_T *pSpifiCtrlAddr =
                   (LPC_SPIFI_CHIPHW_T *)pHandle->pInfoData->spifiCtrlAddr;

  return (spifi_HW_GetStat(pSpifiCtrlAddr) & SPIFI_STAT_MCINIT) != 0;
}

SPIFI_ERR_T spifiDevSetMemMode(const SPIFI_HANDLE_T *pHandle,
                               uint8_t enMMode)
{
  uint32_t cmdValue;
  uint32_t iDataValue;
  uint32_t ctrlReg;
  LPC_SPIFI_CHIPHW_T *pSpifiCtrlAddr =
                 (LPC_SPIFI_CHIPHW_T *) pHandle->pInfoData->spifiCtrlAddr;

  /* RESET the memMode controller */

  spifi_HW_ResetController(pSpifiCtrlAddr);

  /* Wait for HW to acknowledge the reset. */

  spifi_HW_WaitRESET(pSpifiCtrlAddr);

  /* First off set the HW mode based on current option */

  ctrlReg = spifi_HW_GetCtrl(pSpifiCtrlAddr);
  if (pHandle->pInfoData->opts & SPIFI_CAP_QUAD_READ)
    {
      ctrlReg &= ~(SPIFI_CTRL_DUAL(1));
    }
  else if (pHandle->pInfoData->opts & SPIFI_CAP_DUAL_READ)
    {
      ctrlReg |= SPIFI_CTRL_DUAL(1);
    }

  spifi_HW_SetCtrl(pSpifiCtrlAddr, ctrlReg);

  if (enMMode)
    {
      /* Get the device specific memory mode command and iData values */

      pHandle->pFamFx->devGetReadCmd(pHandle, enMMode,
                                     &cmdValue, &iDataValue);

      /* Specify the intermediate data byte. */

      spifi_HW_SetIDATA(pSpifiCtrlAddr, iDataValue);

      /* Set the appropriate values in the command reg. */

      spifi_HW_SetCmd(pSpifiCtrlAddr, cmdValue);
      spifi_HW_WaitCMD(pSpifiCtrlAddr);
      spifi_HW_SetMEMCMD(pSpifiCtrlAddr, cmdValue);
    }
  else
    {
      spifi_HW_SetIDATA(pSpifiCtrlAddr, 0xff);
      spifi_HW_SetMEMCMD(pSpifiCtrlAddr, 0);

      /* RESET the memMode controller */

      spifi_HW_ResetController(pSpifiCtrlAddr);

      /* Wait for HW to acknowledge the reset. */

      spifi_HW_WaitRESET(pSpifiCtrlAddr);
    }

  return SPIFI_ERR_NONE;
}

/* Return the number of supported device families in this driver */

uint32_t spifiGetSuppFamilyCount(void)
{
  /* return number of registered devices */

  return famCount;
}

/* Return the driver family name for a specific index */

const char *spifiGetSuppFamilyName(uint32_t index)
{
  uint32_t idx;
  SPIFI_FAM_NODE_T *pNode;

  if (index >= famCount)
    {
      return noName;
    }

  /* cycle through the list of families skipping over head node since it
   * is NEVER used. Once we break out of this loop pNode should be
   * pointing at the correct node.
   */

  pNode = famListHead.pNext;
  for (idx = 0; idx < index; ++idx)
    {
      pNode = pNode->pNext;
    }

  return pNode->pDesc->pFamName;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* Detect and return memory needed for device handle at passed address */

uint32_t spifiGetHandleMemSize(uint32_t spifiCtrlAddr)
{
  uint32_t bytesNeeded = 0;
  SPIFI_FAM_NODE_T *detectedPart;
  SPIFI_DEV_NODE_T *devData;

  /* Find first device at the base address */

  detectedPart = spifiPrvPartDetect(spifiCtrlAddr, &devData);
  if (detectedPart)
    {
      /* This is the size needed for the device context instance by
       * the driver
       */

      bytesNeeded = spifiPrvCalculateHandleSize(detectedPart);
    }

  return bytesNeeded;
}

/* Initialize driver and hardware for a specific device */

SPIFI_HANDLE_T *spifiInitDevice(void *pMem, uint32_t sizePMem,
                                uint32_t spifiCtrlAddr, uint32_t baseAddr)
{
  SPIFI_FAM_NODE_T *detectedPart;
  SPIFI_DEV_NODE_T *devData;
  SPIFI_HANDLE_T *pSpifiHandle;
  uint32_t *pMem32 = (uint32_t *) pMem;

  /* Is the passed buffer size aligned on a 32-bit boundary? */

  if (((uint32_t) pMem32 & 0x3) != 0)
    {
      return NULL;
    }

  /* Detect the device at at the base address and abort on error. */

  detectedPart = spifiPrvPartDetect(spifiCtrlAddr, &devData);
  if (!detectedPart)
    {
      return NULL;
    }

  /* Is passed memory space big enough? */

  if (spifiPrvCalculateHandleSize(detectedPart) > sizePMem)
    {
      return NULL;
    }

  /* Setup handle */

  pSpifiHandle = (SPIFI_HANDLE_T *) pMem;

  /* Clear entire device context areas */

  spifiPrvMemset(pMem, 0, sizePMem);

  /* Setup device info region */

  pMem32 += (sizeof(SPIFI_HANDLE_T) / sizeof(uint32_t));
  pSpifiHandle->pInfoData = (SPIFI_INFODATA_T *) pMem32;

  /* Save ptr to the detected device specific data into the handle */

  pSpifiHandle->pInfoData->pId = &devData->pDevData->id;

  /* Setup device private data region */

  pMem32 += (sizeof(SPIFI_INFODATA_T) / sizeof(uint32_t));
  pSpifiHandle->pDevContext = (void *) pMem32;

  /* Setup device specific data */

  pSpifiHandle->pInfoData->spifiCtrlAddr = spifiCtrlAddr;
  pSpifiHandle->pInfoData->baseAddr = baseAddr;
  pSpifiHandle->pInfoData->numBlocks = devData->pDevData->blks;
  pSpifiHandle->pInfoData->blockSize = devData->pDevData->blkSize;
  pSpifiHandle->pInfoData->numSubBlocks = devData->pDevData->subBlks;
  pSpifiHandle->pInfoData->subBlockSize = devData->pDevData->subBlkSize;
  pSpifiHandle->pInfoData->pageSize = devData->pDevData->pageSize;
  pSpifiHandle->pInfoData->maxReadSize = devData->pDevData->maxReadSize;
  pSpifiHandle->pInfoData->pDeviceData = devData->pDevData;
  pSpifiHandle->pInfoData->pDevName = devData->pDevData->pDevName;

  /* Call device setup */

  pSpifiHandle->pInfoData->lastErr =
                           detectedPart->pDesc->pPrvDevSetup(pSpifiHandle,
                                                             spifiCtrlAddr,
                                                             baseAddr);

  if (pSpifiHandle->pInfoData->lastErr != SPIFI_ERR_NONE)
    {
      return NULL;
    }

  /* Call the device specific init */

  pSpifiHandle->pInfoData->lastErr = spifiDevInit(pSpifiHandle);
  if (pSpifiHandle->pInfoData->lastErr != SPIFI_ERR_NONE)
    {
      return NULL;
    }

  return pSpifiHandle;
}

SPIFI_ERR_T spifiDevSetOpts(SPIFI_HANDLE_T *pHandle,
                            uint32_t options, uint8_t set)
{
  /* default to not supported */

  SPIFI_ERR_T retValue = SPIFI_ERR_NOTSUPPORTED;

  /* If changing any of the high speed modes process separately */

  if (options & (SPIFI_CAP_DUAL_READ | SPIFI_CAP_DUAL_WRITE |
      SPIFI_CAP_QUAD_READ | SPIFI_CAP_QUAD_WRITE))
    {
      uint32_t hsOptions;
      uint8_t memMode;

      /* first get the current memory mode */

      memMode = spifiDevGetMemoryMode(pHandle);

      /* First clear ALL high speed mode options */

      pHandle->pInfoData->opts &=
                         ~(SPIFI_CAP_DUAL_READ | SPIFI_CAP_DUAL_WRITE |
                           SPIFI_CAP_QUAD_READ | SPIFI_CAP_QUAD_WRITE);

      /* sanitize the change list */

      hsOptions = options &
                  (pHandle->pInfoData->pDeviceData->caps &
                  (SPIFI_CAP_DUAL_READ | SPIFI_CAP_DUAL_WRITE |
                   SPIFI_CAP_QUAD_READ | SPIFI_CAP_QUAD_WRITE));

      /* Now set the high speed options */

      if (set)
        {
          pHandle->pInfoData->opts |= hsOptions;
        }

      /* Perform device specific setup for the option */

      retValue = pHandle->pFamFx->devSetOpts(pHandle, hsOptions, set);

      /* remove so that it won't be interpreted as an error */

      options &= ~(SPIFI_CAP_DUAL_READ | SPIFI_CAP_DUAL_WRITE |
                   SPIFI_CAP_QUAD_READ | SPIFI_CAP_QUAD_WRITE);

      /* update memory mode when changing the high speed options */

      spifiDevSetMemMode(pHandle, memMode);
    }

  /* If the remaining options are valid, process them */

  if ((options &  pHandle->pInfoData->pDeviceData->caps) == options)
    {
      retValue = SPIFI_ERR_NONE;

      /* Set the option in the driver so other routines will act
       * accordingly
       */

      if (set)
        {
          pHandle->pInfoData->opts |= options;
        }
      else
        {
          pHandle->pInfoData->opts &= ~options;
        }

      /* Perform device specific setup for the option if defined */

      if (pHandle->pFamFx->devSetOpts)
        {
          retValue = pHandle->pFamFx->devSetOpts(pHandle, options, set);
        }
    }

  return retValue;
}

/* Returns the address mapped to an block number */

uint32_t spifiGetAddrFromBlock(const SPIFI_HANDLE_T *pHandle,
                               uint32_t blockNum)
{
  uint32_t baseAddr = 0xffffffff;

  if (blockNum < pHandle->pInfoData->numBlocks)
    {
      baseAddr = pHandle->pInfoData->baseAddr +
                (blockNum * pHandle->pInfoData->blockSize);
    }

  return baseAddr;
}

/* Returns the starting address of a sub-block number */

uint32_t spifiGetAddrFromSubBlock(const SPIFI_HANDLE_T *pHandle,
                                  uint32_t subBlockNum)
{
  uint32_t baseAddr = ~0UL;

  /* If the device provides a specific method for calculating the address
   * use it.
   */

  if (!pHandle->pFamFx->subBlockCmd)
    {
      /* If sub-blocks are not supported
       * (.e numSubBlocks = 0) then return error
       */

      if (subBlockNum < pHandle->pInfoData->numSubBlocks)
        {
          baseAddr = pHandle->pInfoData->baseAddr +
                     (subBlockNum * pHandle->pInfoData->subBlockSize);
        }
    }
  else
    {
      baseAddr = pHandle->pFamFx->subBlockCmd(pHandle,
                                              SPIFI_PCMD_SUB_BLOCK_TO_ADDR,
                                              subBlockNum);
    }

  return baseAddr;
}

/* Returns the block number the passedd= address is located in */

uint32_t spifiGetBlockFromAddr(const SPIFI_HANDLE_T *pHandle,
                               uint32_t addr)
{
  uint32_t block;
  block = (addr - pHandle->pInfoData->baseAddr) /
           pHandle->pInfoData->blockSize;

  if (block >= pHandle->pInfoData->numBlocks)
    {
      return ~0UL;
    }

  return block;
}

/* Returns the sub-block number the passed address is located in */

uint32_t spifiGetSubBlockFromAddr(const SPIFI_HANDLE_T *pHandle,
                                  uint32_t addr)
{
  uint32_t subBlock;

  /* If device does not support sub-blocks return error */

  if (!pHandle->pInfoData->subBlockSize)
    {
      return ~0ul;
    }

  if (!pHandle->pFamFx->subBlockCmd)
    {
      subBlock = (addr - pHandle->pInfoData->baseAddr) /
      pHandle->pInfoData->subBlockSize;

      if (subBlock >= pHandle->pInfoData->numSubBlocks)
        {
          return ~0ul;
        }
    }
  else
    {
      subBlock = pHandle->pFamFx->subBlockCmd(pHandle,
                                              SPIFI_PCMD_ADDR_TO_SUB_BLOCK,
                                              addr);
    }

  return subBlock;
}

/* Returns the first sub-block in the passed block */

uint32_t spifiGetSubBlockFromBlock(const SPIFI_HANDLE_T *pHandle,
                                   uint32_t blockNum)
{
  uint32_t subBlock = ~0UL;

  if (!pHandle->pFamFx->subBlockCmd)
    {
      /* If the blockNum passed is larger than this device,
       * or if sub-blocks are not supported report error
       */

      if ((blockNum >= pHandle->pInfoData->numBlocks) ||
         (!pHandle->pInfoData->subBlockSize))
        {
          return subBlock;
        }

      /* Calculate the sub-block number based on detected params */

      subBlock = (blockNum * (pHandle->pInfoData->blockSize /
                  pHandle->pInfoData->subBlockSize));
    }
  else
    {
      subBlock = pHandle->pFamFx->subBlockCmd(pHandle,
                                              SPIFI_PCMD_BLOCK_TO_SUB_BLOCK,
                                              blockNum);
    }

  return subBlock;
}

/* Program the device with the passed buffer */

SPIFI_ERR_T spifiProgram(const SPIFI_HANDLE_T *pHandle,
                         uint32_t addr,
                         const uint32_t *writeBuff,
                         uint32_t bytes)
{
  uint32_t sendBytes;
  SPIFI_ERR_T errcode = SPIFI_ERR_NONE;

  /* Program using up to page size */

  while ((bytes > 0) && (errcode == SPIFI_ERR_NONE))
    {
      sendBytes = bytes;
      if (sendBytes > pHandle->pInfoData->pageSize)
        {
          sendBytes = pHandle->pInfoData->pageSize;
        }

      errcode = pHandle->pFamFx->pageProgram(pHandle,
                                             addr,
                                             writeBuff, sendBytes);
      addr += sendBytes;
      writeBuff += (sendBytes >> 2);
      bytes -= sendBytes;
    }

  return errcode;
}

/* Read the device into the passed buffer */

SPIFI_ERR_T spifiRead(const SPIFI_HANDLE_T *pHandle,
                      uint32_t addr,
                       uint32_t *readBuff, uint32_t bytes)
{
  uint32_t readBytes;
  SPIFI_ERR_T errcode = SPIFI_ERR_NONE;

  /* Read using up to the maximum read size */

  while ((bytes > 0) && (errcode == SPIFI_ERR_NONE))
    {
      readBytes = bytes;
      if (readBytes > pHandle->pInfoData->maxReadSize)
        {
          readBytes = pHandle->pInfoData->maxReadSize;
        }

      errcode = pHandle->pFamFx->read(pHandle, addr, readBuff, readBytes);
      addr += readBytes;
      readBuff += (readBytes / sizeof(uint32_t));
      bytes -= readBytes;
    }

  return errcode;
}

/* Erase multiple blocks */

SPIFI_ERR_T spifiErase(const SPIFI_HANDLE_T *pHandle,
                       uint32_t firstBlock,
                       uint32_t numBlocks)
{
  SPIFI_ERR_T errcode = SPIFI_ERR_NONE;

  if ((firstBlock + numBlocks) > pHandle->pInfoData->numBlocks)
    {
      return SPIFI_ERR_RANGE;
    }

  /* Only perform erase if numBlocks is != 0 */

  for (; (numBlocks); ++firstBlock, --numBlocks)
    {
      errcode = pHandle->pFamFx->eraseBlock(pHandle, firstBlock);
      if (errcode != SPIFI_ERR_NONE)
        {
          break;
        }
    }

  return errcode;
}

/* Erase multiple blocks by address range */

SPIFI_ERR_T spifiEraseByAddr(const SPIFI_HANDLE_T *pHandle,
                             uint32_t firstAddr, uint32_t lastAddr)
{
  uint32_t firstBlock;
  uint32_t lastBlock;
  SPIFI_ERR_T errcode = SPIFI_ERR_RANGE;

  /* Get block numbers for addresses */

  firstBlock = spifiGetBlockFromAddr(pHandle, firstAddr);
  lastBlock = spifiGetBlockFromAddr(pHandle, lastAddr);

  /* Limit to legal address range */

  if ((firstBlock != ~0UL) && (lastBlock != ~0UL))
    {
      errcode = spifiErase(pHandle,
                           firstBlock,
                           ((lastBlock - firstBlock) + 1));
    }

  return errcode;
}

