/****************************************************************************
 * arch/arm/src/lpc43xx/spifi/inc/spifilib_dev.h
 *
 * @note
 * Copyright(C) NXP Semiconductors, 2014
 * All rights reserved.
 *
 * @par
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * LPC products.  This software is supplied "AS IS" without any warranties
 * of any kind, and NXP Semiconductors and its licenser disclaim any and
 * all warranties, express or implied, including all implied warranties of
 * merchantability, fitness for a particular purpose and non-infringement of
 * intellectual property rights. NXP Semiconductors assumes no responsibility
 * or liability for the use of the software, conveys no license or rights
 * under any patent, copyright, mask work right, or any other intellectual
 * property rights in or to any products. NXP Semiconductors reserves the
 * right to make changes in the software without notification. NXP
 * Semiconductors also makes no representation or warranty that such
 * application will be suitable for the specified use without further testing
 * or modification.
 *
 * @par
 * Permission to use, copy, modify, and distribute this software and its
 * documentation is hereby granted, under NXP Semiconductors' and its
 * licensor's relevant copyrights in the software, without fee, provided that
 * it is used in conjunction with NXP Semiconductors microcontrollers.  This
 * copyright, permission, and disclaimer notice must appear in all copies of
 * this code.
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_LPC43XX_SPIFI_INC_SPIFILIB_DEV_H
#define __ARCH_ARM_SRC_LPC43XX_SPIFI_INC_SPIFILIB_DEV_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/* Define for inline functions */
#ifndef INLINE
#ifdef __CC_ARM
#define INLINE  __inline
#else
#define INLINE inline
#endif /* __CC_ARM */
#endif /* !INLINE */

/* LPCSPIFILIB_DEV LPCSPIFILIB device driver API functions */

/*  Possible error codes that can be returned from functions
 */

typedef enum
{
  SPIFI_ERR_NONE = 0,          /* No error */
  SPIFI_ERR_BUSY,              /* Device is busy */
  SPIFI_ERR_GEN,               /* General error */
  SPIFI_ERR_NOTSUPPORTED,      /* Capability not supported */
  SPIFI_ERR_ALIGNERR,          /* Attempted to do an operation on an unaligned section of the device */
  SPIFI_ERR_LOCKED,            /* Device was locked and a program/erase operation was attempted */
  SPIFI_ERR_PROGERR,           /* Error programming device (blocking mode only) */
  SPIFI_ERR_ERASEERR,          /* Erase error (blocking mode only) */
  SPIFI_ERR_NOTBLANK,          /* Program operation on block that is not blank */
  SPIFI_ERR_PAGESIZE,          /* PageProgram write size exceeds page size */
  SPIFI_ERR_VAL,               /* Program operation failed validation or readback compare */
  SPIFI_ERR_RANGE,             /* Range error, bad block number, address out of range, etc. */
  SPIFI_ERR_MEMMODE,           /* Library calls not allowed while in memory mode. */
  SPIFI_ERR_LASTINDEX          /* Internal use to count number of errors */
} SPIFI_ERR_T;

/*  Possible device capabilities returned from getInfo() */

#define SPIFI_CAP_DUAL_READ         (1 << 0)  /* Supports DUAL read mode */
#define SPIFI_CAP_DUAL_WRITE        (1 << 1)  /* Supports DUAL write mode */
#define SPIFI_CAP_QUAD_READ         (1 << 2)  /* Supports QUAD read mode */
#define SPIFI_CAP_QUAD_WRITE        (1 << 3)  /* Supports QUAD write mode */
#define SPIFI_CAP_FULLLOCK          (1 << 4)  /* Full device lock supported */
#define SPIFI_CAP_BLOCKLOCK         (1 << 5)  /* Individual block device lock supported */
#define SPIFI_CAP_SUBBLKERASE       (1 << 6)  /* Sub-block erase supported */
#define SPIFI_CAP_4BYTE_ADDR        (1 << 7)  /* Supports 4 Byte addressing */
#define SPIFI_CAP_NOBLOCK           (1 << 16) /* Non-blocking mode supported */

/*  Possible driver options, may not be supported by all drivers */

#define SPIFI_OPT_USE_DUAL      (3 << 0)  /* Enable DUAL read / write if option is supported */
#define SPIFI_OPT_USE_QUAD      (3 << 2)  /* Enable QUAD read / write if option is supported */
#define SPIFI_OPT_NOBLOCK       (1 << 16) /* Will not block on program and erase operations, poll device status manually */

/*  Possible device statuses returned from getInfo() */

#define SPIFI_STAT_BUSY     (1 << 0) /* Device is busy erasing or programming */
#define SPIFI_STAT_ISWP     (1 << 1) /* Device is write protected (software or hardware) */
#define SPIFI_STAT_FULLLOCK (1 << 2) /* Device is fully locked */
#define SPIFI_STAT_PARTLOCK (1 << 3) /* Device is partially locked (device specific) */
#define SPIFI_STAT_PROGERR  (1 << 4) /* Device status shows a program error (non-blocking mode only) */
#define SPIFI_STAT_ERASEERR (1 << 5) /* Device status shows a erase error (non-blocking mode only) */

/*  Possible info lookup requests */

typedef enum
{
  SPIFI_INFO_BASE_ADDRESS = 0,    /* Device physical memory address */
  SPIFI_INFO_DEVSIZE,             /* Device size in Bytes */
  SPIFI_INFO_ERASE_BLOCKS,        /* Number of erase blocks */
  SPIFI_INFO_ERASE_BLOCKSIZE,     /* Size of erase blocks */
  SPIFI_INFO_ERASE_SUBBLOCKS,     /* Number of erase sub-blocks */
  SPIFI_INFO_ERASE_SUBBLOCKSIZE,  /* Size of erase sub-blocks */
  SPIFI_INFO_PAGESIZE,            /* Size of a page, page write size limit */
  SPIFI_INFO_MAXREADSIZE,         /* Maximum read size, read size limit in bytes */
  SPIFI_INFO_MAXCLOCK,            /* Maximum device speed in Hz */
  SPIFI_INFO_MAX_READ_CLOCK,      /* Maximum device speed for read cmd in Hz */
  SPIFI_INFO_MAX_HSREAD_CLOCK,    /* Maximum device speed for quad / dual read cmd in Hz */
  SPIFI_INFO_MAX_PROG_CLOCK,      /* Maximum device speed for program cmd in Hz */
  SPIFI_INFO_MAX_HSPROG_CLOCK,    /* Maximum device speed for quad program cmd in Hz */
  SPIFI_INFO_CAPS,                /* Device capabilities, OR'ed SPIFI_CAP_* values */
  SPIFI_INFO_STATUS,              /* Or'ed SPIFI_STAT_xxx values. Any persistent hardware bits will be cleared  */
  SPIFI_INFO_STATUS_RETAIN,       /* Or'ed SPIFI_STAT_xxx values. Any persistent hardware bits will be retained */
  SPIFI_INFO_OPTIONS,             /* Device capabilities, Or'ed SPIFI_OPT_* values */
  SPIFI_INFO_LASTINDEX
} SPIFI_INFO_ID_T;

/* SPIFI_INFO_QUADREAD_CLOCK Deprecated!
 * Do NOT use for new development
 */

#define SPIFI_INFO_QUADREAD_CLOCK SPIFI_INFO_MAX_HSREAD_CLOCK

/* SPIFI_INFO_QUADPROG_CLOCK Deprecated!
 * Do NOT use for new development
 */

#define SPIFI_INFO_QUADPROG_CLOCK SPIFI_INFO_MAX_HSPROG_CLOCK

/*  Possible device specific lock / un-lock commands */

typedef enum
{
  SPIFI_PCMD_UNLOCK_DEVICE = 0,  /* unlock device */
  SPIFI_PCMD_LOCK_DEVICE,        /* lock device */
  SPIFI_PCMD_UNLOCK_BLOCK,       /* unlock specified block */
  SPIFI_PCMD_LOCK_BLOCK          /* lock specified block */
} SPIFI_PCMD_LOCK_UNLOCK_T;

/*  Possible device specific sub-block commands */

typedef enum
{
  SPIFI_PCMD_ADDR_TO_SUB_BLOCK = 0, /* Convert address to a sub-block */
  SPIFI_PCMD_SUB_BLOCK_TO_ADDR,     /* Convert sub-block to address */
  SPIFI_PCMD_BLOCK_TO_SUB_BLOCK     /* Convert block to sub-block */
} SPIFI_PCMD_SUBBLK_T;

/*  Enumeration of device specific functions. */

typedef enum
{
  FX_spifiDeviceDataInitDeinit = 0,       /* Generic device init / de-init function */
  FX_spifiDeviceDataInitDeinitS25FL164K,  /* S25FL164K specific device init / de-init function */
  FX_spifiDeviceDataClearStatusNone,      /* General do nothing I.e no status bits to clear */
  FX_spifiDeviceDataClearStatusS25FL032P, /* S25FL032P (and similar) clear status bits function */
  FX_spifiDeviceDataGetStatusS25FL032P,   /* S25FL032P (and similar) get status function */
  FX_spifiDeviceDataGetStatusS25FL164K,   /* S25FL164K (and similar) get status function */
  FX_spifiDeviceDataGetStatusMX25L3235E,  /* MX25L3235E (and similar) get status function */
  FX_spifiDeviceDataGetStatusW25Q80BV,    /* W25Q80BV (and similar) get status function */
  FX_spifiDeviceDataSetStatusS25FL032P,   /* S25FL032P (and similar) set status function */
  FX_spifiDeviceDataSetStatusS25FL164K,   /* S25FL164K (and similar) set status function */
  FX_spifiDeviceDataSetStatusMX25L3235E,  /* MX25L3235E (and similar) set sttus function */
  FX_spifiDeviceDataSetOptsQuadModeBit9,  /* Set bit 9 when enabling Quad mode */
  FX_spifiDeviceDataSetOptsQuadModeBit6,  /* Set bit 6 when enabling Quad mode */
  FX_spifiDeviceInitReadCommand,          /* General return cmdReg value for read */
  FX_spifiDevice4BInitReadCommand,        /* General return cmdReg value for read w/ 4Byte address */
  FX_spifiDeviceInitWriteCommand,         /* General return cmdReg value for write */
  FX_spifiDevice4BInitWriteCommand,       /* General return cmdReg value for write w/ 4Byte address */
  FX_spifiDeviceInitWriteCommandMacronix  /* Macronix return cmdReg value for write */
} SPIFI_DEVFX_T;

/* Forward type declaration */

struct SPIFI_HANDLE;

struct SPIFI_DEVICE_DATA;

struct SPIFI_FAM_DESC;

struct SPIFI_DEVICE_ID;

/*  LPCSPIFILIB family data. */

typedef struct SPIFI_FAM_NODE
{
  const struct SPIFI_FAM_DESC *pDesc;  /* Pointer to device descriptor */
  struct SPIFI_FAM_NODE *pNext;        /* Reserved list pointer */
} SPIFI_FAM_NODE_T;

/* LPCSPIFILIB family descriptor,
 * used to describe devices to non-device specific functions
 */

typedef struct SPIFI_FAM_DESC
{
  const char              *pFamName;          /* (required) Pointer to generic family name */
  struct SPIFI_DEV_NODE   *pDevList;          /* (required) Pointer to device list */
  uint32_t                prvContextSize;     /* Number of bytes needed for driver context allocation */
  uint32_t                *pDevCount;         /* (required) Pointer to device count */

  void (*pPrvDevGetID)(uint32_t baseAddr, struct SPIFI_DEVICE_ID *pID);                                 /* (NULL allowed) Pointer to method that queries deviceID */
  SPIFI_ERR_T (*pPrvDevSetup)(struct SPIFI_HANDLE *pHandle, uint32_t spifiCtrlAddr, uint32_t baseAddr); /* (required) Pointer to device specific device initialization */
} SPIFI_FAM_DESC_T;

/* Register device data node */

typedef struct SPIFI_DEV_NODE
{
  const struct SPIFI_DEVICE_DATA *pDevData;   /* (required) Pointer to device specific data */
  struct SPIFI_DEV_NODE *pNext;               /* Reserved */
} SPIFI_DEV_NODE_T;

typedef SPIFI_ERR_T (*deviceInitDeInitFx)(const struct SPIFI_HANDLE *, uint32_t); /* Fx* to handle init / de-init */
typedef void (*devClearStatusFx)(const struct SPIFI_HANDLE *);                    /* Fx* to clear status */
typedef uint32_t (*devGetStatusFx)(const struct SPIFI_HANDLE *);                  /* Fx* to get status */
typedef void (*devSetStatusFx)(const struct SPIFI_HANDLE *, uint32_t);            /* Fx* to set status */

typedef SPIFI_ERR_T (*devSetOptsFx)(const struct SPIFI_HANDLE *, uint32_t, uint32_t);          /* Fx* to set options */
typedef void (*devGetReadCmdFx)(const struct SPIFI_HANDLE *, uint8_t, uint32_t *, uint32_t *); /* Fx* to return read commandReg value */
typedef void (*devGetWriteCmdFx)(const struct SPIFI_HANDLE *, uint32_t *);                     /* Fx* to return write commandReg value */

/*  Device specific function pointers */

typedef struct SPIFI_FAM_FX
{
  /* Device init and de-initialization */

  SPIFI_ERR_T (*lockCmd)(const struct SPIFI_HANDLE *, SPIFI_PCMD_LOCK_UNLOCK_T, uint32_t);        /* (required) Lock / unlock handler */
  SPIFI_ERR_T (*eraseAll)(const struct SPIFI_HANDLE *);                                           /* (required) Full device erase */
  SPIFI_ERR_T (*eraseBlock)(const struct SPIFI_HANDLE *, uint32_t);                               /* (required) Erase a block by block number */
  SPIFI_ERR_T (*eraseSubBlock)(const struct SPIFI_HANDLE *, uint32_t);                            /* (required) Erase a sub-block by block number */
  SPIFI_ERR_T (*pageProgram)(const struct SPIFI_HANDLE *, uint32_t, const uint32_t *, uint32_t);  /* (required) Program up to a page of data at an address */
  SPIFI_ERR_T (*read)(const struct SPIFI_HANDLE *, uint32_t, uint32_t *, uint32_t);               /* (required) Read an address range */
  SPIFI_ERR_T (*reset)(const struct SPIFI_HANDLE *);                                              /* (required) Reset SPIFI device */

  /* Info query functions */

  uint32_t (*getStatus)(const struct SPIFI_HANDLE *, uint8_t);                         /* (required) Returns device status */
  uint32_t (*subBlockCmd)(const struct SPIFI_HANDLE *, SPIFI_PCMD_SUBBLK_T, uint32_t); /* (NULL allowed) Performs specified cmd */

  /* Device specific functions */

  deviceInitDeInitFx devInitDeInit;  /* run-time assigned Fx* device init de-init */
  devClearStatusFx devClearStatus;   /* run-time assigned Fx* to clear status */
  devGetStatusFx devGetStatus;       /* run-time assigned Fx* to get status */
  devSetStatusFx devSetStatus;       /* run-time assigned Fx* to set status */
  devSetOptsFx devSetOpts;           /* run-time assigned Fx* to set quad mode */
  devGetReadCmdFx devGetReadCmd;     /* run-time assigned Fx* to return read cmd */
  devGetWriteCmdFx devGetWriteCmd;   /* run-time assigned Fx* to return write cmd */
} SPIFI_FAM_FX_T;

/*  Device identification data */

typedef struct SPIFI_DEVICE_ID
{
  uint8_t mfgId[3];      /* JEDEC ID data */
  uint8_t extCount;      /* Number of extended bytes to check */
  uint8_t extId[8];      /* extended data */
} SPIFI_DEVICE_ID_T;

/*  Register device data. */

typedef struct SPIFI_DEVICE_DATA
{
  const char *pDevName;       /* (required) Device friendly name */
  SPIFI_DEVICE_ID_T id;       /* Device id structure */
  uint32_t caps;              /* capabilities supported */
  uint16_t blks;              /* # of blocks */
  uint32_t blkSize;           /* size of block */
  uint16_t subBlks;           /* # of sub-blocks */
  uint16_t subBlkSize;        /* size of sub-block */
  uint16_t pageSize;          /* page size */
  uint32_t maxReadSize;       /* max read allowed in one operation */
  uint8_t maxClkRate;         /* (in MHz) maximum clock rate (max common speed) */
  uint8_t maxReadRate;        /* (in MHz) max clock rate for read (driver may utilize fast read) */
  uint8_t maxHSReadRate;      /* (in MHz) max clock rate for quad / dual read */
  uint8_t maxProgramRate;     /* (in MHz) max clock rate for program */
  uint8_t maxHSProgramRate;   /* (in MHz) max clock rate for quad program */
  uint8_t initDeInitFxId;     /* init/DeInit fx_id */
  uint8_t clearStatusFxId;    /* clearStatus fx_id */
  uint8_t getStatusFxId;      /* getStatus fx_id */
  uint8_t setStatusFxId;      /* setStatus fx_id */
  uint8_t setOptionsFxId;     /* setOptions fx_id */
  uint8_t getReadCmdFxId;     /* getReadCommand fx_id */
  uint8_t getWriteCmdFxId;    /* getWriteCommand fx_id */
} SPIFI_DEVICE_DATA_T;

/*  LPCSPIFILIB device handle, used with all device and info functions */

typedef struct SPIFI_HANDLE
{
  const struct SPIFI_FAM_FX *pFamFx;            /* (required) Pointer to device specific functions */
  struct SPIFI_INFODATA   *pInfoData;           /* (required) Pointer to info data area */
  void                    *pDevContext;         /* (NULL allowed) Pointer to device context (used by device functions) */
} SPIFI_HANDLE_T;

/*  Common data applicable to all devices */

typedef struct SPIFI_INFODATA
{
  uint32_t        spifiCtrlAddr;               /* SPIFI controller base address */
  uint32_t        baseAddr;                    /* Physical base address for the device */
  uint32_t        numBlocks;                   /* Number of blocks on the device */
  uint32_t        blockSize;                   /* Size of blocks on the device */
  uint32_t        numSubBlocks;                /* Number of sub-blocks on the device */
  uint32_t        subBlockSize;                /* Size of sub-blocks on the device */
  uint32_t        pageSize;                    /* Size of a page, usually denotes maximum write size in bytes for a single write operation */
  uint32_t        maxReadSize;                 /* Maximum read size in bytes for a single read operation */
  const struct SPIFI_DEVICE_DATA *pDeviceData; /* (required) Pointer to device specific data */
  uint32_t        opts;                        /* Device options of values SPIFI_OPT_* */
  const char      *pDevName;                   /* (required) Pointer to device name */
  SPIFI_ERR_T     lastErr;                     /* Last error for the driver */
  const SPIFI_DEVICE_ID_T *pId;                /* (required) Device id structure (JEDEC ID etc) */
} SPIFI_INFODATA_T;

/*  Context for enumerating devices */

typedef struct SPIFI_DEV_ENUMERATOR
{
  SPIFI_FAM_NODE_T *pFamily;  /* pointer to family node  */
  SPIFI_DEV_NODE_T *pDevice;  /* pointer to device structure */
} SPIFI_DEV_ENUMERATOR_T;

/* LPCSPIFILIB_REGISTERHELPER LPCSPIFILIB family registration functions */

/* brief     Family registration function
 * return    A pointer to a persistent SPIFI_DEV_FAMILY_T initialized for
 *           family.
 * note      This function constructs and returns a non-volitile
 *           SPIFI_DEV_FAMILY_T
 * structure that contains family specific information needed to register
 *           family.
 * This function MUST NOT be called directly and should only be passed to
 * the registration function spifiRegisterFamily()
 */

SPIFI_FAM_NODE_T *spifi_REG_FAMILY_CommonCommandSet(void);

/* SPIFI_REG_FAMILY_Spansion_2Byte_PStatus Deprecated!
 * Do NOT use for new development
 */
#define SPIFI_REG_FAMILY_Spansion_2Byte_PStatus spifi_REG_FAMILY_CommonCommandSet

/* SPIFI_REG_FAMILY_Spansion_3Byte_Status Deprecated!
 * Do NOT use for new development
 */
#define SPIFI_REG_FAMILY_Spansion_3Byte_Status spifi_REG_FAMILY_CommonCommandSet

/* SPIFI_REG_FAMILY_Macronix_2Byte_Status Deprecated!
 * Do NOT use for new development
 */
#define SPIFI_REG_FAMILY_Macronix_2Byte_Status spifi_REG_FAMILY_CommonCommandSet

/* SPIFI_REG_FAMILY_SpansionS25FLP Deprecated!
 * Do NOT use for new development
 */
#define SPIFI_REG_FAMILY_SpansionS25FLP spifi_REG_FAMILY_CommonCommandSet

/* SPIFI_REG_FAMILY_SpansionS25FL1 Deprecated!
 * Do NOT use for new development
 */
#define SPIFI_REG_FAMILY_SpansionS25FL1 spifi_REG_FAMILY_CommonCommandSet

/* SPIFI_REG_FAMILY_MacronixMX25L Deprecated!
 * Do NOT use for new development
 */
#define SPIFI_REG_FAMILY_MacronixMX25L spifi_REG_FAMILY_CommonCommandSet

#ifdef __cplusplus
}
#endif

#endif /* __ARCH_ARM_SRC_LPC43XX_SPIFI_INC_SPIFILIB_DEV_H */
