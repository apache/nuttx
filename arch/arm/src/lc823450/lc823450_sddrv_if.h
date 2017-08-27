/****************************************************************************
 * arch/arm/src/lc823450/lc823450_sddrv_if.h
 *
 *   Copyright (C) 2014-2015 ON Semiconductor. All rights reserved.
 *   Copyright (C) 2014-2017 Sony Corporation. All rights reserved.
 *   Author: Masayuki Ishikawa <Masayuki.Ishikawa@jp.sony.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_LC823450_LC823450_SDDRV_IF_H
#define __ARCH_ARM_SRC_LC823450_LC823450_SDDRV_IF_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "lc823450_sddrv_type.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SDDR_VER                        (303)

#define SDDR_SUCCESS                    (  0)
#define SDDR_ERR_PARAM                  ( -1)
#define SDDR_ERR_STATE                  ( -2)
#define SDDR_ERR_CARD_NOT_EXIST         ( -3)
#define SDDR_ERR_WRITE_PROTECT          ( -4)
#define SDDR_ERR_R1_STATUS              ( -5)
#define SDDR_ERR_TIMEOUT                ( -6)
#define SDDR_ERR_COMMAND                ( -7)
#define SDDR_ERR_CARD_SIZE              ( -8)
#define SDDR_ERR_CMD_NO_RESPONSE        ( -9)
#define SDDR_ERR_SD_CARD_TYPE           (-10)
#define SDDR_ERR_ERASE_AREA             (-12)
#define SDDR_ERR_NOT_SUPPORT            (-14)
#define SDDR_ERR_SPEEDMODE_CHANGE       (-15)
#define SDDR_ERR_FUNC_IMPLEMENT         (-16)
#define SDDR_ERR_CARD_INFORMATION       (-17)
#define SDDR_ERR_BOOT_CONFIG            (-18)
#define SDDR_ERR_CACHE_CTRL             (-19)

#define SDDR_RW_NOINC_WORD              (0x00)
#define SDDR_RW_NOINC_HWORD             (0x10)
#define SDDR_RW_NOINC_BYTE              (0x20)
#define SDDR_RW_INC_WORD                (0x01)
#define SDDR_RW_INC_HWORD               (0x11)
#define SDDR_RW_INC_BYTE                (0x21)

#define SDDR_MEDIA_TYPE_SD              (0)
#define SDDR_MEDIA_TYPE_MMC             (1)

#define SDDR_CHECK_WP                   (1UL << 4)
#define SDDR_CHECK_CD                   (1UL << 5)
#define SDDR_SDXC_PC                    (1UL << 6)
#define SDDR_SD_SWITCH_18V              (1UL << 7)

#define SDDR_GEN_CMD_WR                 (0)
#define SDDR_GEN_CMD_RD                 (1)

#define SDDR_CACHE_OFF                  (0)
#define SDDR_CACHE_ON                   (1)
#define SDDR_CACHE_FLUSH                (2)

#define SDDR_PORT0                      (0UL)
#define SDDR_PORT1                      (1UL)
#define SDDR_PORT2                      (2UL)

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/* SD specific info */
typedef struct t_SdInfo
{
  UI_32 mScr[2];
  UI_32 mSdStatus[16];
} SdInfo;

/* MMC specific info */
typedef struct t_MmcInfo
{
  UI_32 mExtCsd_CACHE_SIZE;               /* ExtCsd [252:249] */
  UI_32 mExtCsd_SEC_COUNT;                /* ExtCsd [215:212] */

  UI_8  mExtCsd_SEC_FEATURE_SUPPORT;      /* ExtCsd [231] */
  UI_8  mExtCsd_BOOT_SIZE_MULT;           /* ExtCsd [226] */
  UI_8  mExtCsd_DEVICE_TYPE;              /* ExtCsd [196] */
  UI_8  mExtCsd_HS_TIMING;                /* ExtCsd [185] */
  UI_8  mExtCsd_PARTITION_CONFIG;         /* ExtCsd [179] */
  UI_8  mExtCsd_BOOT_CONFIG_PROT;         /* ExtCsd [178] */
  UI_8  mExtCsd_BOOT_BUS_WIDTH;           /* ExtCsd [177] */
  UI_8  mExtCsd_CACHE_CTRL;               /* ExtCsd [ 33] */
} MmcInfo;

/* SdDr configuration */
typedef struct t_SdDrCfg
{
  UI_32 mRegBase;         /* SD Host I/F register base address */
  UI_32 mSysClk;          /* System Clock */
  UI_32 mDetectTime;      /* Card detection time */
  UI_32 mSetting;         /* WP CD settings */
  void *mWorkBuf;         /* Work buffer (512 byte) */

  SINT_T (*mDepHwInit)(struct t_SdDrCfg *);
  SINT_T (*mDepHwExit)(struct t_SdDrCfg *);
  SINT_T (*mDepOsInit)(struct t_SdDrCfg *);
  SINT_T (*mDepOsExit)(struct t_SdDrCfg *);
  void   (*mDepSetClk)(struct t_SdDrCfg *);
  SINT_T (*mDepWait)(UI_32, struct t_SdDrCfg *);
  SINT_T (*mDepWaitStatus)(UI_32 req, UI_32 *status, struct t_SdDrCfg *cfg);
  SINT_T (*mDepReadData)(void *src, void *dst, UI_32 size, SINT_T type, struct t_SdDrCfg *cfg);
  SINT_T (*mDepWriteData)(void *src, void *dst, UI_32 size, SINT_T type, struct t_SdDrCfg *cfg);
  void   (*mDepVoltageSwitch)(struct t_SdDrCfg *);

  /* To here, external members to be set */
  /* From here, internal mermbers (no need to set) */

  UI_32 mInfo;            /* Misc info (e.g. driver state) */
  UI_32 mSecNum;          /* The number of sectors */
  UI_32 mLimitSdClk;      /* Max SD clock */
  UI_32 mClkDiv;          /* Clock divider */
  UI_32 mEraseTmOut;      /* Timeout for Erase/Trim */
  UI_32 mCid[4];          /* CID */
  UI_32 mCsd[4];          /* CSD */

  union {
    SdInfo  Sd;     /* SD specific info */
    MmcInfo Mmc;    /* MMC specific info */
  } mEx;

  UI_32 mReserved;
} SdDrCfg;


/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif


/****************************************************************************
 * Public Functions
 ****************************************************************************/

SINT_T SdDrRefVersion(void);
SINT_T SdDrInitialize(SdDrCfg *cfg);
SINT_T SdDrFinalize(SdDrCfg *cfg);
SINT_T SdDrIdentifyCard(SdDrCfg *cfg);
SINT_T SdDrCheckCardIdentify(SdDrCfg *cfg);
SINT_T SdDrCheckWriteEnable(SdDrCfg *cfg);
SINT_T SdDrCheckCardDetect(SdDrCfg *cfg);
SINT_T SdDrCheckCardRemoved(SdDrCfg *cfg);
SINT_T SdDrGetCardSize(UI_32 *secNum, UI_32 *secSize, SdDrCfg *cfg);
SINT_T SdDrGetCid(UI_32 *cid, SdDrCfg *cfg);
SINT_T SdDrGetCsd(UI_32 *csd, SdDrCfg *cfg);
SINT_T SdDrGetScr(UI_32 *scr, SdDrCfg *cfg);
SINT_T SdDrGetExtCsd(UI_32 *extCsd, SdDrCfg *cfg);
SINT_T SdDrClearCardInfo(SdDrCfg *cfg);
SINT_T SdDrReadSector(UI_32 addr, UI_32 cnt, void *buf, SINT_T type,
          SdDrCfg *cfg);
SINT_T SdDrWriteSector(UI_32 addr, UI_32 cnt, void *buf, SINT_T type,
           SdDrCfg *cfg);
SINT_T SdDrEraseSector(UI_32 addr, UI_32 cnt, SdDrCfg *cfg);
SINT_T SdDrSetClock(UI_32 limitClk, UI_32 sysClk, SdDrCfg *cfg);
SINT_T SdDrChangeSpeedMode(SINT_T mode, SdDrCfg *cfg);
SINT_T SdDrRefMediaType(SdDrCfg *cfg);
SINT_T SdDrSleep(SdDrCfg *cfg);
SINT_T SdDrAwake(SdDrCfg *cfg);
SINT_T SdDrSelectAccessPartition(UI_32 partNumber, SdDrCfg *cfg);
SINT_T SdDrConfigBootMode(SINT_T enable, SINT_T ack, UI_32 bootPartNumber,
        UI_32 bootBusWidth, SdDrCfg *cfg);
SINT_T SdDrGetPartitionSize(UI_32 partNumber, UI_32 *secNum, SdDrCfg *cfg);

SINT_T SdDrCheckSDXC(SdDrCfg *cfg);

SINT_T SdDrEraseSeq(UI_32 type, UI_32 addr, UI_32 cnt, SdDrCfg *cfg);

#define SdDrTrimSector(addr,cnt,cfg)   SdDrEraseSeq(0x00000001,addr,cnt,cfg)
#define SdDrSTrimSector1(addr,cnt,cfg) SdDrEraseSeq(0x80000001,addr,cnt,cfg)
#define SdDrSTrimSector2(cfg)          SdDrEraseSeq(0x80008000,   0,  1,cfg)

SINT_T SdDrGeneralCommand(SINT_T arg, UI_32 size, void *buf, SINT_T type,
                          SdDrCfg *cfg);
SINT_T SdDrCacheCtrl(SINT_T ctrl, SdDrCfg *cfg);



#define SDDR_SUPPORT_TRIM(cfg) \
  ((SdDrRefMediaType(cfg) == SDDR_MEDIA_TYPE_MMC) ? \
   (((cfg)->mEx.Mmc.mExtCsd_SEC_FEATURE_SUPPORT&(1UL << 4)) ? 1 : 0) : 0)

#define SDDR_SUPPORT_CACHE(cfg) \
  ((SdDrRefMediaType(cfg) == SDDR_MEDIA_TYPE_MMC) ? \
   ((cfg)->mEx.Mmc.mExtCsd_CACHE_SIZE ? 1:0) : 0)

SINT_T SdDrBkopsGetStatus(SdDrCfg *cfg);
SINT_T SdDrBkopsEnable(SINT_T ena, SdDrCfg *cfg);
SINT_T SdDrBkopsStart(SdDrCfg *cfg);
SINT_T SdDrHpiEnable(SINT_T ena, SdDrCfg *cfg);
SINT_T SdDrHpiExec(SINT_T check, SdDrCfg *cfg);

SINT_T fixedSdDrReadSector(UI_32 addr, UI_32 cnt, void *buf, SINT_T type, SdDrCfg *cfg);

UI_32 sdif_get_status(UI_32);
  
#if defined(__cplusplus)
}
#endif
#undef EXTERN

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_LC823450_LC823450_SDDRV_IF_H */
