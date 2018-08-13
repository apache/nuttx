/****************************************************************************
 * arch/arm/src/lc823450/lc823450_sddrv_if.h
 *
 *   Copyright (C) 2014-2015 ON Semiconductor. All rights reserved.
 *   Copyright 2014,2015,2017 Sony Video & Sound Products Inc.
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

struct SdInfo_s
{
  UI_32 scr[2];
  UI_32 sdstatus[16];
};

/* MMC specific info */

struct MmcInfo_s
{
  UI_32 extcsd_cache_size;               /* ExtCsd [252:249] */
  UI_32 extcsd_sec_count;                /* ExtCsd [215:212] */

  UI_8  extcsd_sec_feature_support;      /* ExtCsd [231] */
  UI_8  extcsd_boot_size_mult;           /* ExtCsd [226] */
  UI_8  extcsd_device_type;              /* ExtCsd [196] */
  UI_8  extcsd_hs_timing;                /* ExtCsd [185] */
  UI_8  extcsd_partition_config;         /* ExtCsd [179] */
  UI_8  extcsd_boot_config_prot;         /* ExtCsd [178] */
  UI_8  extcsd_boot_bus_width;           /* ExtCsd [177] */
  UI_8  extcsd_cache_ctrl;               /* ExtCsd [ 33] */
};

/* SdDr configuration */

struct SdDrCfg_s
{
  UI_32 regbase;         /* SD Host I/F register base address */
  UI_32 sysclk;          /* System Clock */
  UI_32 detecttime;      /* Card detection time */
  UI_32 setting;         /* WP CD settings */
  void  *workbuf;        /* Work buffer (512 byte) */

  SINT_T (*dephwinit)(struct SdDrCfg_s *);
  SINT_T (*dephwexit)(struct SdDrCfg_s *);
  SINT_T (*deposinit)(struct SdDrCfg_s *);
  SINT_T (*deposexit)(struct SdDrCfg_s *);
  void   (*depsetclk)(struct SdDrCfg_s *);
  SINT_T (*depwait)(UI_32, struct SdDrCfg_s *);
  SINT_T (*depwaitstatus)(UI_32 req, UI_32 *status, struct SdDrCfg_s *cfg);
  SINT_T (*depreaddata)(void *src, void *dst, UI_32 size, SINT_T type,
                        struct SdDrCfg_s *cfg);
  SINT_T (*depwritedata)(void *src, void *dst, UI_32 size, SINT_T type,
                         struct SdDrCfg_s *cfg);
  void   (*depvoltageswitch)(struct SdDrCfg_s *);

  /* To here, external members to be set */
  /* From here, internal mermbers (no need to set) */

  UI_32 info;            /* Misc info (e.g. driver state) */
  UI_32 secnum;          /* The number of sectors */
  UI_32 limitsdclk;      /* Max SD clock */
  UI_32 clkdiv;          /* Clock divider */
  UI_32 erasetmout;      /* Timeout for Erase/Trim */
  UI_32 cid[4];          /* CID */
  UI_32 csd[4];          /* CSD */

  union
  {
    struct SdInfo_s  sd;     /* SD specific info */
    struct MmcInfo_s mmc;    /* MMC specific info */
  } ex;

  UI_32 reserved;
};


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
SINT_T SdDrInitialize(struct SdDrCfg_s *cfg);
SINT_T SdDrFinalize(struct SdDrCfg_s *cfg);
SINT_T SdDrIdentifyCard(struct SdDrCfg_s *cfg);
SINT_T SdDrCheckCardIdentify(struct SdDrCfg_s *cfg);
SINT_T SdDrCheckWriteEnable(struct SdDrCfg_s *cfg);
SINT_T SdDrCheckCardDetect(struct SdDrCfg_s *cfg);
SINT_T SdDrCheckCardRemoved(struct SdDrCfg_s *cfg);
SINT_T SdDrGetCardSize(UI_32 *secnum, UI_32 *secsize, struct SdDrCfg_s *cfg);
SINT_T SdDrGetCid(UI_32 *cid, struct SdDrCfg_s *cfg);
SINT_T SdDrGetCsd(UI_32 *csd, struct SdDrCfg_s *cfg);
SINT_T SdDrGetScr(UI_32 *scr, struct SdDrCfg_s *cfg);
SINT_T SdDrGetExtCsd(UI_32 *extcsd, struct SdDrCfg_s *cfg);
SINT_T SdDrClearCardInfo(struct SdDrCfg_s *cfg);
SINT_T SdDrReadSector(UI_32 addr, UI_32 cnt, void *buf, SINT_T type,
                      struct SdDrCfg_s *cfg);
SINT_T SdDrWriteSector(UI_32 addr, UI_32 cnt, void *buf, SINT_T type,
                       struct SdDrCfg_s *cfg);
SINT_T SdDrEraseSector(UI_32 addr, UI_32 cnt, struct SdDrCfg_s *cfg);
SINT_T SdDrSetClock(UI_32 limitclk, UI_32 sysclk, struct SdDrCfg_s *cfg);
SINT_T SdDrChangeSpeedMode(SINT_T mode, struct SdDrCfg_s *cfg);
SINT_T SdDrRefMediaType(struct SdDrCfg_s *cfg);
SINT_T SdDrSleep(struct SdDrCfg_s *cfg);
SINT_T SdDrAwake(struct SdDrCfg_s *cfg);
SINT_T SdDrSelectAccessPartition(UI_32 partnumber, struct SdDrCfg_s *cfg);
SINT_T SdDrConfigBootMode(SINT_T enable, SINT_T ack, UI_32 bootpartnumber,
                          UI_32 bootbuswidth, struct SdDrCfg_s *cfg);
SINT_T SdDrGetPartitionSize(UI_32 partnumber, UI_32 *secNum,
                            struct SdDrCfg_s *cfg);

SINT_T SdDrCheckSDXC(struct SdDrCfg_s *cfg);

SINT_T SdDrEraseSeq(UI_32 type, UI_32 addr, UI_32 cnt, struct SdDrCfg_s *cfg);

#define SdDrTrimSector(addr, cnt, cfg) \
  SdDrEraseSeq(0x00000001, addr, cnt, cfg)
#define SdDrSTrimSector1(addr, cnt, cfg) \
  SdDrEraseSeq(0x80000001, addr, cnt, cfg)
#define SdDrSTrimSector2(cfg) \
  SdDrEraseSeq(0x80008000,    0,   1, cfg)

SINT_T SdDrGeneralCommand(SINT_T arg, UI_32 size, void *buf, SINT_T type,
                          struct SdDrCfg_s *cfg);
SINT_T SdDrCacheCtrl(SINT_T ctrl, struct SdDrCfg_s *cfg);



#define SDDR_SUPPORT_TRIM(cfg) \
  ((SdDrRefMediaType(cfg) == SDDR_MEDIA_TYPE_MMC) ? \
   (((cfg)->ex.mmc.extcsd_sec_feature_support & (1UL << 4)) ? 1 : 0) : 0)

#define SDDR_SUPPORT_CACHE(cfg) \
  ((SdDrRefMediaType(cfg) == SDDR_MEDIA_TYPE_MMC) ? \
   ((cfg)->ex.mmc.extcsd_cache_size ? 1:0) : 0)

SINT_T SdDrBkopsGetStatus(struct SdDrCfg_s *cfg);
SINT_T SdDrBkopsEnable(SINT_T ena, struct SdDrCfg_s *cfg);
SINT_T SdDrBkopsStart(struct SdDrCfg_s *cfg);
SINT_T SdDrHpiEnable(SINT_T ena, struct SdDrCfg_s *cfg);
SINT_T SdDrHpiExec(SINT_T check, struct SdDrCfg_s *cfg);

SINT_T fixedSdDrReadSector(UI_32 addr, UI_32 cnt, void *buf, SINT_T type,
                           struct SdDrCfg_s *cfg);

UI_32 sdif_get_status(UI_32);

#if defined(__cplusplus)
}
#endif
#undef EXTERN

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_LC823450_LC823450_SDDRV_IF_H */
