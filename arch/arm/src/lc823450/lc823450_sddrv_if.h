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

struct sdinfo_s
{
  UI_32 scr[2];
  UI_32 sdstatus[16];
};

/* MMC specific info */

struct mmcinfo_s
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

struct sddrcfg_s
{
  UI_32 regbase;         /* SD Host I/F register base address */
  UI_32 sysclk;          /* System Clock */
  UI_32 detecttime;      /* Card detection time */
  UI_32 setting;         /* WP CD settings */
  void  *workbuf;        /* Work buffer (512 byte) */

  SINT_T (*dephwinit)(struct sddrcfg_s *);
  SINT_T (*dephwexit)(struct sddrcfg_s *);
  SINT_T (*deposinit)(struct sddrcfg_s *);
  SINT_T (*deposexit)(struct sddrcfg_s *);
  void   (*depsetclk)(struct sddrcfg_s *);
  SINT_T (*depwait)(UI_32, struct sddrcfg_s *);
  SINT_T (*depwaitstatus)(UI_32 req, UI_32 *status, struct sddrcfg_s *cfg);
  SINT_T (*depreaddata)(void *src, void *dst, UI_32 size, SINT_T type,
                        struct sddrcfg_s *cfg);
  SINT_T (*depwritedata)(void *src, void *dst, UI_32 size, SINT_T type,
                         struct sddrcfg_s *cfg);
  void   (*depvoltageswitch)(struct sddrcfg_s *);

  /* To here, external members to be set
   * From here, internal mermbers (no need to set)
   */

  UI_32 info;            /* Misc info (e.g. driver state) */
  UI_32 secnum;          /* The number of sectors */
  UI_32 limitsdclk;      /* Max SD clock */
  UI_32 clkdiv;          /* Clock divider */
  UI_32 erasetmout;      /* Timeout for Erase/Trim */
  UI_32 cid[4];          /* CID */
  UI_32 csd[4];          /* CSD */

  union
  {
    struct sdinfo_s  sd;     /* SD specific info */
    struct mmcinfo_s mmc;    /* MMC specific info */
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
 * Public Function Prototypes
 ****************************************************************************/

SINT_T sddr_ref_version(void);
SINT_T sddr_initialize(struct sddrcfg_s *cfg);
SINT_T sddr_finalize(struct sddrcfg_s *cfg);
SINT_T sddr_identifycard(struct sddrcfg_s *cfg);
SINT_T sddr_checkcardidentify(struct sddrcfg_s *cfg);
SINT_T sddr_checkwriteenable(struct sddrcfg_s *cfg);
SINT_T sddr_checkcarddetect(struct sddrcfg_s *cfg);
SINT_T sddr_checkcardremoved(struct sddrcfg_s *cfg);
SINT_T sddr_getcardsize(UI_32 *secnum, UI_32 *secsize,
                        struct sddrcfg_s *cfg);
SINT_T sddr_getcid(UI_32 *cid, struct sddrcfg_s *cfg);
SINT_T sddr_getcsd(UI_32 *csd, struct sddrcfg_s *cfg);
SINT_T sddr_getscr(UI_32 *scr, struct sddrcfg_s *cfg);
SINT_T sddr_getextcsd(UI_32 *extcsd, struct sddrcfg_s *cfg);
SINT_T sddr_clearcardinfo(struct sddrcfg_s *cfg);
SINT_T sddr_readsector(UI_32 addr, UI_32 cnt, void *buf, SINT_T type,
                       struct sddrcfg_s *cfg);
SINT_T sddr_writesector(UI_32 addr, UI_32 cnt, void *buf, SINT_T type,
                        struct sddrcfg_s *cfg);
SINT_T sddr_erasesector(UI_32 addr, UI_32 cnt, struct sddrcfg_s *cfg);
SINT_T sddr_setclock(UI_32 limitclk, UI_32 sysclk, struct sddrcfg_s *cfg);
SINT_T sddr_changespeedmode(SINT_T mode, struct sddrcfg_s *cfg);
SINT_T sddr_refmediatype(struct sddrcfg_s *cfg);
SINT_T sddr_sleep(struct sddrcfg_s *cfg);
SINT_T sddr_awake(struct sddrcfg_s *cfg);
SINT_T sddr_selectaccesspartition(UI_32 partnumber, struct sddrcfg_s *cfg);
SINT_T sddr_configbootmode(SINT_T enable, SINT_T ack, UI_32 bootpartnumber,
                           UI_32 bootbuswidth, struct sddrcfg_s *cfg);
SINT_T sddr_getpartitionsize(UI_32 partnumber, UI_32 *secnum,
                             struct sddrcfg_s *cfg);

SINT_T sddr_checksdxc(struct sddrcfg_s *cfg);

SINT_T sddr_eraseseq(UI_32 type, UI_32 addr, UI_32 cnt,
                     struct sddrcfg_s *cfg);

#define sddr_trimsector(addr, cnt, cfg) \
  sddr_eraseseq(0x00000001, addr, cnt, cfg)
#define sddr_stimsector1(addr, cnt, cfg) \
  sddr_eraseseq(0x80000001, addr, cnt, cfg)
#define sddr_strimsector2(cfg) \
  sddr_eraseseq(0x80008000,    0,   1, cfg)

SINT_T sddr_generalcommand(SINT_T arg, UI_32 size, void *buf, SINT_T type,
                           struct sddrcfg_s *cfg);
SINT_T sddr_cachectrl(SINT_T ctrl, struct sddrcfg_s *cfg);

#define SDDR_SUPPORT_TRIM(cfg) \
  ((sddr_refmediatype(cfg) == SDDR_MEDIA_TYPE_MMC) ? \
   (((cfg)->ex.mmc.extcsd_sec_feature_support & (1UL << 4)) ? 1 : 0) : 0)

#define SDDR_SUPPORT_CACHE(cfg) \
  ((sddr_refmediatype(cfg) == SDDR_MEDIA_TYPE_MMC) ? \
   ((cfg)->ex.mmc.extcsd_cache_size ? 1:0) : 0)

SINT_T sddr_bkopsgetstatus(struct sddrcfg_s *cfg);
SINT_T sddr_bkopsenable(SINT_T ena, struct sddrcfg_s *cfg);
SINT_T sddr_bkopsstart(struct sddrcfg_s *cfg);
SINT_T sddr_hpienable(SINT_T ena, struct sddrcfg_s *cfg);
SINT_T sddr_hpiexec(SINT_T check, struct sddrcfg_s *cfg);

SINT_T fixed_sddr_readsector(UI_32 addr, UI_32 cnt, void *buf, SINT_T type,
                             struct sddrcfg_s *cfg);

UI_32 sdif_get_status(UI_32);

#if defined(__cplusplus)
}
#endif
#undef EXTERN

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_LC823450_LC823450_SDDRV_IF_H */
