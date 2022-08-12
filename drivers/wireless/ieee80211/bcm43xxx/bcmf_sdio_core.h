/****************************************************************************
 * drivers/wireless/ieee80211/bcm43xxx/bcmf_sdio_core.h
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 ****************************************************************************/

#ifndef __DRIVERS_WIRELESS_IEEE80211_BCM43XXX_BCMF_SDIO_CORE_H
#define __DRIVERS_WIRELESS_IEEE80211_BCM43XXX_BCMF_SDIO_CORE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef PAD
#define _PADLINE(line) pad ## line
#define _XSTR(line)    _PADLINE(line)
#define PAD            _XSTR(__LINE__)
#endif

/* SDIO device ID */

#define SDIO_DEVICE_ID_BROADCOM_43012        43012
#define SDIO_DEVICE_ID_BROADCOM_43013        43013
#define SDIO_DEVICE_ID_BROADCOM_43143        43143
#define SDIO_DEVICE_ID_BROADCOM_43241        0x4324
#define SDIO_DEVICE_ID_BROADCOM_4329         0x4329
#define SDIO_DEVICE_ID_BROADCOM_4330         0x4330
#define SDIO_DEVICE_ID_BROADCOM_4334         0x4334
#define SDIO_DEVICE_ID_BROADCOM_4335_4339    0x4335
#define SDIO_DEVICE_ID_BROADCOM_43362        43362
#define SDIO_DEVICE_ID_BROADCOM_43430        43430
#define SDIO_DEVICE_ID_BROADCOM_43455        0x4345
#define SDIO_DEVICE_ID_INFINEON_CYW43439     43439

/* Core reg address translation.
 * Both macro's returns a 32 bits byte address on the backplane bus.
 */

#define CORE_CC_REG(base, field) \
        (base + offsetof(struct chipcregs, field))
#define CORE_BUS_REG(base, field) \
        (base + offsetof(struct sdpcmd_regs, field))
#define CORE_SB(base, field) \
        (base + offsetof(struct sbconfig, field))

#define BRCMF_MAX_CORENUM   6
#define SI_ENUM_BASE        0x18000000 /* Enumeration space base */

/* Target state register description */

#define  SSB_TMSLOW_RESET     0x00000001 /* Reset */
#define  SSB_TMSLOW_REJECT    0x00000002 /* Reject (Standard Backplane) */
#define  SSB_TMSLOW_REJECT_23 0x00000004 /* Reject (Backplane rev 2.3) */
#define  SSB_TMSLOW_CLOCK     0x00010000 /* Clock Enable */
#define  SSB_TMSLOW_FGC       0x00020000 /* Force Gated Clocks On */
#define  SSB_TMSLOW_PE        0x40000000 /* Power Management Enable */
#define  SSB_TMSLOW_BE        0x80000000 /* BIST Enable */

#define I_HMB_SW_MASK                 ( (uint32_t) 0x000000F0 )
#define I_HMB_FRAME_IND               ( 1<<6 )

/* tosbmailbox bits corresponding to intstatus bits */

#define SMB_NAK     (1 << 0)  /* Frame NAK */
#define SMB_INT_ACK (1 << 1)  /* Host Interrupt ACK */
#define SMB_USE_OOB (1 << 2)  /* Use OOB Wakeup */
#define SMB_DEV_INT (1 << 3)  /* Miscellaneous Interrupt */

enum
{
  CHIPCOMMON_CORE_ID = 0,
  DOT11MAC_CORE_ID,
  SDIOD_CORE_ID,
#if defined(CONFIG_IEEE80211_BROADCOM_BCM4301X) || \
    defined(CONFIG_IEEE80211_BROADCOM_BCM43362) || \
    defined(CONFIG_IEEE80211_BROADCOM_BCM43438) || \
    defined(CONFIG_IEEE80211_INFINEON_CYW43439)
  WLAN_ARMCM3_CORE_ID,
  SOCSRAM_CORE_ID,
#endif
#if defined(CONFIG_IEEE80211_BROADCOM_BCM43455)
  WLAN_ARMCR4_CORE_ID,
#endif
  MAX_CORE_ID
};

struct chip_core_info
{
  uint16_t id;
  uint16_t rev;
  uint32_t base;
  uint32_t wrapbase;
  uint32_t caps;
  uint32_t cib;
};

struct sbconfig
{
  uint8_t  PAD[0xf00];
  uint32_t PAD[2];
  uint32_t sbipsflag;     /* initiator port ocp slave flag */
  uint32_t PAD[3];
  uint32_t sbtpsflag;     /* target port ocp slave flag */
  uint32_t PAD[11];
  uint32_t sbtmerrloga;   /* (sonics >= 2.3) */
  uint32_t PAD;
  uint32_t sbtmerrlog;    /* (sonics >= 2.3) */
  uint32_t PAD[3];
  uint32_t sbadmatch3;    /* address match3 */
  uint32_t PAD;
  uint32_t sbadmatch2;    /* address match2 */
  uint32_t PAD;
  uint32_t sbadmatch1;    /* address match1 */
  uint32_t PAD[7];
  uint32_t sbimstate;     /* initiator agent state */
  uint32_t sbintvec;      /* interrupt mask */
  uint32_t sbtmstatelow;  /* target state */
  uint32_t sbtmstatehigh; /* target state */
  uint32_t sbbwa0;        /* bandwidth allocation table0 */
  uint32_t PAD;
  uint32_t sbimconfiglow;  /* initiator configuration */
  uint32_t sbimconfighigh; /* initiator configuration */
  uint32_t sbadmatch0;     /* address match0 */
  uint32_t PAD;
  uint32_t sbtmconfiglow;  /* target configuration */
  uint32_t sbtmconfighigh; /* target configuration */
  uint32_t sbbconfig;      /* broadcast configuration */
  uint32_t PAD;
  uint32_t sbbstate;       /* broadcast state */
  uint32_t PAD[3];
  uint32_t sbactcnfg;      /* activate configuration */
  uint32_t PAD[3];
  uint32_t sbflagst;       /* current sbflags */
  uint32_t PAD[3];
  uint32_t sbidlow;        /* identification */
  uint32_t sbidhigh;       /* identification */
};

/* sdio core registers */

struct sdpcmd_regs
{
  uint32_t corecontrol;        /* 0x00, rev8 */
  uint32_t corestatus;         /* rev8 */
  uint32_t PAD[1];
  uint32_t biststatus;         /* rev8 */

  /* PCMCIA access */

  uint16_t pcmciamesportaladdr;  /* 0x010, rev8 */
  uint16_t PAD[1];
  uint16_t pcmciamesportalmask;  /* rev8 */
  uint16_t PAD[1];
  uint16_t pcmciawrframebc;      /* rev8 */
  uint16_t PAD[1];
  uint16_t pcmciaunderflowtimer; /* rev8 */
  uint16_t PAD[1];

  /* interrupt */

  uint32_t intstatus;         /* 0x020, rev8 */
  uint32_t hostintmask;       /* rev8 */
  uint32_t intmask;           /* rev8 */
  uint32_t sbintstatus;       /* rev8 */
  uint32_t sbintmask;         /* rev8 */
  uint32_t funcintmask;       /* rev4 */
  uint32_t PAD[2];
  uint32_t tosbmailbox;       /* 0x040, rev8 */
  uint32_t tohostmailbox;     /* rev8 */
  uint32_t tosbmailboxdata;   /* rev8 */
  uint32_t tohostmailboxdata; /* rev8 */

  /* synchronized access to registers in SDIO clock domain */

  uint32_t sdioaccess;        /* 0x050, rev8 */
  uint32_t PAD[3];

  /* PCMCIA frame control */

  uint8_t pcmciaframectrl;    /* 0x060, rev8 */
  uint8_t PAD[3];
  uint8_t pcmciawatermark;    /* rev8 */
  uint8_t PAD[155];

  /* interrupt batching control */

  uint32_t intrcvlazy;        /* 0x100, rev8 */
  uint32_t PAD[3];

  /* counters */

  uint32_t cmd52rd;           /* 0x110, rev8 */
  uint32_t cmd52wr;           /* rev8 */
  uint32_t cmd53rd;           /* rev8 */
  uint32_t cmd53wr;           /* rev8 */
  uint32_t abort;             /* rev8 */
  uint32_t datacrcerror;      /* rev8 */
  uint32_t rdoutofsync;       /* rev8 */
  uint32_t wroutofsync;       /* rev8 */
  uint32_t writebusy;         /* rev8 */
  uint32_t readwait;          /* rev8 */
  uint32_t readterm;          /* rev8 */
  uint32_t writeterm;         /* rev8 */
  uint32_t PAD[40];
  uint32_t clockctlstatus;    /* rev8 */
  uint32_t PAD[7];

  uint32_t PAD[128];          /* DMA engines */

  /* SDIO/PCMCIA CIS region */

  char cis[512];              /* 0x400-0x5ff, rev6 */

  /* PCMCIA function control registers */

  char pcmciafcr[256];        /* 0x600-6ff, rev6 */
  uint16_t PAD[55];

  /* PCMCIA backplane access */

  uint16_t backplanecsr;      /* 0x76E, rev6 */
  uint16_t backplaneaddr0;    /* rev6 */
  uint16_t backplaneaddr1;    /* rev6 */
  uint16_t backplaneaddr2;    /* rev6 */
  uint16_t backplaneaddr3;    /* rev6 */
  uint16_t backplanedata0;    /* rev6 */
  uint16_t backplanedata1;    /* rev6 */
  uint16_t backplanedata2;    /* rev6 */
  uint16_t backplanedata3;    /* rev6 */
  uint16_t PAD[31];

  /* sprom "size" & "blank" info */

  uint16_t spromstatus;       /* 0x7BE, rev2 */
  uint32_t PAD[464];

  uint16_t PAD[0x80];
};

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __DRIVERS_WIRELESS_IEEE80211_BCM43XXX_BCMF_SDIO_CORE_H */
