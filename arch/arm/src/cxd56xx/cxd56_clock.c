/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_clock.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/arch.h>

#include <assert.h>
#include <debug.h>
#include <stdio.h>
#include <stdint.h>

#include <arch/chip/pm.h>

#include "arm_arch.h"

#include "chip.h"
#include "hardware/cxd56_crg.h"
#include "hardware/cxd5602_backupmem.h"
#include "hardware/cxd5602_topreg.h"

#include "cxd56_clock.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Debug */

#ifdef CONFIG_CXD56_PM_DEBUG_ERROR
#  define pmerr(format, ...)   _err(format, ##__VA_ARGS__)
#else
#  define pmerr(x, ...)
#endif
#ifdef CONFIG_CXD56_PM_DEBUG_WARN
#  define pmwarn(format, ...)  _warn(format, ##__VA_ARGS__)
#else
#  define pmwarn(x, ...)
#endif
#ifdef CONFIG_CXD56_PM_DEBUG_INFO
#  define pminfo(format, ...)  _info(format, ##__VA_ARGS__)
#else
#  define pminfo(x, ...)
#endif

/* For enable_pwd, disable_pwd (digital domain) */

#define PDID_SCU         0
#define PDID_APP_DSP     9
#define PDID_APP_SUB    10
#define PDID_APP_AUD    14

/* For enable_apwd, disable_apwd (analog domain) */

#define APDID_RCOSC      0
#define APDID_XOSC       1
#define APDID_HPADC     12
#define APDID_LPADC     13

/* Compiler hint shortcut */

#define __unused __attribute__((unused))

#define ALIGNUP(v, a) (((v) + ((a) - 1)) & ~((a) - 1))
#define TILESIZESHIT 17
#define TILESIZE     (1 << TILESIZESHIT)
#define TILEALIGN(v)  ALIGNUP(v, TILESIZE)
#define TILEALIGNIDX(v) (((v) >> TILESIZESHIT) & 0xf)

#ifndef CONFIG_CXD56_UART2_BASE_CLOCK_DIVIDER
#define CONFIG_CXD56_UART2_BASE_CLOCK_DIVIDER 4
#endif

/* Flags for IMG device active
 *
 * This flags for fixed clock devices.
 */

#define FLAG_IMG_CISIF   (1 << 0)
#define FLAG_IMG_GE2D    (1 << 1)

/****************************************************************************
 * Private Types
 ****************************************************************************/

enum clock_source
{
  RCOSC = 1,
  RTC,
  RCRTC,
  XOSC,
  SYSPLL,
};

struct scu_peripheral
{
  int8_t cken;
  int8_t swreset;
  int8_t crgintmask;
  int8_t reserved;
};

struct power_domain
{
  uint8_t refs[16];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void cxd56_img_clock_enable(void);
static void cxd56_img_clock_disable(void);
static void cxd56_scu_clock_ctrl(uint32_t block, uint32_t intr, int on);
static void cxd56_scu_peri_clock_enable(FAR const struct scu_peripheral *p)
  __unused;
static void cxd56_scu_peri_clock_disable(FAR const struct scu_peripheral *p)
  __unused;
static void cxd56_scu_peri_clock_gating(FAR const struct scu_peripheral *p,
                                        int enable) __unused;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct power_domain g_digital;
static struct power_domain g_analog;

/* Store calibrated RCOSC */

static uint32_t rcosc_clock = 0;

/* Save used IMG block devices */

static uint32_t g_active_imgdevs = 0;

/* Exclusive control */

static sem_t g_clockexc = SEM_INITIALIZER(1);

/* For peripherals inside SCU block
 *
 * Related registers are:
 *  cken       : SCU_CKEN
 *  swreset    : SWRESET_SCU
 *  crgintmask : CRG_INT_CLR0, CRG_INT_STAT_RAW0
 *
 * Each member values are indicated the number of bit
 * in appropriate registers.
 */

#if defined(CONFIG_CXD56_SPI3)
const struct scu_peripheral g_scuspi =
{
  .cken = 3,
  .swreset = 8,
  .crgintmask = 10,
};
#endif
#if defined(CONFIG_CXD56_I2C0)
const struct scu_peripheral g_scui2c0 =
{
  .cken = 1,
  .swreset = 5,
  .crgintmask = 11,
};
#endif
#if defined(CONFIG_CXD56_I2C1)
const struct scu_peripheral g_scui2c1 =
{
  .cken = 2,
  .swreset = 6,
  .crgintmask = 12,
};
#endif
#if defined(CONFIG_CXD56_SCUSEQ)
const struct scu_peripheral g_scuseq =
{
  .cken = 4,
  .swreset = 7,
  .crgintmask = 13,
};
#endif
#if defined(CONFIG_CXD56_ADC)
const struct scu_peripheral g_sculpadc =
{
  .cken = 6,
  .swreset = 4,
  .crgintmask = 17,
};
const struct scu_peripheral g_scuhpadc =
{
  .cken = 7,
  .swreset = 2,
  .crgintmask = 16,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void clock_semtake(sem_t *id)
{
  if (!up_interrupt_context())
    {
      nxsem_wait_uninterruptible(id);
    }
}

static void clock_semgive(sem_t *id)
{
  if (!up_interrupt_context())
    {
      nxsem_post(id);
    }
}

static void busy_wait(int cnt)
{
  for (; cnt; cnt--)
    {
      getreg32(CXD56_TOPREG_CHIP_ID);
    }
}

static void do_power_control(void)
{
  uint32_t stat;

  putreg32(0xf1f, CXD56_TOPREG_PMU_INT_CLR);
  putreg32(0xf1f, CXD56_TOPREG_PMU_INT_MASK);
  putreg32(1, CXD56_TOPREG_PMU_PW_CTL);

  do
    {
      stat = getreg32(CXD56_TOPREG_PMU_RAW_INT_STAT);
      stat &= 0x1f;
    }
  while (stat == 0);

  DEBUGASSERT(stat == 1);

  putreg32(0xf1f, CXD56_TOPREG_PMU_INT_CLR);
}

static inline void release_pwd_reset(uint32_t domain)
{
  /* Reset acts only belows
   *   [ 0] SCU
   *   [ 6] SYSIOP_SUB
   *   [ 8] APP
   *   [12] GNSS_ITP
   *   [13] GNSS
   */

  if (domain & 0x3141)
    {
      /* Release power domain reset */

      putreg32(domain | domain << 16, CXD56_TOPREG_PWD_RESET0);
    }
}

static void enable_pwd(int pdid)
{
  uint32_t stat;
  int domain = 1u << pdid;

  stat = getreg32(CXD56_TOPREG_PWD_STAT);
  if ((stat & domain) != domain)
    {
      putreg32((domain | (domain << 16)), CXD56_TOPREG_PWD_CTL);
      do_power_control();
      release_pwd_reset(domain);
    }

  g_digital.refs[pdid]++;
}

static void disable_pwd(int pdid)
{
  uint32_t stat;
  int domain = 1u << pdid;

  stat = getreg32(CXD56_TOPREG_PWD_STAT);
  if (stat & domain)
    {
      g_digital.refs[pdid]--;
      if (g_digital.refs[pdid] == 0)
        {
          putreg32(domain << 16, CXD56_TOPREG_PWD_CTL);
          do_power_control();
        }
    }
}

static void enable_apwd(int apdid)
{
  uint32_t stat;
  int domain = 1u << apdid;

  stat = getreg32(CXD56_TOPREG_ANA_PW_STAT);
  if ((stat & domain) != domain)
    {
      putreg32(domain | (domain << 16), CXD56_TOPREG_ANA_PW_CTL);
      do_power_control();
    }

  g_analog.refs[apdid]++;
}

static void disable_apwd(int apdid)
{
  uint32_t stat;
  int domain = 1u << apdid;

  stat = getreg32(CXD56_TOPREG_ANA_PW_STAT);
  if (stat & domain)
    {
      g_analog.refs[apdid]--;
      if (g_analog.refs[apdid] == 0)
        {
          putreg32(domain << 16, CXD56_TOPREG_ANA_PW_CTL);
          do_power_control();
        }
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cxd56_rcosc_enable
 *
 * Description:
 *
 ****************************************************************************/

void cxd56_rcosc_enable(void)
{
  enable_apwd(APDID_RCOSC);
}

/****************************************************************************
 * Name: cxd56_rcosc_disable
 *
 * Description:
 *
 ****************************************************************************/

void cxd56_rcosc_disable(void)
{
  disable_apwd(APDID_RCOSC);
}

/****************************************************************************
 * Name: cxd56_xosc_enable
 *
 * Description:
 *   Enable XOSC (if needed)
 *
 ****************************************************************************/

void cxd56_xosc_enable(void)
{
  enable_apwd(APDID_XOSC);
}

/****************************************************************************
 * Name: cxd56_xosc_disable
 *
 * Description:
 *   Disable XOSC.
 *
 * CAUTION:
 *   This function is tentative. We need to consider that
 *   clock source control  with other devices which XOSC is used.
 *
 ****************************************************************************/

void cxd56_xosc_disable(void)
{
  disable_apwd(APDID_XOSC);
}

/****************************************************************************
 * Name: cxd56_spif_clock_enable
 *
 * Description:
 *
 ****************************************************************************/

void cxd56_spif_clock_enable(void)
{
  uint32_t val;
  uint32_t rst;

  val = getreg32(CXD56_TOPREG_SYSIOP_SUB_CKEN);
  if (val & CK_SFC)
    {
      return;
    }

  putreg32(val | CK_SFC, CXD56_TOPREG_SYSIOP_SUB_CKEN);

  busy_wait(10);

  putreg32(val, CXD56_TOPREG_SYSIOP_SUB_CKEN);
  rst = getreg32(CXD56_TOPREG_SWRESET_BUS);
  putreg32(rst | XRST_SFC, CXD56_TOPREG_SWRESET_BUS);
  putreg32(val | CK_SFC, CXD56_TOPREG_SYSIOP_SUB_CKEN);
}

/****************************************************************************
 * Name: cxd56_spif_clock_disable
 *
 * Description:
 *
 ****************************************************************************/

void cxd56_spif_clock_disable(void)
{
  uint32_t val;
  uint32_t rst;

  val = getreg32(CXD56_TOPREG_SYSIOP_SUB_CKEN);
  if (!(val & CK_SFC))
    {
      return;
    }

  putreg32(val & ~CK_SFC, CXD56_TOPREG_SYSIOP_SUB_CKEN);
  rst = getreg32(CXD56_TOPREG_SWRESET_BUS);
  putreg32(rst & ~XRST_SFC, CXD56_TOPREG_SWRESET_BUS);
}

/****************************************************************************
 * Name: cxd56_get_cpu_baseclk
 *
 * Description:
 *   Get CPU clock.
 *
 ****************************************************************************/

uint32_t cxd56_get_cpu_baseclk(void)
{
  uint32_t val;
  int n;
  int m;

  val = getreg32(CXD56_CRG_GEAR_AHB);
  n = (val >> 16) & 0x7f;
  m = val & 0x7f;

  if (n && m)
    {
      return cxd56_get_appsmp_baseclock() * n / m;
    }
  else
    {
      return 0;
    }
}

/****************************************************************************
 * Name: cxd56_cpu_clock_enable
 *
 * Description:
 *
 ****************************************************************************/

void cxd56_cpu_clock_enable(int cpu)
{
  cxd56_cpulist_clock_enable(1 << cpu);
}

/****************************************************************************
 * Name: cxd56_cpulist_clock_enable
 *
 * Description:
 *
 ****************************************************************************/

void cxd56_cpulist_clock_enable(uint32_t cpus)
{
  uint32_t c;
  uint32_t bits = (cpus & 0x3f) << 16;

  c = getreg32(CXD56_CRG_CK_GATE_AHB);
  putreg32(c | bits, CXD56_CRG_CK_GATE_AHB);
}

/****************************************************************************
 * Name: cxd56_cpu_clock_disable
 *
 * Description:
 *
 ****************************************************************************/

void cxd56_cpu_clock_disable(int cpu)
{
  cxd56_cpulist_clock_disable(1 << cpu);
}

/****************************************************************************
 * Name: cxd56_cpulist_clock_disable
 *
 * Description:
 *
 ****************************************************************************/

void cxd56_cpulist_clock_disable(uint32_t cpus)
{
  uint32_t c;
  uint32_t bits = (cpus & 0x3f) << 16;

  c = getreg32(CXD56_CRG_CK_GATE_AHB);
  putreg32(c & ~bits, CXD56_CRG_CK_GATE_AHB);
}

/****************************************************************************
 * Name: cxd56_cpu_reset
 *
 * Description:
 *
 ****************************************************************************/

void cxd56_cpu_reset(int cpu)
{
  cxd56_cpulist_reset(1 << cpu);
}

/****************************************************************************
 * Name: cxd56_cpulist_reset
 *
 * Description:
 *
 ****************************************************************************/

void cxd56_cpulist_reset(uint32_t cpus)
{
  uint32_t c;
  uint32_t r;
  uint32_t bits = (cpus & 0x3f) << 16;

  /* Reset assert */

  r = getreg32(CXD56_CRG_RESET);
  putreg32(r & ~bits, CXD56_CRG_RESET);

  /* Temporary provide clock for perform reset */

  c = getreg32(CXD56_CRG_CK_GATE_AHB);
  putreg32(c | bits, CXD56_CRG_CK_GATE_AHB);
  busy_wait(10);
  putreg32(c, CXD56_CRG_CK_GATE_AHB);

  /* Reset deassert */

  putreg32(r | bits, CXD56_CRG_RESET);
}

/****************************************************************************
 * Name: cxd56_usb_clock_enable
 *
 * Description:
 *
 ****************************************************************************/

void cxd56_usb_clock_enable(void)
{
  uint32_t c;
  uint32_t r;

  enable_pwd(PDID_APP_SUB);

  c = getreg32(CXD56_CRG_CK_GATE_AHB);
  if (!(c & CK_GATE_USB))
    {
      r = getreg32(CXD56_CRG_RESET);
      putreg32(r & ~XRS_USB, CXD56_CRG_RESET);
      putreg32(c | CK_GATE_USB, CXD56_CRG_CK_GATE_AHB);
      busy_wait(10);
      putreg32(r | XRS_USB, CXD56_CRG_RESET);
      putreg32(1, CXD56_TOPREG_USBPHY_CKEN);
      putreg32(0x00010002, CXD56_CRG_GEAR_PER_USB);
    }
}

/****************************************************************************
 * Name: cxd56_usb_clock_disable
 *
 * Description:
 *
 ****************************************************************************/

void cxd56_usb_clock_disable(void)
{
  uint32_t c;
  uint32_t r;

  c = getreg32(CXD56_CRG_CK_GATE_AHB);
  if (c & CK_GATE_USB)
    {
      putreg32(0, CXD56_CRG_GEAR_PER_USB);
      putreg32(0, CXD56_TOPREG_USBPHY_CKEN);
      putreg32(c & ~CK_GATE_USB, CXD56_CRG_CK_GATE_AHB);
      r = getreg32(CXD56_CRG_RESET);
      putreg32(r & ~XRS_USB, CXD56_CRG_RESET);
    }

  disable_pwd(PDID_APP_SUB);
}

/****************************************************************************
 * Name: cxd56_emmc_clock_enable
 *
 * Description:
 *
 ****************************************************************************/

void cxd56_emmc_clock_enable(uint32_t div, uint32_t driver, uint32_t sample)
{
  uint32_t c;
  uint32_t r;
  uint32_t g;

  enable_pwd(PDID_APP_SUB);

  c = getreg32(CXD56_CRG_CKEN_EMMC);
  if (c == 7)
    {
      /* already enabled */

      return;
    }

  r = getreg32(CXD56_CRG_RESET);
  r = (r & ~XRS_MMC) | XRS_MMC_CRG;
  putreg32(r, CXD56_CRG_RESET);

  g = getreg32(CXD56_CRG_CK_GATE_AHB);
  putreg32(g | CK_GATE_MMC, CXD56_CRG_CK_GATE_AHB);
  putreg32(7, CXD56_CRG_CKEN_EMMC);

  busy_wait(10);

  putreg32(g & ~CK_GATE_MMC, CXD56_CRG_CK_GATE_AHB);
  putreg32(0, CXD56_CRG_CKEN_EMMC);

  r |= XRS_MMC;
  putreg32(r, CXD56_CRG_RESET);

  putreg32(g | CK_GATE_MMC, CXD56_CRG_CK_GATE_AHB);

  /* T.B.D: This register located at eMMC controller */

  *((volatile uint32_t *)0x4e201108) = (div << 30) |
    ((driver & 0x7f) << 23) | ((sample & 0x7f) << 16);

  putreg32(7, CXD56_CRG_CKEN_EMMC);
}

/****************************************************************************
 * Name: cxd56_emmc_clock_disable
 *
 * Description:
 *
 ****************************************************************************/

void cxd56_emmc_clock_disable(void)
{
  uint32_t c;
  uint32_t r;
  uint32_t g;

  c = getreg32(CXD56_CRG_CKEN_EMMC);
  if (c != 7)
    {
      return;
    }

  g = getreg32(CXD56_CRG_CK_GATE_AHB);
  putreg32(g & ~CK_GATE_MMC, CXD56_CRG_CK_GATE_AHB);
  putreg32(0, CXD56_CRG_CKEN_EMMC);

  r = getreg32(CXD56_CRG_RESET);
  putreg32(r & ~(XRS_MMC | XRS_MMC_CRG), CXD56_CRG_RESET);

  disable_pwd(PDID_APP_SUB);
}

/****************************************************************************
 * Name: cxd56_sdio_clock_enable
 *
 * Description:
 *
 ****************************************************************************/

void cxd56_sdio_clock_enable(void)
{
  uint32_t c;
  uint32_t r;

  enable_pwd(PDID_APP_SUB);

  c = getreg32(CXD56_CRG_CK_GATE_AHB);
  if (!(c & CK_GATE_SDIO))
    {
      r = getreg32(CXD56_CRG_RESET);
      putreg32(r & ~XRS_SDIO, CXD56_CRG_RESET);
      putreg32(c | CK_GATE_SDIO, CXD56_CRG_CK_GATE_AHB);
      putreg32(0x00010002, CXD56_CRG_GEAR_PER_SDIO);
      busy_wait(10);
      putreg32(r | XRS_SDIO, CXD56_CRG_RESET);
    }
}

/****************************************************************************
 * Name: cxd56_sdio_clock_disable
 *
 * Description:
 *
 ****************************************************************************/

void cxd56_sdio_clock_disable(void)
{
  uint32_t c;
  uint32_t r;

  c = getreg32(CXD56_CRG_CK_GATE_AHB);
  if (c & CK_GATE_SDIO)
    {
      putreg32(0, CXD56_CRG_GEAR_PER_SDIO);
      putreg32(c & ~CK_GATE_SDIO, CXD56_CRG_CK_GATE_AHB);
      r = getreg32(CXD56_CRG_RESET);
      putreg32(r & ~XRS_SDIO, CXD56_CRG_RESET);
    }

  disable_pwd(PDID_APP_SUB);
}

/****************************************************************************
 * Name: cxd56_audio_clock_enable
 *
 * Description:
 *
 ****************************************************************************/

void cxd56_audio_clock_enable(uint32_t clk, uint32_t div)
{
  enable_pwd(PDID_APP_AUD);

  modifyreg32(CXD56_TOPREG_APP_CKSEL, AUD_MCLK_MASK, clk);
  if (AUD_MCLK_XOSC == clk)
    {
      putreg32(div, CXD56_TOPREG_APP_DIV);
    }

  modifyreg32(CXD56_TOPREG_APP_CKEN, 0, APP_CKEN_MCLK);
  modifyreg32(CXD56_CRG_RESET, 0, XRS_AUD);
  modifyreg32(CXD56_CRG_CK_GATE_AHB, 0, CK_GATE_AUD);
}

/****************************************************************************
 * Name: cxd56_audio_clock_disable
 *
 * Description:
 *
 ****************************************************************************/

void cxd56_audio_clock_disable(void)
{
  modifyreg32(CXD56_CRG_RESET, XRS_AUD, 0);
  modifyreg32(CXD56_CRG_CK_GATE_AHB, CK_GATE_AUD, 0);
  modifyreg32(CXD56_TOPREG_APP_CKEN, APP_CKEN_MCLK, 0);

  disable_pwd(PDID_APP_AUD);
}

/****************************************************************************
 * Name: cxd56_audio_clock_is_enabled
 *
 * Description:
 *
 ****************************************************************************/

bool cxd56_audio_clock_is_enabled(void)
{
  return (getreg32(CXD56_CRG_CK_GATE_AHB) & CK_GATE_AUD) == CK_GATE_AUD;
}

#if defined(CONFIG_CXD56_SPI0)
/****************************************************************************
 * Name: cxd56_spim_clock_enable
 *
 * Description:
 *   Enable SPI channel 0 device clock. This is a SYS domain peripheral.
 *   I expect SYSIOP_SUB power domain already enabled.
 *
 ****************************************************************************/

static void cxd56_spim_clock_enable(void)
{
  uint32_t val;
  uint32_t rst;

  val = getreg32(CXD56_TOPREG_SYSIOP_SUB_CKEN);
  if (val & CK_SPIM)
    {
      return;
    }

  putreg32(val | CK_SPIM | CK_COM_BRG |
    CK_AHB_BRG_COMIF, CXD56_TOPREG_SYSIOP_SUB_CKEN);

  busy_wait(10);

  putreg32(val | CK_COM_BRG | CK_AHB_BRG_COMIF,
           CXD56_TOPREG_SYSIOP_SUB_CKEN);
  rst = getreg32(CXD56_TOPREG_SWRESET_BUS);
  putreg32(rst | XRST_SPIM, CXD56_TOPREG_SWRESET_BUS);
  putreg32(val | CK_SPIM | CK_COM_BRG |
    CK_AHB_BRG_COMIF, CXD56_TOPREG_SYSIOP_SUB_CKEN);
}

/****************************************************************************
 * Name: cxd56_spim_clock_disable
 *
 * Description:
 *   Disable SPI channel 0 device clock.
 *
 ****************************************************************************/

static void cxd56_spim_clock_disable(void)
{
  uint32_t val;
  uint32_t rst;
  uint32_t mask;

  val = getreg32(CXD56_TOPREG_SYSIOP_SUB_CKEN);
  if (!(val & CK_SPIM))
    {
      return;
    }

  mask = CK_SPIM;
  if (!(val & (CK_UART1 | CK_I2CM)))
    {
      mask |= CK_COM_BRG | CK_AHB_BRG_COMIF;
    }

  putreg32(val & ~mask, CXD56_TOPREG_SYSIOP_SUB_CKEN);
  rst = getreg32(CXD56_TOPREG_SWRESET_BUS);
  putreg32(rst & ~XRST_SPIM, CXD56_TOPREG_SWRESET_BUS);
}
#endif

#if defined(CONFIG_CXD56_SPI4)
/****************************************************************************
 * Name: cxd56_img_spi_clock_enable
 *
 * Description:
 *   Enable SPI channel 4 device clock.
 *   This is called IMG_SPI, located at APP domain and inside of IMG block.
 *
 ****************************************************************************/

static void cxd56_img_spi_clock_enable(void)
{
  clock_semtake(&g_clockexc);
  enable_pwd(PDID_APP_SUB);
  cxd56_img_clock_enable();
  putreg32(0x00010002, CXD56_CRG_GEAR_IMG_SPI);
  clock_semgive(&g_clockexc);
}

/****************************************************************************
 * Name: cxd56_img_spi_clock_disable
 *
 * Description:
 *   Disable SPI channel 4 device clock.
 *
 ****************************************************************************/

static void cxd56_img_spi_clock_disable(void)
{
  clock_semtake(&g_clockexc);
  putreg32(0, CXD56_CRG_GEAR_IMG_SPI);
  cxd56_img_clock_disable();
  disable_pwd(PDID_APP_SUB);
  clock_semgive(&g_clockexc);
}
#endif

#if defined(CONFIG_CXD56_SPI5)
/****************************************************************************
 * Name: cxd56_img_wspi_clock_enable
 *
 * Description:
 *   Enable SPI channel 5 device clock.
 *   This is called IMG_WSPI, located at APP domain and inside of IMG block.
 *
 ****************************************************************************/

static void cxd56_img_wspi_clock_enable(void)
{
  clock_semtake(&g_clockexc);
  enable_pwd(PDID_APP_SUB);
  cxd56_img_clock_enable();
  putreg32(0x00010004, CXD56_CRG_GEAR_IMG_WSPI);
  clock_semgive(&g_clockexc);
}

/****************************************************************************
 * Name: cxd56_img_wspi_clock_disable
 *
 * Description:
 *   Disable SPI channel 5 device clock.
 *
 ****************************************************************************/

static void cxd56_img_wspi_clock_disable(void)
{
  clock_semtake(&g_clockexc);
  putreg32(0, CXD56_CRG_GEAR_IMG_WSPI);
  cxd56_img_clock_disable();
  disable_pwd(PDID_APP_SUB);
  clock_semgive(&g_clockexc);
}
#endif

void cxd56_spi_clock_enable(int port)
{
#if defined(CONFIG_CXD56_SPI4)
  if (port == 4)
    {
      cxd56_img_spi_clock_enable();
    }

#endif
#if defined(CONFIG_CXD56_SPI5)
  if (port == 5)
    {
      cxd56_img_wspi_clock_enable();
    }

#endif
#if defined(CONFIG_CXD56_SPI0)
  if (port == 0)
    {
      cxd56_spim_clock_enable();
    }

#endif
#if defined(CONFIG_CXD56_SPI3)
  if (port == 3)
    {
      cxd56_scu_peri_clock_enable(&g_scuspi);
    }

#endif
}

void cxd56_spi_clock_disable(int port)
{
#if defined(CONFIG_CXD56_SPI4)
  if (port == 4)
    {
      cxd56_img_spi_clock_disable();
    }

#endif
#if defined(CONFIG_CXD56_SPI5)
  if (port == 5)
    {
      cxd56_img_wspi_clock_disable();
    }

#endif
#if defined(CONFIG_CXD56_SPI0)
  if (port == 0)
    {
      cxd56_spim_clock_disable();
    }

#endif
#if defined(CONFIG_CXD56_SPI3)
  if (port == 3)
    {
      cxd56_scu_peri_clock_disable(&g_scuspi);
    }

#endif
}

/****************************************************************************
 * Name: cxd56_spi_clock_gate_enable
 *
 * Description:
 *
 ****************************************************************************/

void cxd56_spi_clock_gate_enable(int port)
{
#if defined(CONFIG_CXD56_SPI3)
  if (port == 3)
    {
      cxd56_scu_peri_clock_gating(&g_scuspi, 1);
    }
#endif
}

/****************************************************************************
 * Name: cxd56_spi_clock_gate_disable
 *
 * Description:
 *
 ****************************************************************************/

void cxd56_spi_clock_gate_disable(int port)
{
#if defined(CONFIG_CXD56_SPI3)
  if (port == 3)
    {
      cxd56_scu_peri_clock_gating(&g_scuspi, 0);
    }
#endif
}

/****************************************************************************
 * Name: cxd56_spi_clock_gear_adjust
 *
 * Description:
 *
 ****************************************************************************/

void cxd56_spi_clock_gear_adjust(int port, uint32_t maxfreq)
{
  uint32_t baseclock;
  uint32_t gear;
  uint32_t divisor;
  uint32_t maxdivisor;
  uint32_t addr;

  if (maxfreq == 0)
    {
      return;
    }

#if defined(CONFIG_CXD56_SPI4)
  if (port == 4)
    {
      maxdivisor = 0x7f;
      addr       = CXD56_CRG_GEAR_IMG_SPI;
    }
  else
#endif
#if defined(CONFIG_CXD56_SPI5)
  if (port == 5)
    {
      maxdivisor = 0xf;
      addr       = CXD56_CRG_GEAR_IMG_WSPI;
    }
  else
#endif
    {
      return;
    }

  clock_semtake(&g_clockexc);
  baseclock = cxd56_get_appsmp_baseclock();
  if (baseclock != 0)
    {
      divisor = baseclock / (maxfreq * 2);
      if (baseclock % (maxfreq * 2))
        {
          divisor += 1;
        }

      if (divisor > maxdivisor)
        {
          divisor = maxdivisor;
        }

      gear = 0x00010000 | divisor;
      putreg32(gear, addr);
    }

  clock_semgive(&g_clockexc);
}

#if defined(CONFIG_CXD56_I2C2)
/****************************************************************************
 * Name: cxd56_i2cm_clock_enable
 *
 * Description:
 *
 ****************************************************************************/

static void cxd56_i2cm_clock_enable(void)
{
  uint32_t val;
  uint32_t rst;

  val = getreg32(CXD56_TOPREG_SYSIOP_SUB_CKEN);
  if (val & CK_I2CM)
    {
      return;
    }

  putreg32(val | CK_I2CM | CK_COM_BRG |
    CK_AHB_BRG_COMIF, CXD56_TOPREG_SYSIOP_SUB_CKEN);

  busy_wait(10);

  putreg32(val | CK_COM_BRG | CK_AHB_BRG_COMIF,
           CXD56_TOPREG_SYSIOP_SUB_CKEN);
  rst = getreg32(CXD56_TOPREG_SWRESET_BUS);
  putreg32(rst | XRST_I2CM, CXD56_TOPREG_SWRESET_BUS);
  putreg32(val | CK_I2CM | CK_COM_BRG |
    CK_AHB_BRG_COMIF, CXD56_TOPREG_SYSIOP_SUB_CKEN);
}

/****************************************************************************
 * Name: cxd56_i2c0_clock_disable
 *
 * Description:
 *
 ****************************************************************************/

static void cxd56_i2cm_clock_disable(void)
{
  uint32_t val;
  uint32_t rst;
  uint32_t mask;

  val = getreg32(CXD56_TOPREG_SYSIOP_SUB_CKEN);
  if (!(val & CK_I2CM))
    {
      return;
    }

  mask = CK_I2CM;
  if (!(val & (CK_UART1 | CK_SPIM)))
    {
      mask |= CK_COM_BRG | CK_AHB_BRG_COMIF;
    }

  putreg32(val & ~mask, CXD56_TOPREG_SYSIOP_SUB_CKEN);
  rst = getreg32(CXD56_TOPREG_SWRESET_BUS);
  putreg32(rst & ~XRST_I2CM, CXD56_TOPREG_SWRESET_BUS);
}
#endif

/****************************************************************************
 * Name: cxd56_i2c_clock_enable
 *
 * Description:
 *
 ****************************************************************************/

void cxd56_i2c_clock_enable(int port)
{
#if defined(CONFIG_CXD56_I2C0)
  if (port == 0)
    {
      cxd56_scu_peri_clock_enable(&g_scui2c0);
    }

#endif
#if defined(CONFIG_CXD56_I2C1)
  if (port == 1)
    {
      cxd56_scu_peri_clock_enable(&g_scui2c1);
    }

#endif
#if defined(CONFIG_CXD56_I2C2)
  if (port == 2)
    {
      cxd56_i2cm_clock_enable();
    }

#endif
}

/****************************************************************************
 * Name: cxd56_i2c1_clock_disable
 *
 * Description:
 *
 ****************************************************************************/

void cxd56_i2c_clock_disable(int port)
{
#if defined(CONFIG_CXD56_I2C0)
  if (port == 0)
    {
      cxd56_scu_peri_clock_disable(&g_scui2c0);
    }

#endif
#if defined(CONFIG_CXD56_I2C1)
  if (port == 1)
    {
      cxd56_scu_peri_clock_disable(&g_scui2c1);
    }

#endif
#if defined(CONFIG_CXD56_I2C2)
  if (port == 2)
    {
      cxd56_i2cm_clock_disable();
    }

#endif
}

/****************************************************************************
 * Name: cxd56_i2c_clock_gate_enable
 *
 * Description:
 *
 ****************************************************************************/

void cxd56_i2c_clock_gate_enable(int port)
{
#if defined(CONFIG_CXD56_I2C0)
  if (port == 0)
    {
      cxd56_scu_peri_clock_gating(&g_scui2c0, 1);
    }

#endif
#if defined(CONFIG_CXD56_I2C1)
  if (port == 1)
    {
      cxd56_scu_peri_clock_gating(&g_scui2c1, 1);
    }

#endif
}

/****************************************************************************
 * Name: cxd56_i2c_clock_gate_disable
 *
 * Description:
 *
 ****************************************************************************/

void cxd56_i2c_clock_gate_disable(int port)
{
#if defined(CONFIG_CXD56_I2C0)
  if (port == 0)
    {
      cxd56_scu_peri_clock_gating(&g_scui2c0, 0);
    }

#endif
#if defined(CONFIG_CXD56_I2C1)
  if (port == 1)
    {
      cxd56_scu_peri_clock_gating(&g_scui2c1, 0);
    }

#endif
}

uint32_t cxd56_get_img_uart_baseclock(void)
{
  uint32_t val;
  int n;
  int m;

  val = getreg32(CXD56_CRG_GEAR_IMG_UART);
  n = (val >> 16) & 1;
  m = val & 0x7f;

  if (n && m)
    {
      return cxd56_get_appsmp_baseclock() * n / m;
    }
    else
    {
      return 0;
    }
}

/****************************************************************************
 * Name: cxd56_img_uart_clock_enable
 *
 * Description:
 *   Enable img uart clock.
 *
 ****************************************************************************/

void cxd56_img_uart_clock_enable()
{
  uint32_t val = 0;

  clock_semtake(&g_clockexc);

  enable_pwd(PDID_APP_SUB);
  cxd56_img_clock_enable();

  val = getreg32(CXD56_CRG_GEAR_IMG_UART);
  val |= (1UL << 16);
#ifdef CONFIG_CXD56_UART2
  val &= ~0x7f;
  val |= CONFIG_CXD56_UART2_BASE_CLOCK_DIVIDER;
#endif /* CONFIG_CXD56_UART2 */
  putreg32(val, CXD56_CRG_GEAR_IMG_UART);

  clock_semgive(&g_clockexc);
}

/****************************************************************************
 * Name: cxd56_img_uart_clock_dsiable
 *
 * Description:
 *   Disable img uart clock.
 *
 ****************************************************************************/

void cxd56_img_uart_clock_disable()
{
  uint32_t val = 0;

  clock_semtake(&g_clockexc);

  val = getreg32(CXD56_CRG_GEAR_IMG_UART);
  val &= ~(1UL << 16);
  putreg32(val, CXD56_CRG_GEAR_IMG_UART);

  cxd56_img_clock_disable();
  disable_pwd(PDID_APP_SUB);

  clock_semgive(&g_clockexc);
}

/****************************************************************************
 * Name: cxd56_img_cisif_clock_enable
 *
 * Description:
 *   Enable img cisif clock.
 *
 ****************************************************************************/

void cxd56_img_cisif_clock_enable(void)
{
  clock_semtake(&g_clockexc);

  enable_pwd(PDID_APP_SUB);
  cxd56_img_clock_enable();
  g_active_imgdevs |= FLAG_IMG_CISIF;

  clock_semgive(&g_clockexc);
}

/****************************************************************************
 * Name: cxd56_img_cisif_clock_disable
 *
 * Description:
 *   Disable img cisif clock.
 *
 ****************************************************************************/

void cxd56_img_cisif_clock_disable(void)
{
  clock_semtake(&g_clockexc);

  g_active_imgdevs &= ~FLAG_IMG_CISIF;
  cxd56_img_clock_disable();
  disable_pwd(PDID_APP_SUB);

  clock_semgive(&g_clockexc);
}

/****************************************************************************
 * Name: cxd56_img_ge2d_clock_enable
 *
 * Description:
 *   Enable img cisif clock.
 *
 ****************************************************************************/

void cxd56_img_ge2d_clock_enable(void)
{
  clock_semtake(&g_clockexc);

  enable_pwd(PDID_APP_SUB);
  cxd56_img_clock_enable();
  g_active_imgdevs |= FLAG_IMG_GE2D;

  clock_semgive(&g_clockexc);
}

/****************************************************************************
 * Name: cxd56_img_ge2d_clock_disable
 *
 * Description:
 *   Disable img cisif clock.
 *
 ****************************************************************************/

void cxd56_img_ge2d_clock_disable(void)
{
  clock_semtake(&g_clockexc);

  g_active_imgdevs &= ~FLAG_IMG_GE2D;
  cxd56_img_clock_disable();
  disable_pwd(PDID_APP_SUB);

  clock_semgive(&g_clockexc);
}

static uint32_t cxd56_get_clock(enum clock_source cs)
{
  if (!rcosc_clock)
    {
      rcosc_clock = BKUP->rcosc_clock;
    }

  switch (cs)
    {
    case RCOSC:
      return rcosc_clock;
    case RTC:
      return 32768;
    case RCRTC:
      return rcosc_clock / 250;
    case XOSC:
      return CONFIG_CXD56_XOSC_CLOCK;
    case SYSPLL:
      {
        uint32_t ctrl;
        uint32_t rc;
        uint32_t fb;

        ctrl = getreg32(CXD56_TOPREG_SYS_PLL_CTRL2);
        rc = ctrl >> 30;
        fb = (ctrl >> 27) & 0x7;

        switch (rc)
          {
          case 0:
            rc = 1;
            break;
          case 1:
            rc = 2;
            break;
          case 3:
            rc = 4;
            break;
          }

        switch (fb)
          {
          case 0:
            fb = 10;
            break;
          case 1:
            fb = 12;
            break;
          case 2:
            fb = 15;
            break;
          }

        return CONFIG_CXD56_XOSC_CLOCK * fb / rc;
      }
    }

  return 0;
}

uint32_t cxd56_get_sys_baseclock(void)
{
  uint32_t val;

  val = getreg32(CXD56_TOPREG_CKSEL_ROOT);

  switch ((val >> 22) & 0x3)
    {
    case 0:
      return cxd56_get_clock(RCOSC);

    case 1:
      {
        uint32_t div = ((val >> 10) & 0x3) + 1;

        if (div == 4 && (val & (1 << 2)))
          {
            div = 5;
          }

        return cxd56_get_clock(SYSPLL) / div;
      }

    case 2:
      return cxd56_get_clock(XOSC);

    case 3:
      return cxd56_get_clock(RTC);
    }

  return 0;
}

uint32_t cxd56_get_scu_baseclock(void)
{
  uint32_t val;

  val = getreg32(CXD56_TOPREG_CKSEL_SCU);

  switch (val & 0x3)
    {
    case 0:
      return cxd56_get_clock(RCOSC);

    case 1:
      return cxd56_get_clock(XOSC) / (((val >> 8) & 0x3) + 1);

    case 2:
      if (val & (1 << 4))
        {
          return cxd56_get_clock(RTC);
        }
      else
        {
          return cxd56_get_clock(RCRTC);
        }
    }

  return 0;
}

uint32_t cxd56_get_appsmp_baseclock(void)
{
  uint32_t val = getreg32(CXD56_TOPREG_APP_CKSEL);

  switch ((val >> 8) & 0x3)
    {
    case 0:
      return cxd56_get_clock(RCOSC);

    case 1:
      {
        uint32_t div = ((val >> 10) & 0x3) + 1;

        if (div == 4 && (val & (1 << 7)))
          {
            div = 5;
          }

        return cxd56_get_clock(SYSPLL) / div;
      }

    case 2:
      return cxd56_get_clock(XOSC);

    case 3:
      return cxd56_get_clock(RTC);
    }

  return 0;
}

uint32_t cxd56_get_com_baseclock(void)
{
  uint32_t clock = cxd56_get_sys_baseclock();
  uint32_t val = getreg32(CXD56_TOPREG_CKDIV_COM);

  return clock / ((val & 0x1f) + 1);
}

uint32_t cxd56_get_sdio_baseclock(void)
{
  uint32_t val;
  int n;
  int m;

  val = getreg32(CXD56_CRG_GEAR_PER_SDIO);
  n = (val >> 16) & 1;
  m = val & 0x3;

  if (m != 0)
    {
      return cxd56_get_appsmp_baseclock() * n / m;
    }
  else
    {
      return 0;
    }
}

uint32_t cxd56_get_img_spi_baseclock(void)
{
  uint32_t val;
  int n;
  int m;

  val = getreg32(CXD56_CRG_GEAR_IMG_SPI);
  n = (val >> 16) & 1;
  m = val & 0x7f;

  if (n && m)
    {
      return cxd56_get_appsmp_baseclock() * n / m;
    }
  else
    {
      return 0;
    }
}

uint32_t cxd56_get_img_wspi_baseclock(void)
{
  uint32_t val;
  int n;
  int m;

  val = getreg32(CXD56_CRG_GEAR_IMG_WSPI);
  n = (val >> 16) & 1;
  m = val & 0xf;

  if (n && m)
    {
      return cxd56_get_appsmp_baseclock() * n / m;
    }
  else
    {
      return 0;
    }
}

uint32_t cxd56_get_spi_baseclock(int port)
{
#if defined(CONFIG_CXD56_SPI4)
  if (port == 4)
    {
      return cxd56_get_img_spi_baseclock();
    }

#endif
#if defined(CONFIG_CXD56_SPI5)
  if (port == 5)
    {
      return cxd56_get_img_wspi_baseclock();
    }

#endif
#if defined(CONFIG_CXD56_SPI0)
  if (port == 0)
    {
      return cxd56_get_com_baseclock();
    }

#endif
#if defined(CONFIG_CXD56_SPI3)
  if (port == 3)
    {
      return cxd56_get_scu_baseclock();
    }

#endif
  return 0;
}

uint32_t cxd56_get_i2c_baseclock(int port)
{
#if defined(CONFIG_CXD56_I2C0)
  if (port == 0)
    {
      return cxd56_get_scu_baseclock();
    }

#endif
#if defined(CONFIG_CXD56_I2C1)
  if (port == 1)
    {
      return cxd56_get_scu_baseclock();
    }

#endif
#if defined(CONFIG_CXD56_I2C2)
  if (port == 2)
    {
      return cxd56_get_com_baseclock();
    }

#endif
  return 0;
}

uint32_t cxd56_get_pwm_baseclock(void)
{
  return cxd56_get_scu_baseclock();
}

static void cxd56_img_clock_enable(void)
{
  uint32_t val;

  val = getreg32(CXD56_CRG_CK_GATE_AHB);
  if (val & CK_GATE_IMG)
    {
      return;
    }

  putreg32(val | CK_GATE_IMG, CXD56_CRG_CK_GATE_AHB);
  val = getreg32(CXD56_CRG_RESET);
  putreg32(val | XRS_IMG, CXD56_CRG_RESET);
}

static void cxd56_img_clock_disable(void)
{
  uint32_t val;

  /* Check IMG block peripherals in use */

  val =  getreg32(CXD56_CRG_GEAR_IMG_UART) >> 16;
  val |= getreg32(CXD56_CRG_GEAR_IMG_SPI) >> 16;
  val |= getreg32(CXD56_CRG_GEAR_IMG_WSPI) >> 16;
  val |= getreg32(CXD56_CRG_GEAR_N_IMG_VENB);

  if (val || g_active_imgdevs)
    {
      return;
    }

  val = getreg32(CXD56_CRG_RESET);
  putreg32(val & ~XRS_IMG, CXD56_CRG_RESET);
  val = getreg32(CXD56_CRG_CK_GATE_AHB);
  putreg32(val & ~CK_GATE_IMG, CXD56_CRG_CK_GATE_AHB);
}

static void cxd56_scu_clock_ctrl(uint32_t block, uint32_t intr, int on)
{
  uint32_t val;
  uint32_t stat;
  int retry = 10000;

  putreg32(0xffffffff, CXD56_TOPREG_CRG_INT_CLR0);

  val = getreg32(CXD56_TOPREG_SCU_CKEN);
  if (on)
    {
      if ((val & block) == block)
        {
          /* Already clock on */

          return;
        }

      putreg32(val | block, CXD56_TOPREG_SCU_CKEN);
    }
  else
    {
      if ((val & block) == 0)
        {
          /* Already clock off */

          return;
        }

      putreg32(val & ~block, CXD56_TOPREG_SCU_CKEN);
    }

  do
    {
      stat = getreg32(CXD56_TOPREG_CRG_INT_STAT_RAW0);
      busy_wait(1000);
    }

  while (retry-- && !(stat & intr));

  putreg32(0xffffffff, CXD56_TOPREG_CRG_INT_CLR0);
}

static void cxd56_scu_clock_enable(void)
{
  uint32_t val;
  uint32_t stat;
  int retry = 1000;

  val = getreg32(CXD56_TOPREG_SYSIOP_CKEN);
  if (val & CKEN_BRG_SCU)
    {
      return;
    }

  /* SCU clock select default 0 (RCOSC) */

#ifdef CONFIG_CXD56_SCU_32K
  val = 2;
#elif defined CONFIG_CXD56_SCU_XOSC
  cxd56_xosc_enable();
  val = 1 | ((CONFIG_CXD56_SCU_XOSC_DIV - 1) << 8);
#else
  cxd56_rcosc_enable();
  val = 0;
#endif
#ifdef CONFIG_CXD56_SCU32K_RTC
  val |= 1 << 4;
#endif
  putreg32(val, CXD56_TOPREG_CKSEL_SCU);

  putreg32(0xffffffff, CXD56_TOPREG_CRG_INT_CLR0);

  /* Enable SYSIOP and SCU bridge */

  val = getreg32(CXD56_TOPREG_SYSIOP_CKEN);
  putreg32(val | CKEN_BRG_SCU, CXD56_TOPREG_SYSIOP_CKEN);

  /* Enable each blocks in SCU */

  val = getreg32(CXD56_TOPREG_SCU_CKEN);
  putreg32(val | SCU_SCU | SCU_SC | SCU_32K | SCU_SEQ,
           CXD56_TOPREG_SCU_CKEN);

  do
    {
      stat = getreg32(CXD56_TOPREG_CRG_INT_STAT_RAW0);
      busy_wait(1000);
    }
  while (retry-- &&
         !(stat & (CRG_CK_SCU |
                   CRG_CK_SCU_SC |
                   CRG_CK_BRG_SCU |
                   CRG_CK_32K |
                   CRG_CK_SCU_SEQ)));

  putreg32(0xffffffff, CXD56_TOPREG_CRG_INT_CLR0);
}

void cxd56_scu_clock_disable(void)
{
  uint32_t val;
  uint32_t stat;
  int retry = 1000;

  val = getreg32(CXD56_TOPREG_SYSIOP_CKEN);
  if (!(val & CKEN_BRG_SCU))
    {
      /* Already disabled */

      return;
    }

  val = getreg32(CXD56_TOPREG_SWRESET_SCU);
  putreg32(val & ~XRST_SCU_ISOP, CXD56_TOPREG_SWRESET_SCU);

  up_udelay(1);

  putreg32(0xffffffff, CXD56_TOPREG_CRG_INT_CLR0);

  /* Enable SYSIOP and SCU bridge */

  val = getreg32(CXD56_TOPREG_SYSIOP_CKEN);
  putreg32(val & ~CKEN_BRG_SCU, CXD56_TOPREG_SYSIOP_CKEN);

  /* Enable each blocks in SCU */

  val = getreg32(CXD56_TOPREG_SCU_CKEN);
  putreg32(val & ~(SCU_SCU | SCU_SC | SCU_32K | SCU_SEQ),
     CXD56_TOPREG_SCU_CKEN);

  do
    {
      stat = getreg32(CXD56_TOPREG_CRG_INT_STAT_RAW0);
      busy_wait(1000);
    }
  while (retry-- &&
         !(stat & (CRG_CK_SCU |
                   CRG_CK_SCU_SC |
                   CRG_CK_BRG_SCU |
                   CRG_CK_32K |
                   CRG_CK_SCU_SEQ)));

  putreg32(0xffffffff, CXD56_TOPREG_CRG_INT_CLR0);
}

bool cxd56_scuseq_clock_is_enabled(void)
{
  uint32_t rst;

  /* If SCU reset is already released, it assumes that the SCU sequencer is
   * already in running.
   */

  rst = getreg32(CXD56_TOPREG_SWRESET_SCU);
  if (rst & XRST_SCU_ISOP)
    {
      return true;
    }
  else
    {
      return false;
    }
}

int cxd56_scuseq_clock_enable(void)
{
  /* Enable SCU IRAM/DRAM & FIFO memory power.
   * Actual power control will running at SCU power control.
   */

  putreg32(0x133f, CXD56_TOPREG_TOP_SCU_RAMMODE_SEL);

  /* Up SCU power if needed */

  enable_pwd(PDID_SCU);

  cxd56_scu_clock_enable();

  return OK;
}

void cxd56_scuseq_release_reset(void)
{
  uint32_t rst;

  cxd56_scu_clock_ctrl(SCU_SEQ, CRG_CK_SCU_SEQ, 0);

  rst = getreg32(CXD56_TOPREG_SWRESET_SCU);
  putreg32(rst | XRST_SCU_ISOP, CXD56_TOPREG_SWRESET_SCU);

  cxd56_scu_clock_ctrl(SCU_SEQ, CRG_CK_SCU_SEQ, 1);
}

void cxd56_scuseq_clock_disable(void)
{
  uint32_t rst;

  cxd56_scu_clock_ctrl(SCU_SEQ, CRG_CK_SCU_SEQ, 0);

  rst = getreg32(CXD56_TOPREG_SWRESET_SCU);
  putreg32(rst & ~XRST_SCU_ISOP, CXD56_TOPREG_SWRESET_SCU);

  /* Down SCU power if needed */

  disable_pwd(PDID_SCU);
}

static void cxd56_scu_peri_clock_enable(FAR const struct scu_peripheral *p)
{
  uint32_t val;
  uint32_t rst;
  uint32_t cken = 1u << p->cken;
  uint32_t crgintmask = 1u << p->crgintmask;
  uint32_t swreset = 1u << p->swreset;

  /* Up SCU power if needed */

  enable_pwd(PDID_SCU);

  cxd56_scu_clock_enable();

  val = getreg32(CXD56_TOPREG_SCU_CKEN);
  if (val & cken)
    {
      return;
    }

  cxd56_scu_clock_ctrl(cken, crgintmask, 1);
  cxd56_scu_clock_ctrl(cken, crgintmask, 0);

  rst = getreg32(CXD56_TOPREG_SWRESET_SCU);
  putreg32(rst | swreset, CXD56_TOPREG_SWRESET_SCU);

  cxd56_scu_clock_ctrl(cken, crgintmask, 1);
}

static void cxd56_scu_peri_clock_disable(FAR const struct scu_peripheral *p)
{
  uint32_t val;
  uint32_t rst;
  uint32_t cken = 1u << p->cken;
  uint32_t crgintmask = 1u << p->crgintmask;
  uint32_t swreset = 1u << p->swreset;

  val = getreg32(CXD56_TOPREG_SCU_CKEN);
  if (!(val & cken))
    {
      return;
    }

  cxd56_scu_clock_ctrl(cken, crgintmask, 0);

  rst = getreg32(CXD56_TOPREG_SWRESET_SCU);
  putreg32(rst & ~swreset, CXD56_TOPREG_SWRESET_SCU);

  /* Down SCU power if needed */

  disable_pwd(PDID_SCU);
}

static void cxd56_scu_peri_clock_gating(
  FAR const struct scu_peripheral *p, int enable)
{
  uint32_t cken = 1u << p->cken;

  if (enable)
    {
      modifyreg32(CXD56_TOPREG_SCU_CKEN, cken, 0);  /* clock stop */
    }
  else
    {
      modifyreg32(CXD56_TOPREG_SCU_CKEN, 0, cken);  /* clock start */
    }
}

void cxd56_udmac_clock_enable(void)
{
  uint32_t val;
  val = getreg32(CXD56_TOPREG_SYSIOP_CKEN);
  putreg32(val | CKEN_AHB_DMAC0, CXD56_TOPREG_SYSIOP_CKEN);
}

void cxd56_udmac_clock_disable(void)
{
  uint32_t val;
  val = getreg32(CXD56_TOPREG_SYSIOP_CKEN);
  putreg32(val & ~CKEN_AHB_DMAC0, CXD56_TOPREG_SYSIOP_CKEN);
}

void cxd56_lpadc_clock_enable(uint32_t div)
{
#if defined(CONFIG_CXD56_ADC)
  uint32_t val;
  uint32_t mask;

  if (div > 4)
    {
      return;
    }

  enable_apwd(APDID_LPADC);

  mask = 0x0000000f;
  val = getreg32(CXD56_TOPREG_CKDIV_SCU) & ~mask;
  val |= div;
  putreg32(val, CXD56_TOPREG_CKDIV_SCU);

  cxd56_scu_peri_clock_enable(&g_sculpadc);
#endif
}

void cxd56_lpadc_clock_disable(void)
{
#if defined(CONFIG_CXD56_ADC)
  cxd56_scu_peri_clock_disable(&g_sculpadc);

  disable_apwd(APDID_LPADC);
#endif
}

void cxd56_hpadc_clock_enable(uint32_t div)
{
#if defined(CONFIG_CXD56_ADC)
  uint32_t val;
  uint32_t mask;

  if (div > 4)
    {
      return;
    }

  enable_apwd(APDID_HPADC);

  mask = 0x000000f0;
  val = getreg32(CXD56_TOPREG_CKDIV_SCU) & ~mask;
  val |= (div << 4);
  putreg32(val, CXD56_TOPREG_CKDIV_SCU);

  mask = 0x00004000;
  val = getreg32(CXD56_TOPREG_RCOSC_CTRL1) & ~mask;
  putreg32(val, CXD56_TOPREG_RCOSC_CTRL1);

  mask = 0x00020000;
  val = getreg32(CXD56_TOPREG_XOSC_CTRL) & ~mask;
  val |= mask;
  putreg32(val, CXD56_TOPREG_XOSC_CTRL);

  cxd56_scu_peri_clock_enable(&g_scuhpadc);
#endif
}

void cxd56_hpadc_clock_disable(void)
{
#if defined(CONFIG_CXD56_ADC)
  uint32_t val;
  uint32_t mask;

  mask = 0x00004000;
  val = getreg32(CXD56_TOPREG_RCOSC_CTRL1) & ~mask;
  val |= mask;
  putreg32(val, CXD56_TOPREG_RCOSC_CTRL1);

  mask = 0x00020000;
  val = getreg32(CXD56_TOPREG_XOSC_CTRL) & ~mask;
  putreg32(val, CXD56_TOPREG_XOSC_CTRL);

  cxd56_scu_peri_clock_disable(&g_scuhpadc);

  disable_apwd(APDID_HPADC);
#endif
}

uint32_t cxd56_get_xosc_clock(void)
{
  return cxd56_get_clock(XOSC);
}

uint32_t cxd56_get_rcosc_clock(void)
{
  return cxd56_get_clock(RCOSC);
}

uint32_t cxd56_get_rtc_clock(void)
{
  return cxd56_get_clock(RTC);
}

uint32_t cxd56_get_syspll_clock(void)
{
  return cxd56_get_clock(SYSPLL);
}

uint32_t cxd56_get_sys_ahb_baseclock(void)
{
  uint32_t bus;
  uint32_t ahb;

  bus = getreg32(CXD56_TOPREG_CKDIV_CPU_DSP_BUS);
  ahb = 1 << ((bus >> 16) & 0x7);
  return cxd56_get_sys_baseclock() / ahb;
}

uint32_t cxd56_get_sys_apb_baseclock(void)
{
  uint32_t bus;
  uint32_t apb;

  bus = getreg32(CXD56_TOPREG_CKDIV_CPU_DSP_BUS);
  apb = 1 << ((bus >> 24) & 0x3);
  return cxd56_get_sys_ahb_baseclock() / apb;
}

uint32_t cxd56_get_sys_sfc_baseclock(void)
{
  uint32_t bus;
  uint32_t sfchclk;

  bus = getreg32(CXD56_TOPREG_CKDIV_CPU_DSP_BUS);
  sfchclk = ((bus >> 28) & 0xf);
  if (sfchclk <= 9)
    {
      return (cxd56_get_sys_baseclock() / ((sfchclk * 2) + 2));
    }
  else
    {
      return (cxd56_get_sys_baseclock() / ((1 << (sfchclk - 10)) * 32));
    }
}

static uint32_t cxd56_get_suc32k_baseclock(void)
{
  uint32_t ckscu;

  ckscu = getreg32(CXD56_TOPREG_CKSEL_SCU);

  if (((ckscu >> 4) & 0x1) == 0)
    {
      return cxd56_get_clock(RCOSC) / 250;
    }
  else
    {
      return cxd56_get_clock(RTC);
    }
}

uint32_t cxd56_get_hpadc_baseclock(void)
{
  uint32_t divscu;

  divscu = getreg32(CXD56_TOPREG_CKDIV_SCU);
  return cxd56_get_suc32k_baseclock() / (1 << ((divscu >> 4) & 0xf));
}

uint32_t cxd56_get_lpadc_baseclock(void)
{
  uint32_t divscu;

  divscu = getreg32(CXD56_TOPREG_CKDIV_SCU);
  return cxd56_get_suc32k_baseclock() / (1 << (divscu & 0xf));
}

uint32_t cxd56_get_pmui2c_baseclock(void)
{
  uint32_t ckpmu;

  ckpmu = getreg32(CXD56_TOPREG_CKSEL_PMU);
  switch (ckpmu & 0x3)
    {
      case 0:
        return cxd56_get_sys_apb_baseclock();
        break;
      case 1:
        return cxd56_get_clock(RTC);
        break;
      case 2:
        return cxd56_get_clock(RCOSC);
        break;
      default:
        return 0;
        break;
    }
}

uint32_t cxd56_get_gps_cpu_baseclock(void)
{
  uint32_t gnssdiv;

  gnssdiv = getreg32(CXD56_TOPREG_GNSS_DIV);
  return cxd56_get_sys_baseclock() / ((gnssdiv & 0x1f) + 1);
}

uint32_t cxd56_get_gps_ahb_baseclock(void)
{
  uint32_t gnssdiv;

  gnssdiv = getreg32(CXD56_TOPREG_GNSS_DIV);
  return cxd56_get_sys_baseclock() / (((gnssdiv >> 16) & 0x1f) + 1);
}

uint32_t cxd56_get_usb_baseclock(void)
{
  uint32_t val;
  int n;
  int m;

  val = getreg32(CXD56_CRG_GEAR_PER_USB);
  n = (val >> 16) & 1;
  m = val & 0x3;

  if (m != 0)
    {
      return cxd56_get_appsmp_baseclock() * n / m;
    }
  else
    {
      return 0;
    }
}

uint32_t cxd56_get_img_vsync_baseclock(void)
{
  int n;
  int m;

  n = getreg32(CXD56_CRG_GEAR_N_IMG_VENB);
  m = getreg32(CXD56_CRG_GEAR_M_IMG_VENB);

  if (n != 0)
    {
      return cxd56_get_appsmp_baseclock() * n / m;
    }
  else
    {
      return 0;
    }
}

static int cxd56_hostif_clock_ctrl(uint32_t block, uint32_t intr, int on)
{
  uint32_t val;
  uint32_t stat;
  int      retry = 10000;

  putreg32(0xffffffff, CXD56_TOPREG_CRG_INT_CLR0);

  val = getreg32(CXD56_TOPREG_SYSIOP_CKEN);
  if (on)
    {
      if ((val & block) == block)
        {
          /* Already clock on */

          return OK;
        }

      putreg32(val | block, CXD56_TOPREG_SYSIOP_CKEN);
    }
  else
    {
      if ((val & block) == 0)
        {
          /* Already clock off */

          return OK;
        }

      putreg32(val & ~block, CXD56_TOPREG_SYSIOP_CKEN);
    }

  do
    {
      stat = getreg32(CXD56_TOPREG_CRG_INT_STAT_RAW0);
      busy_wait(1000);
    }
  while (retry-- && !(stat & intr));

  putreg32(0xffffffff, CXD56_TOPREG_CRG_INT_CLR0);

  return (retry) ? OK : -ETIMEDOUT;
}

int cxd56_hostif_clock_enable(void)
{
  int      ret = OK;
  uint32_t mask;
  uint32_t intr;

  /* Enable HOSTIF IRAM/DRAM & general RAM memory power. */

  putreg32((0x3 << 24) | 0xf, CXD56_TOPREG_HOSTIFC_RAMMODE_SEL);

  do_power_control();

  mask = CKEN_HOSSPI | CKEN_HOSI2C | CKEN_HOSTIFC_SEQ | CKEN_BRG_HOST |
    CKEN_I2CS | CKEN_PCLK_HOSTIFC | CKEN_PCLK_UART0 | CKEN_UART0;

  if (getreg32(CXD56_TOPREG_SYSIOP_CKEN) & mask)
    {
      /* Already enabled */

      return ret;
    }

  putreg32(0, CXD56_TOPREG_CKDIV_HOSTIFC);
  putreg32(0, CXD56_TOPREG_CKSEL_SYSIOP);

  mask = CKEN_HOSSPI | CKEN_HOSI2C | CKEN_BRG_HOST |
    CKEN_I2CS | CKEN_PCLK_HOSTIFC;

  intr = CRG_CK_BRG_HOST | CRG_CK_I2CS | CRG_CK_PCLK_HOSTIFC;

  ret = cxd56_hostif_clock_ctrl(mask, intr, 1);
  if (ret < 0)
    {
      return ret;
    }

  ret = cxd56_hostif_clock_ctrl(mask, intr, 0);
  if (ret < 0)
    {
      return ret;
    }

  modifyreg32(CXD56_TOPREG_SWRESET_BUS, 0, XRST_HOSTIFC);
  ret = cxd56_hostif_clock_ctrl(mask, intr, 1);

  return ret;
}

int cxd56_hostif_clock_disable(void)
{
  int      ret = OK;
  uint32_t mask;
  uint32_t intr;

  mask = CKEN_HOSSPI | CKEN_HOSI2C | CKEN_HOSTIFC_SEQ | CKEN_BRG_HOST |
    CKEN_I2CS |  CKEN_PCLK_HOSTIFC |  CKEN_PCLK_UART0 |  CKEN_UART0;

  if (0 == (getreg32(CXD56_TOPREG_SYSIOP_CKEN) & mask))
    {
      /* Already disabled */

      return ret;
    }

  mask = CKEN_HOSSPI | CKEN_HOSI2C | CKEN_BRG_HOST |
    CKEN_I2CS |  CKEN_PCLK_HOSTIFC;

  intr = CRG_CK_BRG_HOST | CRG_CK_I2CS | CRG_CK_PCLK_HOSTIFC;

  ret = cxd56_hostif_clock_ctrl(mask, intr, 0);
  if (ret < 0)
    {
      return ret;
    }

  modifyreg32(CXD56_TOPREG_SWRESET_BUS, XRST_HOSTIFC, 0);

  /* Disable HOSTIF IRAM/DRAM & general RAM memory power. */

  putreg32(0x3, CXD56_TOPREG_HOSTIFC_RAMMODE_SEL);

  do_power_control();

  return ret;
}

int cxd56_hostseq_clock_enable(void)
{
  int ret = OK;

  if (getreg32(CXD56_TOPREG_SYSIOP_CKEN) & CKEN_HOSTIFC_SEQ)
    {
      /* Already enabled */

      return ret;
    }

  ret = cxd56_hostif_clock_ctrl(CKEN_HOSTIFC_SEQ, CRG_CK_HOSTIFC_SEQ, 1);
  if (ret < 0)
    {
      return ret;
    }

  ret = cxd56_hostif_clock_ctrl(CKEN_HOSTIFC_SEQ, CRG_CK_HOSTIFC_SEQ, 0);
  if (ret < 0)
    {
      return ret;
    }

  modifyreg32(CXD56_TOPREG_SWRESET_BUS, 0, XRST_HOSTIFC_ISOP);
  ret = cxd56_hostif_clock_ctrl(CKEN_HOSTIFC_SEQ, CRG_CK_HOSTIFC_SEQ, 1);

  return ret;
}

int cxd56_hostseq_clock_disable(void)
{
  int ret = OK;

  if (0 == (getreg32(CXD56_TOPREG_SYSIOP_CKEN) & CKEN_HOSTIFC_SEQ))
    {
      /* Already disabled */

      return ret;
    }

  modifyreg32(CXD56_TOPREG_SWRESET_BUS, XRST_HOSTIFC_ISOP, 0);
  ret = cxd56_hostif_clock_ctrl(CKEN_HOSTIFC_SEQ, CRG_CK_HOSTIFC_SEQ, 0);

  return ret;
}

int up_pmramctrl(int cmd, uintptr_t addr, size_t size)
{
  int startidx;
  int endidx;
  int i;
  uint32_t mode;
  uint32_t mask;
  uint32_t ctrl;
  uint32_t stat;
  uint32_t val;
  int changed = 0;

  DEBUGASSERT(cmd == PMCMD_RAM_ON || cmd == PMCMD_RAM_OFF ||
              cmd == PMCMD_RAM_RET);

  /* Get tile index from address and size. */

  startidx = TILEALIGNIDX(addr);
  endidx = TILEALIGNIDX(TILEALIGN(addr + size));

  DEBUGASSERT(startidx < 12 && endidx <= 12);
  pminfo("%x (size: %x) [%d:%d] -> %d\n", addr, size,
         startidx, endidx, cmd);

  /* Make controls bits for RAM power control */

  mode = 0;
  mask = 0;
  ctrl = 0;
  for (i = startidx; i < endidx; i++)
    {
      mode |= cmd << (i * 2);
      mask |= 3 << (i * 2);
      ctrl |= 1 << i;
    }

  /* Determine mode changes on lower half tiles. */

  stat = getreg32(CXD56_TOPREG_APPDSP_RAMMODE_STAT0);
  if ((stat & (mask & 0xfff)) != (mode & 0xfff))
    {
      val = (ctrl & 0x3f) << 24 | (mode & 0xfff);
      putreg32(val, CXD56_TOPREG_APPDSP_RAMMODE_SEL0);
      changed = 1;
    }

  /* Shift all bits for upper tiles. */

  ctrl >>= 6;
  mode >>= 12;
  mask >>= 12;

  /* Determine mode changes on upper half tiles. */

  stat = getreg32(CXD56_TOPREG_APPDSP_RAMMODE_STAT1);
  if ((stat & (mask & 0xfff)) != (mode & 0xfff))
    {
      val = (ctrl & 0x3f) << 24 | (mode & 0xfff);
      putreg32(val, CXD56_TOPREG_APPDSP_RAMMODE_SEL1);
      changed = 1;
    }

  /* Apply RAM tile power status changes */

  if (changed)
    {
      do_power_control();

      /* Clock gating for inactive tiles. */

      stat  = getreg32(CXD56_TOPREG_APPDSP_RAMMODE_STAT1) << 12;
      stat |= getreg32(CXD56_TOPREG_APPDSP_RAMMODE_STAT0);
      val = 0;
      for (i = 0, mask = 3; i < 12; i++, mask <<= 2)
        {
          if ((stat & mask) == 0)
            {
              val |= 1 << i;
            }
        }

      putreg32(val, CXD56_CRG_APP_TILE_CLK_GATING_ENB);
    }

  return OK;
}

#ifdef CONFIG_CXD56_PM_DEBUG_INFO
/****************************************************************************
 * Name: up_pmstatdump
 *
 * Description:
 *   Print architecture specific power status
 *
 ****************************************************************************/

void up_pmstatdump(void)
{
  uint32_t stat0;
  uint32_t stat1;
  const char statch[] = " -?+"; /* OFF, retention, invalid, ON */
  const char gatech[] = "| ";   /* clock on, clock off */

  stat0 = getreg32(CXD56_TOPREG_APPDSP_RAMMODE_STAT0);
  stat1 = getreg32(CXD56_TOPREG_APPDSP_RAMMODE_STAT1);

  pminfo("              0 1 2 3 4 5 6 7 8 9 A B\n");
  pminfo("DSP RAM stat: %c %c %c %c %c %c %c %c %c %c %c %c\n",
         statch[(stat0 >>  0) & 3],
         statch[(stat0 >>  2) & 3],
         statch[(stat0 >>  4) & 3],
         statch[(stat0 >>  6) & 3],
         statch[(stat0 >>  8) & 3],
         statch[(stat0 >> 10) & 3],
         statch[(stat1 >>  0) & 3],
         statch[(stat1 >>  2) & 3],
         statch[(stat1 >>  4) & 3],
         statch[(stat1 >>  6) & 3],
         statch[(stat1 >>  8) & 3],
         statch[(stat1 >> 10) & 3]);

  stat0 = getreg32(CXD56_CRG_APP_TILE_CLK_GATING_ENB);
  pminfo("Clock gating: %c %c %c %c %c %c %c %c %c %c %c %c\n",
         gatech[(stat0 >>  0) & 1],
         gatech[(stat0 >>  1) & 1],
         gatech[(stat0 >>  2) & 1],
         gatech[(stat0 >>  3) & 1],
         gatech[(stat0 >>  4) & 1],
         gatech[(stat0 >>  5) & 1],
         gatech[(stat0 >>  6) & 1],
         gatech[(stat0 >>  7) & 1],
         gatech[(stat0 >>  8) & 1],
         gatech[(stat0 >>  9) & 1],
         gatech[(stat0 >> 10) & 1],
         gatech[(stat0 >> 11) & 1]);
}
#endif
