/****************************************************************************
 * arch/arm/src/samd5e5/sam_clockconfig.c
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

#include <stdint.h>
#include <stdbool.h>
#include <assert.h>

#include "arm_internal.h"
#include "hardware/sam_pm.h"
#include "hardware/sam_supc.h"
#include "hardware/sam_oscctrl.h"
#include "hardware/sam_osc32kctrl.h"
#include "hardware/sam_gclk.h"
#include "hardware/sam_nvmctrl.h"
#include "sam_gclk.h"

#include "sam_periphclks.h"

#include <arch/board/board.h>  /* Normally must be included last */
#include "sam_clockconfig.h"   /* Needs to be included after board.h */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This describes the power-up clock configuration using data provided in the
 * board.h header file.
 */

static const struct sam_clockconfig_s g_initial_clocking =
{
  .waitstates        = BOARD_FLASH_WAITSTATES,
  .cpudiv            = BOARD_MCLK_CPUDIV,
  .gclkset1          = BOARD_GCLK_SET1,
  .gclkset2          = BOARD_GCLK_SET2,
  .cpu_frequency     = BOARD_CPU_FREQUENCY,
#if BOARD_HAVE_XOSC32K != 0
  .xosc32k          =
    {
      .enable         = BOARD_XOSC32K_ENABLE,
      .highspeed      = BOARD_XOSC32K_HIGHSPEED,
      .extalen        = BOARD_XOSC32K_XTALEN,
      .en32k          = BOARD_XOSC32K_EN32K,
      .en1k           = BOARD_XOSC32K_EN1K,
      .runstdby       = BOARD_XOSC32K_RUNSTDBY,
      .ondemand       = BOARD_XOSC32K_ONDEMAND,
      .cfden          = BOARD_XOSC32K_CFDEN,
      .cfdeo          = BOARD_XOSC32K_CFDEO,
      .caliben        = BOARD_XOSC32K_CALIBEN,
      .startup        = BOARD_XOSC32K_STARTUP,
      .calib          = BOARD_XOSC32K_CALIB,
      .rtcsel         = BOARD_XOSC32K_RTCSEL,
    },
#endif
#if BOARD_HAVE_XOSC0 != 0
  .xosc0            =
    {
      .enable         = BOARD_XOSC0_ENABLE,
      .extalen        = BOARD_XOSC0_XTALEN,
      .runstdby       = BOARD_XOSC0_RUNSTDBY,
      .ondemand       = BOARD_XOSC0_ONDEMAND,
      .lowgain        = BOARD_XOSC0_LOWGAIN,
      .enalc          = BOARD_XOSC0_ENALC,
      .cfden          = BOARD_XOSC0_CFDEN,
      .startup        = BOARD_XOSC0_STARTUP,
      .xosc_frequency = BOARD_XOSC0_FREQUENCY,
    },
#endif
#if BOARD_HAVE_XOSC1 != 0
  .xosc1            =
    {
      .enable         = BOARD_XOSC1_ENABLE,
      .extalen        = BOARD_XOSC1_XTALEN,
      .runstdby       = BOARD_XOSC1_RUNSTDBY,
      .ondemand       = BOARD_XOSC1_ONDEMAND,
      .lowgain        = BOARD_XOSC1_LOWGAIN,
      .enalc          = BOARD_XOSC1_ENALC,
      .cfden          = BOARD_XOSC1_CFDEN,
      .startup        = BOARD_XOSC1_STARTUP,
      .xosc_frequency = BOARD_XOSC1_FREQUENCY,
    },
#endif
  .dfll             =
    {
      .enable         = BOARD_DFLL_ENABLE,
      .runstdby       = BOARD_DFLL_RUNSTDBY,
      .ondemand       = BOARD_DFLL_ONDEMAND,
      .mode           = BOARD_DFLL_MODE,
      .stable         = BOARD_DFLL_STABLE,
      .llaw           = BOARD_DFLL_LLAW,
      .usbcrm         = BOARD_DFLL_USBCRM,
      .ccdis          = BOARD_DFLL_CCDIS,
      .qldis          = BOARD_DFLL_QLDIS,
      .bplckc         = BOARD_DFLL_BPLCKC,
      .waitlock       = BOARD_DFLL_WAITLOCK,
      .caliben        = BOARD_DFLL_CALIBEN,
      .gclklock       = BOARD_DFLL_GCLKLOCK,
      .fcalib         = BOARD_DFLL_FCALIB,
      .ccalib         = BOARD_DFLL_CCALIB,
      .fstep          = BOARD_DFLL_FSTEP,
      .cstep          = BOARD_DFLL_CSTEP,
      .gclk           = BOARD_DFLL_GCLK,
      .mul            = BOARD_DFLL_MUL
    },
  .dpll             =
    {
      {
        .enable       = BOARD_DPLL0_ENABLE,
        .dcoen        = BOARD_DPLL0_DCOEN,
        .lbypass      = BOARD_DPLL0_LBYPASS,
        .wuf          = BOARD_DPLL0_WUF,
        .runstdby     = BOARD_DPLL0_RUNSTDBY,
        .ondemand     = BOARD_DPLL0_ONDEMAND,
        .reflock      = BOARD_DPLL0_REFLOCK,
        .refclk       = BOARD_DPLL0_REFCLK,
        .ltime        = BOARD_DPLL0_LTIME,
        .filter       = BOARD_DPLL0_FILTER,
        .dcofilter    = BOARD_DPLL0_DCOFILTER,
        .gclk         = BOARD_DPLL0_GCLK,
        .ldrfrac      = BOARD_DPLL0_LDRFRAC,
        .ldrint       = BOARD_DPLL0_LDRINT,
        .div          = BOARD_DPLL0_DIV
      },
      {
        .enable       = BOARD_DPLL1_ENABLE,
        .dcoen        = BOARD_DPLL1_DCOEN,
        .lbypass      = BOARD_DPLL1_LBYPASS,
        .wuf          = BOARD_DPLL1_WUF,
        .runstdby     = BOARD_DPLL1_RUNSTDBY,
        .ondemand     = BOARD_DPLL1_ONDEMAND,
        .reflock      = BOARD_DPLL1_REFLOCK,
        .refclk       = BOARD_DPLL1_REFCLK,
        .ltime        = BOARD_DPLL1_LTIME,
        .filter       = BOARD_DPLL1_FILTER,
        .dcofilter    = BOARD_DPLL1_DCOFILTER,
        .gclk         = BOARD_DPLL1_GCLK,
        .ldrfrac      = BOARD_DPLL1_LDRFRAC,
        .ldrint       = BOARD_DPLL1_LDRINT,
        .div          = BOARD_DPLL1_DIV
      }
    },
  .gclk             =
    {
      {
        .enable         = BOARD_GCLK0_ENABLE,
        .oov            = BOARD_GCLK0_OOV,
        .oe             = BOARD_GCLK0_OE,
        .runstdby       = BOARD_GCLK0_RUNSTDBY,
        .source         = BOARD_GCLK0_SOURCE,
        .div            = BOARD_GCLK0_DIV
      },
      {
        .enable         = BOARD_GCLK1_ENABLE,
        .oov            = BOARD_GCLK1_OOV,
        .oe             = BOARD_GCLK1_OE,
        .runstdby       = BOARD_GCLK1_RUNSTDBY,
        .source         = BOARD_GCLK1_SOURCE,
        .div            = BOARD_GCLK1_DIV
      },
      {
        .enable         = BOARD_GCLK2_ENABLE,
        .oov            = BOARD_GCLK2_OOV,
        .oe             = BOARD_GCLK2_OE,
        .runstdby       = BOARD_GCLK2_RUNSTDBY,
        .source         = BOARD_GCLK2_SOURCE,
        .div            = BOARD_GCLK2_DIV
      },
      {
        .enable         = BOARD_GCLK3_ENABLE,
        .oov            = BOARD_GCLK3_OOV,
        .oe             = BOARD_GCLK3_OE,
        .runstdby       = BOARD_GCLK3_RUNSTDBY,
        .source         = BOARD_GCLK3_SOURCE,
        .div            = BOARD_GCLK3_DIV
      },
      {
        .enable         = BOARD_GCLK4_ENABLE,
        .oov            = BOARD_GCLK4_OOV,
        .oe             = BOARD_GCLK4_OE,
        .runstdby       = BOARD_GCLK4_RUNSTDBY,
        .source         = BOARD_GCLK4_SOURCE,
        .div            = BOARD_GCLK4_DIV
      },
      {
        .enable         = BOARD_GCLK5_ENABLE,
        .oov            = BOARD_GCLK5_OOV,
        .oe             = BOARD_GCLK5_OE,
        .runstdby       = BOARD_GCLK5_RUNSTDBY,
        .source         = BOARD_GCLK5_SOURCE,
        .div            = BOARD_GCLK5_DIV
      },
      {
        .enable         = BOARD_GCLK6_ENABLE,
        .oov            = BOARD_GCLK6_OOV,
        .oe             = BOARD_GCLK6_OE,
        .runstdby       = BOARD_GCLK6_RUNSTDBY,
        .source         = BOARD_GCLK6_SOURCE,
        .div            = BOARD_GCLK6_DIV
      },
      {
        .enable         = BOARD_GCLK7_ENABLE,
        .oov            = BOARD_GCLK7_OOV,
        .oe             = BOARD_GCLK7_OE,
        .runstdby       = BOARD_GCLK7_RUNSTDBY,
        .source         = BOARD_GCLK7_SOURCE,
        .div            = BOARD_GCLK7_DIV
      },
      {
        .enable         = BOARD_GCLK8_ENABLE,
        .oov            = BOARD_GCLK8_OOV,
        .oe             = BOARD_GCLK8_OE,
        .runstdby       = BOARD_GCLK8_RUNSTDBY,
        .source         = BOARD_GCLK8_SOURCE,
        .div            = BOARD_GCLK8_DIV
      },
      {
        .enable         = BOARD_GCLK9_ENABLE,
        .oov            = BOARD_GCLK9_OOV,
        .oe             = BOARD_GCLK9_OE,
        .runstdby       = BOARD_GCLK9_RUNSTDBY,
        .source         = BOARD_GCLK9_SOURCE,
        .div            = BOARD_GCLK9_DIV
      },
      {
        .enable         = BOARD_GCLK10_ENABLE,
        .oov            = BOARD_GCLK10_OOV,
        .oe             = BOARD_GCLK10_OE,
        .runstdby       = BOARD_GCLK10_RUNSTDBY,
        .source         = BOARD_GCLK10_SOURCE,
        .div            = BOARD_GCLK10_DIV
      },
      {
        .enable         = BOARD_GCLK11_ENABLE,
        .oov            = BOARD_GCLK11_OOV,
        .oe             = BOARD_GCLK11_OE,
        .runstdby       = BOARD_GCLK11_RUNSTDBY,
        .source         = BOARD_GCLK11_SOURCE,
        .div            = BOARD_GCLK11_DIV
      }
    }
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_get_waitstates
 *
 * Description:
 *   Set the number of FLASH wait states.
 *
 ****************************************************************************/

static uint16_t sam_get_waitstates(void)
{
  uint16_t regval = getreg16(SAM_NVMCTRL_CTRLA);
  return (regval & NVMCTRL_CTRLA_RWS_MASK) >> NVMCTRL_CTRLA_RWS_SHIFT;
}

/****************************************************************************
 * Name: sam_set_waitstates
 *
 * Description:
 *   Set the number of FLASH wait states.
 *
 ****************************************************************************/

static void sam_set_waitstates(const struct sam_clockconfig_s *config)
{
  DEBUGASSERT(config->waitstates < 16);
  modifyreg16(SAM_NVMCTRL_CTRLA, NVMCTRL_CTRLA_RWS_MASK,
              NVMCTRL_CTRLA_RWS(config->waitstates));
}

/****************************************************************************
 * Name: sam_xosc32k_configure
 *
 * Description:
 *   Configure XOSC32K
 *
 ****************************************************************************/

#if BOARD_HAVE_XOSC32K != 0
static void sam_xosc32k_configure(const struct sam_xosc32_config_s *config)
{
  uint32_t regval32;
  uint32_t regval16;
  uint32_t regval8;

  /* Configure the OSC32KCTRL register */

  regval16 = OSC32KCTRL_XOSC32K_STARTUP(config->startup);

  if (config->enable)
    {
      regval16 |= OSC32KCTRL_XOSC32K_ENABLE;
    }

  if (config->highspeed)
    {
      regval16 |= OSC32KCTRL_XOSC32K_GCM_HS;
    }

  if (config->extalen)
    {
      regval16 |= OSC32KCTRL_XOSC32K_XTALEN;
    }

  if (config->en32k)
    {
      regval16 |= OSC32KCTRL_XOSC32K_EN32K;
    }

  if (config->en1k)
    {
      regval16 |= OSC32KCTRL_XOSC32K_EN1K;
    }

  if (config->runstdby)
    {
      regval16 |= OSC32KCTRL_XOSC32K_RUNSTDBY;
    }

  if (config->ondemand)
    {
      regval16 |= OSC32KCTRL_XOSC32K_ONDEMAND;
    }

  putreg16(regval16, SAM_OSC32KCTRL_XOSC32K);

  /* Configure the CFDCTRL register */

  regval8 = config->cfden ? OSC32KCTRL_CFDCTRL_CFDEN : 0;
  putreg8(regval8, SAM_OSC32KCTRL_CFDCTRL);

  regval8 = config->cfdeo ? OSC32KCTRL_EVCTRL_CFDEO : 0;
  putreg8(regval8, SAM_OSC32KCTRL_EVCTRL);

  /* Setup OSCULP32K calibration */

  if (config->caliben)
    {
      regval32  = getreg32(SAM_OSC32KCTRL_OSCULP32K);
      regval32 &= ~OSC32KCTRL_OSCULP32K_CALIB_MASK;
      regval32 |= OSC32KCTRL_OSCULP32K_CALIB(config->calib);
    }

  /* Wait for XOSC32 to become ready if it was enabled */

  if (config->enable && !config->ondemand)
    {
      while ((getreg32(SAM_OSC32KCTRL_STATUS) &
              OSC32KCTRL_INT_XOSC32KRDY) == 0)
        {
        }
    }

  /* Set the RTC clock source */

  putreg8(OSC32KCTRL_RTCCTRL_RTCSEL(config->rtcsel),
          SAM_OSC32KCTRL_RTCCTRL);
}
#endif

/****************************************************************************
 * Name: sam_xoscctrl
 *
 * Description:
 *   Get the appropriate settings for the XOSCCTRL register for XOSC0 or 1.
 *
 ****************************************************************************/

#if BOARD_HAVE_XOSC0 != 0 || BOARD_HAVE_XOSC1 != 0
static uint32_t sam_xoscctrl(const struct sam_xosc_config_s *config)
{
  uint32_t regval;
  uint32_t cfdpresc;
  uint32_t imult;
  uint8_t iptat = 3;

  /* Some settings determined by the crystal frequency */

  if (config->xosc_frequency > 32000000)
    {
      cfdpresc = 0;
      imult    = 7;
      iptat    = 3;
    }
  else if (config->xosc_frequency > 24000000)
    {
      cfdpresc = 1;
      imult    = 6;
      iptat    = 3;
    }
  else if (config->xosc_frequency > 16000000)
    {
      cfdpresc = 2;
      imult    = 5;
      iptat    = 3;
    }
  else
    {
      cfdpresc = 3;
      imult    = 4;
      iptat    = 3;
    }

  /* Get the XOSCTCTL register *configuration */

  regval = OSCCTRL_XOSCCTRL_IPTAT(iptat) | OSCCTRL_XOSCCTRL_IMULT(imult) |
           OSCCTRL_XOSCCTRL_STARTUP(config->startup) |
           OSCCTRL_XOSCCTRL_CFDPRESC(cfdpresc);

  if (config->enable)
    {
      regval |= OSCCTRL_XOSCCTRL_ENABLE;
    }

  if (config->extalen)
    {
      regval |= OSCCTRL_XOSCCTRL_XTALEN;
    }

  if (config->runstdby)
    {
      regval |= OSCCTRL_XOSCCTRL_RUNSTDBY;
    }

  if (config->ondemand)
    {
      regval |= OSCCTRL_XOSCCTRL_ONDEMAND;
    }

  if (config->lowgain)
    {
      regval |= OSCCTRL_XOSCCTRL_LOWBUFGAIN;
    }

  if (config->enalc)
    {
      regval |= OSCCTRL_XOSCCTRL_ENALC;
    }

  if (config->cfden)
    {
      regval |= OSCCTRL_XOSCCTRL_CFDEN;
    }

  if (config->swben)
    {
      regval |= OSCCTRL_XOSCCTRL_SWBEN;
    }

  return regval;
}
#endif

/****************************************************************************
 * Name: sam_xosc0_configure
 *
 * Description:
 *   Configure XOSC0K
 *
 ****************************************************************************/

#if BOARD_HAVE_XOSC0 != 0
static void sam_xosc0_configure(const struct sam_xosc_config_s *config)
{
  uint32_t regval;

  /* Configure the XOSCTCTL register */

  regval = sam_xoscctrl(config);
  putreg32(regval, SAM_OSCCTRL_XOSCCTRL0);

  /* Wait for XOSC0 to become ready if it was enabled */

  if (config->enable)
    {
      while ((getreg32(SAM_OSCCTRL_STATUS) & OSCCTRL_INT_XOSCRDY0) == 0)
        {
        }
    }

  /* Re-select OnDemand */

  if (config->ondemand)
    {
      regval  = getreg32(SAM_OSCCTRL_XOSCCTRL0);
      regval |= OSCCTRL_XOSCCTRL_ONDEMAND;
      putreg32(regval, SAM_OSCCTRL_XOSCCTRL0);
    }
}
#endif

#if BOARD_HAVE_XOSC1 != 0
void sam_xosc1_configure(const struct sam_xosc_config_s *config)
{
  uint32_t regval;

  /* Configure the XOSCTCTL register */

  regval = sam_xoscctrl(config);
  putreg32(regval, SAM_OSCCTRL_XOSCCTRL1);

  /* Wait for XOSC1 to become ready if it was enabled */

  if (config->enable)
    {
      while ((getreg32(SAM_OSCCTRL_STATUS) & OSCCTRL_INT_XOSCRDY1) == 0)
        {
        }
    }

  /* Re-select OnDemand */

  if (config->ondemand)
    {
      regval  = getreg32(SAM_OSCCTRL_XOSCCTRL1);
      regval |= OSCCTRL_XOSCCTRL_ONDEMAND;
      putreg32(regval, SAM_OSCCTRL_XOSCCTRL1);
    }
}
#endif

/****************************************************************************
 * Name: sam_mclk_configure
 *
 * Description:
 *   Configure master clock generator
 *
 ****************************************************************************/

static inline void sam_mclk_configure(uint8_t cpudiv)
{
  putreg8(cpudiv, SAM_MCLK_CPUDIV);
}

/****************************************************************************
 * Name: sam_gclkset_configure
 *
 * Description:
 *   Configure a set of GCLKs
 *
 ****************************************************************************/

static void sam_gclkset_configure(uint16_t gclkset,
                                  const struct sam_gclk_config_s *config)
{
  uint16_t mask;
  int gclk;

  /* Try every GCLK */

  for (gclk = 0; gclk < SAM_NGCLK && gclkset != 0; gclk++)
    {
      /* Check if this one is in the set */

      mask = 1 << gclk;
      if ((gclkset & mask) != 0)
        {
          /* Yes.. Remove it from the set and configure it */

          gclkset &= ~mask;
          sam_gclk_configure(gclk, &config[gclk]);
        }
    }
}

/****************************************************************************
 * Name: sam_dfll_configure, sam_dfll_ready, and sam_dfll_gclkready
 *
 * Description:
 *   Configure the DFLL
 *
 ****************************************************************************/

static void sam_dfll_configure(const struct sam_dfll_config_s *config)
{
  uint32_t regval32;
  uint8_t regval8;

  /* Set GCLK0 source to OSCULP32K (temporarily) */

  regval32  = getreg32(SAM_GCLK_GENCTRL(0));
  regval32 &= ~GCLK_GENCTRL_SRC_MASK;
  regval32 |= GCLK_GENCTRL_SRC_OSCULP32K;
  putreg32(regval32, SAM_GCLK_GENCTRL(0));

  /* Disable the DFLL */

  putreg8(0, SAM_OSCCTRL_DFLLCTRLA);

  /* If we are running in closed loop mode and we are in USB clock recover
   * mode, then set up the input source GCLK channel (unless it has already
   * been configured and the configuration is locked).
   */

  if (config->usbcrm && config->mode &&
      !sam_gclk_chan_locked(GCLK_CHAN_OSCCTRL_DFLL))
    {
      /* Configure the DFLL GCLK channel to use the GCLK source. */

      sam_gclk_chan_enable(GCLK_CHAN_OSCCTRL_DFLL, config->gclk,
                           (bool)config->gclklock);
    }

  /* Setup the DFLLMUL register */

  regval32 = OSCCTRL_DFLLMUL_MUL(config->mul) |
             OSCCTRL_DFLLMUL_FSTEP(config->fstep) |
             OSCCTRL_DFLLMUL_CSTEP(config->cstep);
  putreg32(regval32, SAM_OSCCTRL_DFLLMUL);

  /* Wait until the multiplier is synchronized */

  do
    {
      regval8  = getreg32(SAM_OSCCTRL_DFLLSYNC);
      regval8 &= OSCCTRL_DFLLSYNC_DFLLMUL;
    }
  while (regval8 != 0);

  /* Reset the DFLLCTRLB register */

  putreg32(0, SAM_OSCCTRL_DFLLCTRLB);

  do
    {
      regval8  = getreg8(SAM_OSCCTRL_DFLLSYNC);
      regval8 &= OSCCTRL_DFLLSYNC_DFLLCTRLB;
    }
  while (regval8 != 0);

  /* Set up the DFLLCTRLA register */

  regval8 = OSCCTRL_DPLLCTRLA_ENABLE;

  if (config->runstdby)
    {
      regval8 |= OSCCTRL_DPLLCTRLA_RUNSTDBY;
    }

  putreg8(regval8, SAM_OSCCTRL_DFLLCTRLA);

  do
    {
      regval8  = getreg8(SAM_OSCCTRL_DFLLSYNC);
      regval8 &= OSCCTRL_DPLLCTRLA_ENABLE;
    }
  while (regval8 != 0);

  /* Overwrite factory calibration values is so requested */

  if (config->caliben)
    {
      regval32 = OSCCTRL_DFLLVAL_FINE(config->fcalib) |
                 OSCCTRL_DFLLVAL_COARSE(config->ccalib);
      putreg32(regval32, SAM_OSCCTRL_DFLLVAL);
    }
  else
    {
      regval32 = getreg32(SAM_OSCCTRL_DFLLVAL);
    }

  /* Writing to the DFLLVAL will force it to re-synchronize */

  putreg32(regval32, SAM_OSCCTRL_DFLLVAL);

  do
    {
      regval8  = getreg8(SAM_OSCCTRL_DFLLSYNC);
      regval8 &= OSCCTRL_DFLLSYNC_DFLLVAL;
    }
  while (regval8 != 0);

  /* Setup the DFLLCTRLB register */

  regval8 = 0;

  if (config->mode)
    {
      regval8 |= OSCCTRL_DFLLCTRLB_MODE;
    }

  if (config->stable)
    {
      regval8 |= OSCCTRL_DFLLCTRLB_STABLE;
    }

  if (config->llaw)
    {
      regval8 |= OSCCTRL_DFLLCTRLB_LLAW;
    }

  if (config->usbcrm)
    {
      regval8 |= OSCCTRL_DFLLCTRLB_USBCRM;
    }

  if (config->ccdis)
    {
      regval8 |= OSCCTRL_DFLLCTRLB_CCDIS;
    }

  if (config->qldis)
    {
      regval8 |= OSCCTRL_DFLLCTRLB_QLDIS;
    }

  if (config->bplckc)
    {
      regval8 |= OSCCTRL_DFLLCTRLB_BPLCKC;
    }

  if (config->waitlock)
    {
      regval8 |= OSCCTRL_DFLLCTRLB_WAITLOCK;
    }

  putreg8(regval8, SAM_OSCCTRL_DPLL0CTRLB);

  do
    {
      regval8  = getreg8(SAM_OSCCTRL_DFLLSYNC);
      regval8 &= OSCCTRL_DFLLSYNC_DFLLCTRLB;
    }
  while (regval8 != 0);
}

static void sam_dfll_ready(const struct sam_dfll_config_s *config)
{
  uint32_t ready;
  uint32_t regval32;
  uint8_t regval8;

  /* Check if the mode bit was set, i.e., we are in closed-loop mode. If so
   * wait for the DFLL to be ready for and for the coarse lock to be
   * obtained.
   */

  regval8 = getreg8(SAM_OSCCTRL_DPLL0CTRLB);
  if ((regval8 & OSCCTRL_DFLLCTRLB_MODE) != 0)
    {
      ready = OSCCTRL_INT_DFLLRDY | OSCCTRL_INT_DFLLLCKC;
    }

  /* In open-loop mode, wait only for DFLL ready */

  else
    {
      ready = OSCCTRL_INT_DFLLRDY;
    }

  /* Wait for whichever ready condition */

  do
    {
      regval32  = getreg32(SAM_OSCCTRL_STATUS);
      regval32 &= ready;
    }
  while (regval32 != ready);

  /* Now, we can set the OnDemand bit in the DFLLCTRLA */

  if (config->ondemand)
    {
      regval8  = getreg8(SAM_OSCCTRL_DFLLCTRLA);
      regval8 |= OSCCTRL_DFLLCTRLA_ONDEMAND;
      putreg8(regval8, SAM_OSCCTRL_DFLLCTRLA);
    }
}

static void sam_dfll_gclkready(const struct sam_dfll_config_s *config)
{
  uint32_t regval32;

  /* Wait until all GCLKs are synchronized */

  while (getreg32(SAM_GCLK_SYNCHBUSY) != 0)
    {
    }

  /* Set the source of GCLK0 to to the configured source. */

  regval32  = getreg32(SAM_GCLK_GENCTRL(0));
  regval32 &= ~GCLK_GENCTRL_SRC_MASK;
  regval32 |= GCLK_GENCTRL_SRC(config->gclk);
  putreg32(regval32, SAM_GCLK_GENCTRL(0));
}

/****************************************************************************
 * Name: sam_dpll_configure and sam_dpll_ready
 *
 * Description:
 *   Configure a DPLL, DPLL0 or DPLL1
 *
 ****************************************************************************/

static void sam_dpll_gclkchannel(uint8_t chan,
                                 const struct sam_dpll_config_s *config)
{
  /* Check if we are using a dedicated GCLK as the reference clock.  If so
   * configure GCLK unless it has already been configured and configuration
   * registers have been locked.
   */

  if (config->refclk == 0 && !sam_gclk_chan_locked(chan))
    {
      /* Yes.. configure the GCLK channel that will be used as
       * refclk source.
       */

      sam_gclk_chan_enable(chan, config->gclk, (bool)config->reflock);
    }
}

static void sam_dpll_configure(uintptr_t base,
                               const struct sam_dpll_config_s *config)
{
  uint32_t regval;

  /* Set up the DPLL ratio control register */

  regval = OSCCTRL_DPLLRATIO_LDR(config->ldrint) |
           OSCCTRL_DPLLRATIO_LDRFRAC(config->ldrfrac);
  putreg32(regval, base + SAM_OSCCTRL_DPLLRATIO_OFFSET);

  /* Set up the DPLL control B register */

  regval = OSCCTRL_DPLLCTRLB_FILTER(config->filter) |
           OSCCTRL_DPLLCTRLB_REFLCK(config->refclk) |
           OSCCTRL_DPLLCTRLB_LTIME(config->ltime) |
           OSCCTRL_DPLLCTRLB_DCOFILTER(config->dcofilter) |
           OSCCTRL_DPLLCTRLB_DIV(config->div);

  if (config->wuf)
    {
      regval |= OSCCTRL_DPLLCTRLB_WUF;
    }

  if (config->lbypass)
    {
      regval |= OSCCTRL_DPLLCTRLB_LBYPASS;
    }

  if (config->dcoen)
    {
      regval |= OSCCTRL_DPLLCTRLB_DCOEN;
    }

  putreg32(regval, base + SAM_OSCCTRL_DPLLCTRLB_OFFSET);

  /* Set up the DPLL control A register */

  regval = 0;

  if (config->enable)
    {
      regval |= OSCCTRL_DFLLCTRLA_ENABLE;
    }

  if (config->runstdby)
    {
      regval |= OSCCTRL_DFLLCTRLA_RUNSTDBY;
    }

  putreg32(regval, base + SAM_OSCCTRL_DPLLCTRLA_OFFSET);
}

static void sam_dpll_ready(uintptr_t base,
                           const struct sam_dpll_config_s *config)
{
  uint32_t regval;

  /* If the DPLL was enabled, then wait for it to lock and for the clock to
   * be ready.
   */

  if (config->enable)
    {
      uint32_t lockready = (OSCCTRL_DPLLSTATUS_LOCK |
                            OSCCTRL_DPLLSTATUS_CLKRDY);
      do
        {
          regval  = getreg32(base + SAM_OSCCTRL_DPLLSTATUS_OFFSET);
          regval &= lockready;
        }
      while (regval != lockready);
    }

  /* Finally, set the OnDemand bit if selected */

  if (config->ondemand)
    {
      regval  = getreg32(base + SAM_OSCCTRL_DPLLCTRLA_OFFSET);
      regval |= OSCCTRL_DFLLCTRLA_ONDEMAND;
      putreg32(regval, base + SAM_OSCCTRL_DPLLCTRLA_OFFSET);
    }
}

/****************************************************************************
 * Name: sam_loop_configure
 *
 * Description:
 *   Configure all loops:  DFLL, DPLL0, and DPLL1
 *
 ****************************************************************************/

static void sam_loop_configure(const struct sam_clockconfig_s *config)
{
  /* Configure and enable all loops */

  sam_dfll_configure(&config->dfll);
  sam_dpll_gclkchannel(GCLK_CHAN_OSCCTRL_DPLL0, &config->dpll[0]);
  sam_dpll_configure(SAM_OSCCTRL_DPLL0_BASE, &config->dpll[0]);
  sam_dpll_gclkchannel(GCLK_CHAN_OSCCTRL_DPLL1, &config->dpll[1]);
  sam_dpll_configure(SAM_OSCCTRL_DPLL1_BASE, &config->dpll[1]);

  /* Wait for them to become ready */

  sam_dfll_ready(&config->dfll);
  sam_dpll_ready(SAM_OSCCTRL_DPLL0_BASE, &config->dpll[0]);
  sam_dpll_ready(SAM_OSCCTRL_DPLL1_BASE, &config->dpll[1]);
  sam_dfll_gclkready(&config->dfll);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_clock_configure
 *
 * Description:
 *   Configure the clock sub-system per the provided configuration data.
 *
 *   This should be called only (1) early in the initialization sequence, or
 *   (2) later but within a critical section.
 *
 ****************************************************************************/

void sam_clock_configure(const struct sam_clockconfig_s *config)
{
  uint16_t waitstates;

  /* Check if the number of wait states is increasing or decreasing */

  waitstates = sam_get_waitstates();
  if (config->waitstates > waitstates)
    {
      /* Increasing.  Set the new number of wait states before configuring
       * the clocking.
       */

      sam_set_waitstates(config);
    }

  /* REVISIT:  This function is intended to support both the power-up reset
   * clock configuration as well as the subsequent re-configuration of the
   * clocking.  Logic is here dependent upon the initial clock state.  In
   * order to successfully reconfigure the clocks, I suspect that it will be
   * necessary to always restore the power-up clock configuration here before
   * setting the new clock configuration.
   */

#if BOARD_HAVE_XOSC32K != 0
  /* Configure XOSC32 */

  sam_xosc32k_configure(&config->xosc32k);
#endif

#if BOARD_HAVE_XOSC0 != 0
  /* Configure XOSC0 */

  sam_xosc0_configure(&config->xosc0);
#endif

#if BOARD_HAVE_XOSC1 != 0
  /* Configure XOSC1 */

  sam_xosc1_configure(&config->xosc1);
#endif

  /* Configure master clock generator */

  sam_mclk_configure(config->cpudiv);

  /* Pre-configure some GCLKs before configuring the DPLLs */

  sam_gclkset_configure(config->gclkset1, config->gclk);

  /* Configure loops:  DFLL, DPLL0, and DPLL1. */

  sam_loop_configure(config);

  /* Configure the renaming GCLKs before configuring the DPLLs */

  sam_gclkset_configure(config->gclkset2, config->gclk);

  /* Check if the number of wait states is increasing or decreasing */

  if (config->waitstates < waitstates)
    {
      /* Decreasing.  Set the new number of wait states after configuring
       * the clocking.
       */

      sam_set_waitstates(config);
    }
}

/****************************************************************************
 * Name: sam_clock_initialize
 *
 * Description:
 *   Configure the initial power-up clocking.  This function may be called
 *   only once by the power-up reset logic.
 *
 ****************************************************************************/

void sam_clock_initialize(void)
{
  sam_clock_configure(&g_initial_clocking);
}
