/****************************************************************************
 * arch/arm/src/lpc31xx/lpc31_pllconfig.c
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

/* References:
 *   - NXP UM10314 LPC3130/31 User manual Rev. 1.01 - 9 September 2009
 *   - NXP lpc313x.cdl.drivers.zip example driver code
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <arch/board/board.h>

#include "lpc31_cgudrvr.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc31_switchdomains
 *
 * Description:
 *   Temporarily switch the reference clock of all domains whose selected
 *   input is the PLL-to-be configured .
 *
 ****************************************************************************/

static inline uint16_t
lpc31_switchdomains(const struct lpc31_pllconfig_s * const cfg)
{
  uint32_t hppll  = (cfg->hppll ? CGU_SSR_HPPLL1 : CGU_SSR_HPPLL0);
  uint32_t address;
  uint32_t regval;
  uint16_t dmnset = 0;
  int    i;

  /* Check each domain */

  for (i = 0; i < CGU_NDOMAINS; i++)
    {
      /* Get the switch status registers (SSR) for this frequency input
       * domain
       */

      address = LPC31_CGU_SSR(i);
      regval  = getreg32(address);

      /* Check if the current frequency selection is the
       * PLL-to-be-configured
       */

      if ((regval & CGU_SSR_FS_MASK) == hppll)
        {
          /* Yes.. switch reference clock in to FFAST */

          lpc31_selectfreqin((enum lpc31_domainid_e)i, CGU_FS_FFAST);

          /* Add the domain to the set to be restored after the PLL is
           * configured
           */

          dmnset |= (1 << i);
        }
    }

  return dmnset;
}

/****************************************************************************
 * Name: lpc31_restoredomains
 *
 * Description:
 *   Restore the PLL reference clock to the domains that were temporarily
 *   switched to FFAST by lpc31_switchdomains.
 *
 ****************************************************************************/

static inline void
lpc31_restoredomains(const struct lpc31_pllconfig_s * const cfg,
                       uint16_t dmnset)
{
  uint32_t finsel = (cfg->hppll ? CGU_FS_HPPLL1 : CGU_FS_HPPLL0);
  int    i;

  /* Check each domain */

  for (i = 0; i < CGU_NDOMAINS; i++)
    {
      /* Was this one switched? */

      if ((dmnset & (1 << i)) != 0)
        {
          /* Switch input reference clock to newly configured HPLL */

          lpc31_selectfreqin((enum lpc31_domainid_e)i, finsel);
        }
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc31_pllconfig
 *
 * Description:
 *   Configure the PLL according to the provided selections.
 *
 ****************************************************************************/

void lpc31_pllconfig(const struct lpc31_pllconfig_s * const cfg)
{
  uint32_t pllbase;
  uint16_t dmnset = 0;

  /* Switch domains connected to HPLL to FFAST */

  dmnset = lpc31_switchdomains(cfg);

  /* Get the PLL register base address */

  pllbase = LPC313x_CGU_HPPLL(cfg->hppll);

  /* Disable clock, disable skew enable, power down pll, (dis/en)able post
   * divider, (dis/en)able pre-divider, disable free running mode, disable
   * bandsel, enable up limmiter, disable bypass
   */

  putreg32(CGU_HPMODE_PD, pllbase + LPC31_CGU_HPMODE_OFFSET);

  /* Select PLL input frequency source */

  putreg32(cfg->finsel, pllbase + LPC31_CGU_HPFINSEL_OFFSET);

  /* Set M divider */

  putreg32(cfg->mdec & CGU_HPMDEC_MASK, pllbase + LPC31_CGU_HPMDEC_OFFSET);

  /* Set N divider */

  putreg32(cfg->ndec & CGU_HPNDEC_MASK, pllbase + LPC31_CGU_HPNDEC_OFFSET);

  /* Set P divider */

  putreg32(cfg->pdec & CGU_HPPDEC_MASK, pllbase + LPC31_CGU_HPPDEC_OFFSET);

  /* Set bandwidth */

  putreg32(cfg->selr, pllbase + LPC31_CGU_HPSELR_OFFSET);
  putreg32(cfg->seli, pllbase + LPC31_CGU_HPSELI_OFFSET);
  putreg32(cfg->selp, pllbase + LPC31_CGU_HPSELP_OFFSET);

  /* Power up pll */

  putreg32((cfg->mode & ~CGU_HPMODE_PD) | CGU_HPMODE_CLKEN,
            pllbase + LPC31_CGU_HPMODE_OFFSET);

  /* Save the estimated freq in driver data for future clk calcs */

  g_boardfreqin[CGU_FREQIN_HPPLL0 + cfg->hppll] = cfg->freq;

  /* Wait for PLL to lock */

  while ((getreg32(pllbase + LPC31_CGU_HPSTATUS_OFFSET) &
                   CGU_HPSTATUS_LOCK) == 0);

  /* Switch the domains that were temporarily switched to FFAST back to
   * the HPPLL
   */

  lpc31_restoredomains(cfg, dmnset);
}

/****************************************************************************
 * Name: lpc31_hp0pllconfig
 *
 * Description:
 *   Configure the HP0 PLL according to the board.h selections.
 *
 ****************************************************************************/

void lpc31_hp0pllconfig(void)
{
  struct lpc31_pllconfig_s cfg =
  {
    .hppll  = CGU_HP0PLL,
    .finsel = BOARD_HPLL0_FINSEL,
    .ndec   = BOARD_HPLL0_NDEC,
    .mdec   = BOARD_HPLL0_MDEC,
    .pdec   = BOARD_HPLL0_PDEC,
    .selr   = BOARD_HPLL0_SELR,
    .seli   = BOARD_HPLL0_SELI,
    .selp   = BOARD_HPLL0_SELP,
    .mode   = BOARD_HPLL0_MODE,
    .freq   = BOARD_HPLL0_FREQ
  };

  lpc31_pllconfig(&cfg);
}

/****************************************************************************
 * Name: lpc31_hp1pllconfig
 *
 * Description:
 *   Configure the HP1 PLL according to the board.h selections.
 *
 ****************************************************************************/

void lpc31_hp1pllconfig(void)
{
  struct lpc31_pllconfig_s cfg =
  {
    .hppll  = CGU_HP1PLL,
    .finsel = BOARD_HPLL1_FINSEL,
    .ndec   = BOARD_HPLL1_NDEC,
    .mdec   = BOARD_HPLL1_MDEC,
    .pdec   = BOARD_HPLL1_PDEC,
    .selr   = BOARD_HPLL1_SELR,
    .seli   = BOARD_HPLL1_SELI,
    .selp   = BOARD_HPLL1_SELP,
    .mode   = BOARD_HPLL1_MODE,
    .freq   = BOARD_HPLL1_FREQ
  };

  lpc31_pllconfig(&cfg);
}
