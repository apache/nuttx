/****************************************************************************
 * arch/arm/src/s32k3xx/s32k3xx_clockconfig.c
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

/* Copyright 2022 NXP */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>

#include "arm_internal.h"

#include "hardware/s32k3xx_mc_me.h"
#include "hardware/s32k3xx_mc_cgm.h"
#include "hardware/s32k3xx_firc.h"
#include "hardware/s32k3xx_sirc.h"
#include "hardware/s32k3xx_fxosc.h"
#include "hardware/s32k3xx_sxosc.h"
#include "hardware/s32k3xx_pll.h"
#include "s32k3xx_pin.h"
#include "hardware/s32k344_pinmux.h"
#include "s32k3xx_periphclocks.h"
#include "s32k3xx_clockconfig.h"

#include <arch/board/board.h>  /* Include last.  May have dependencies */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: s32k3xx_get_scs_clk_source
 *
 * Description:
 *   Gets SCS current system clock source
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   1 equals PHI1 0 equals FIRC
 *
 ****************************************************************************/

static inline uint32_t s32k3xx_get_scs_clk_source(void)
{
  return ((getreg32(S32K3XX_MC_CGM_MUX_0_CSC)
           & MC_CGM_MUX_0_CSC_SELCTL_MASK) ==
           MC_CGM_MUX_CSC_SELCTL_PLL_PHI0_CLK);
}

/****************************************************************************
 * Name: s32k3xx_get_fxoscfreq
 *
 * Description:
 *   Gets FXOSC clock frequency (FXOSC).
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   The FXOSC frequency.  Zero is returned if the FXOSC is not stable.
 *
 ****************************************************************************/

static uint32_t s32k3xx_get_fxoscfreq(void)
{
  /* Check if the FXOSC is stable */

  if ((getreg32(S32K3XX_FXOSC_STAT) & FXOSC_STAT_OSC_STAT_ON) != 0)
    {
      return BOARD_XTAL_FREQUENCY;
    }
  else
    {
      return 0;
    }
}

/****************************************************************************
 * Name: s32k3xx_get_sircfreq
 *
 * Description:
 *   Gets Slow IRC clock frequency (SIRC).
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   The SIRC frequency.  Zero is returned if the SIRC is invalid.
 *
 ****************************************************************************/

static uint32_t s32k3xx_get_sircfreq(void)
{
  /* Check if the SIRC is valid */

  if ((getreg32(S32K3XX_SIRC_SR) & SIRC_SR_STATUS_ON) != 0)
    {
      return CGM_SIRC_FREQUENCY0;
    }

  return 0;
}

/****************************************************************************
 * Name: s32k3xx_get_fircfreq
 *
 * Description:
 *   Gets Fast IRC clock frequency (FIRC).
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   The FIRC frequency.  Zero is returned if the FIRC is invalid.
 *
 ****************************************************************************/

static uint32_t s32k3xx_get_fircfreq(void)
{
  /* Check if the FIRC is valid */

  if ((getreg32(S32K3XX_FIRC_STATUS) & FIRC_STATUS_ON) != 0)
    {
      /* TODO check HSE FIRC DIV register */

      return CGM_FIRC_HIGHRANGE_FREQUENCY;
    }
  else
    {
      return 0;
    }
}

/****************************************************************************
 * Name: s32k3xx_get_pllfreq
 *
 * Description:
 *   Gets PLL clock frequency (SPLL).
 *
 * Input Parameters:
 *   uint32_t id
 *
 * Returned Value:
 *   The PLL frequency.  Zero is returned if the PLL is invalid.
 *
 ****************************************************************************/

static uint32_t s32k3xx_get_pllfreq(void)
{
  uint32_t freq;
  uint32_t regval;
  uint32_t prediv;
  uint32_t postdiv;
  uint32_t mult;

  /* Check if the PLL is valid */

  if ((getreg32(S32K3XX_PLL_SR) & PLL_SR_LOCK) != 0)
    {
      /* Get System Oscillator frequency. */

      freq = s32k3xx_get_fxoscfreq();
      if (freq != 0)
        {
          regval = getreg32(S32K3XX_PLL_DV);
          prediv = ((regval & PLL_DV_RDIV_MASK) >>
                     PLL_DV_RDIV_SHIFT);
          if (prediv == 0)
            {
              prediv = 1;
            }

          mult   = ((regval & PLL_DV_MFI_MASK) >>
                     PLL_DV_MFI_SHIFT);
          postdiv = ((regval & PLL_DV_ODIV2_MASK) >>
                     PLL_DV_ODIV2_SHIFT);
          if (postdiv == 0)
            {
              postdiv = 1;
            }

          freq  /= prediv;
          freq  *= mult;
          freq  /= postdiv;
        }

      return freq;
    }
  else
    {
      return 0;
    }
}

static uint32_t s32k3xx_get_phi0freq(void)
{
  uint32_t regval;
  uint32_t div;

  regval = getreg32(S32K3XX_PLL_ODIV0);

  div = ((regval & PLL_ODIV_DIV_MASK) >> PLL_ODIV_DIV_SHIFT);

  if (div == 0)
    {
      return 0;
    }
  else
    {
      return s32k3xx_get_pllfreq() / (div + 1);
    }
}

/****************************************************************************
 * Name: s32k3xx_get_scsfreq
 *
 * Description:
 *   Gets SCS current system clock frequence
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   1 equals PHI1 0 equals FIRC
 *
 ****************************************************************************/

static inline uint32_t s32k3xx_get_scsfreq(void)
{
  if (s32k3xx_get_scs_clk_source())
    {
      return s32k3xx_get_phi0freq();
    }
  else
    {
      return s32k3xx_get_fxoscfreq();
    }
}

static uint32_t s32k3xx_mux_0_clocktransition(void)
{
  /* Clock transition */

  uint32_t timeout = 1000;
  uint32_t regval;

  putreg32(1, S32K3XX_MC_CGM_MUX_0_DIV_TRIG);

  do
    {
      timeout--;
    }
  while ((getreg32(S32K3XX_MC_CGM_MUX_0_DIV_UPD_STAT)
         & MC_CGM_MUX_DIV_UPD_STAT_DIV_STAT) != 0 && timeout > 0);

  if (timeout == 0)
    {
      return -ETIMEDOUT;
    }

  timeout = 1000;

  do
    {
      timeout--;
    }
  while ((getreg32(S32K3XX_MC_CGM_MUX_0_CSS)
         & MC_CGM_MUX_CSS_SWIP) != 0 && timeout > 0);

  if (timeout == 0)
    {
      return -ETIMEDOUT;
    }

  regval = getreg32(S32K3XX_MC_CGM_MUX_0_CSC);
  regval |= MC_CGM_MUX_CSC_CLK_SW;
  putreg32(regval, S32K3XX_MC_CGM_MUX_0_CSC);

  timeout = 1000;

  do
    {
      timeout--;
    }
  while ((getreg32(S32K3XX_MC_CGM_MUX_0_CSS)
         & MC_CGM_MUX_CSS_CLK_SW) == 0 && timeout > 0);

  if (timeout == 0)
    {
      return -ETIMEDOUT;
    }
  else
    {
      return 0;
    }
}

static uint32_t s32k3xx_mux_x_clocktransition(uint32_t mux_csc_base)
{
  /* Clock transition */

  uint32_t timeout = 1000;
  uint32_t regval;

  do
    {
      timeout--;
    }
  while ((getreg32(mux_csc_base + S32K3XX_MC_CGM_MUX_X_DIV_UPD_STAT_OFFSET)
         & MC_CGM_MUX_DIV_UPD_STAT_DIV_STAT) != 0 && timeout > 0);

  if (timeout == 0)
    {
      return -ETIMEDOUT;
    }

  timeout = 1000;

  do
    {
      timeout--;
    }
  while ((getreg32(mux_csc_base + S32K3XX_MC_CGM_MUX_X_CSS_OFFSET)
         & MC_CGM_MUX_CSS_SWIP) != 0 && timeout > 0);

  if (timeout == 0)
    {
      return -ETIMEDOUT;
    }

  regval = getreg32(mux_csc_base + S32K3XX_MC_CGM_MUX_X_CSC_OFFSET);
  regval |= MC_CGM_MUX_CSC_CLK_SW;
  putreg32(regval, mux_csc_base + S32K3XX_MC_CGM_MUX_X_CSC_OFFSET);

  timeout = 1000;

  do
    {
      timeout--;
    }
  while ((getreg32(mux_csc_base + S32K3XX_MC_CGM_MUX_X_CSS_OFFSET)
         & MC_CGM_MUX_CSS_CLK_SW) == 0 && timeout > 0);

  if (timeout == 0)
    {
      return -ETIMEDOUT;
    }
  else
    {
      return 0;
    }
}

static int s32k3xx_fxosc_peripheral(bool enable)
{
  uint32_t regval;
  uint32_t timeout;

  /* Check if clock is running */

  regval  = getreg32(S32K3XX_MC_ME_PRTN1_COFB1_STAT);

  if ((regval & MC_ME_PRTN1_COFB1_STAT_FXOSC) == 0)
    {
      /* Enable FXOSC pheripheral clock */

      regval  = getreg32(S32K3XX_MC_ME_PRTN1_COFB1_CLKEN);

      if (enable)
        {
          regval |= MC_ME_PRTN1_COFB1_CLKEN_FXOSC;
        }
      else
        {
          regval &= ~MC_ME_PRTN1_COFB1_CLKEN_FXOSC;
        }

      putreg32(regval, S32K3XX_MC_ME_PRTN1_COFB1_CLKEN);

      /* Partition clock enable */

      regval = MC_ME_PRTN_PCONF_PCE;
      putreg32(regval, S32K3XX_MC_ME_PRTN1_PCONF);

      /* Update PRTN1 using Process update register */

      regval = MC_ME_PRTN_PUPD_PCUD;
      putreg32(regval, S32K3XX_MC_ME_PRTN1_PUPD);

      /* Control key register */

      putreg32(MC_ME_CTL_KEY(0x5af0), S32K3XX_MC_ME_CTL_KEY);
      putreg32(MC_ME_CTL_KEY(~0x5af0), S32K3XX_MC_ME_CTL_KEY);

      timeout = 1000;

      do
        {
          timeout--;
        }
      while ((getreg32(S32K3XX_MC_ME_PRTN1_COFB1_STAT)
          & MC_ME_PRTN1_COFB1_STAT_FXOSC) == !enable);

      if (timeout == 0)
        {
          return -ETIMEDOUT;
        }
      else
        {
          return 0;
        }
    }

  else
    {
      return 0;
    }
}

static int s32k3xx_mscm_peripheral(bool enable)
{
  uint32_t regval;
  uint32_t timeout;

  /* Check if clock is running */

  regval  = getreg32(S32K3XX_MC_ME_PRTN1_COFB0_STAT);

  if ((regval & MC_ME_PRTN1_COFB0_STAT_MSCM) == 0)
    {
      /* Enable MSCM pheripheral clock */

      regval  = getreg32(S32K3XX_MC_ME_PRTN1_COFB0_CLKEN);
      if (enable)
        {
          regval |= MC_ME_PRTN1_COFB0_CLKEN_MSCM;
        }
      else
        {
          regval &= ~MC_ME_PRTN1_COFB0_CLKEN_MSCM;
        }

      putreg32(regval, S32K3XX_MC_ME_PRTN1_COFB0_CLKEN);

      /* Partition clock enable */

      putreg32(MC_ME_PRTN_PCONF_PCE, S32K3XX_MC_ME_PRTN1_PCONF);

      /* Update PRTN1 using Process update register */

      putreg32(MC_ME_PRTN_PUPD_PCUD, S32K3XX_MC_ME_PRTN1_PUPD);

      /* Control key register */

      putreg32(MC_ME_CTL_KEY(0x5af0), S32K3XX_MC_ME_CTL_KEY);
      putreg32(MC_ME_CTL_KEY(~0x5af0), S32K3XX_MC_ME_CTL_KEY);

      timeout = 1000;

      do
        {
          timeout--;
        }
      while ((getreg32(S32K3XX_MC_ME_PRTN1_COFB0_STAT)
              & MC_ME_PRTN1_COFB0_STAT_MSCM) == !enable && timeout > 0);

      if (timeout == 0)
        {
          return -ETIMEDOUT;
        }
      else
        {
          return 0;
        }
    }

  return 0;
}

static int s32k3xx_pll_peripheral(bool enable)
{
  uint32_t regval;
  uint32_t timeout;

  /* Check if clock is running */

  regval  = getreg32(S32K3XX_MC_ME_PRTN1_COFB1_STAT);

  if ((regval & MC_ME_PRTN1_COFB1_STAT_PLL) == 0)
    {
      /* Enable FXOSC pheripheral clock */

      regval  = getreg32(S32K3XX_MC_ME_PRTN1_COFB1_CLKEN);
      if (enable)
        {
          regval |= MC_ME_PRTN1_COFB1_CLKEN_PLL;
        }
      else
        {
          regval &= ~MC_ME_PRTN1_COFB1_CLKEN_PLL;
        }

      putreg32(regval, S32K3XX_MC_ME_PRTN1_COFB1_CLKEN);

      /* Partition clock enable */

      putreg32(MC_ME_PRTN_PCONF_PCE, S32K3XX_MC_ME_PRTN1_PCONF);

      /* Update PRTN1 using Process update register */

      putreg32(MC_ME_PRTN_PUPD_PCUD, S32K3XX_MC_ME_PRTN1_PUPD);

      /* Control key register */

      putreg32(MC_ME_CTL_KEY(0x5af0), S32K3XX_MC_ME_CTL_KEY);
      putreg32(MC_ME_CTL_KEY(~0x5af0), S32K3XX_MC_ME_CTL_KEY);

      timeout = 1000;

      do
        {
          timeout--;
        }
      while ((getreg32(S32K3XX_MC_ME_PRTN1_COFB1_STAT)
              & MC_ME_PRTN1_COFB1_STAT_PLL) == !enable && timeout > 0);

      if (timeout == 0)
        {
          return -ETIMEDOUT;
        }
      else
        {
          return 0;
        }
    }

  return 0;
}

/****************************************************************************
 * Name: s32k3xx_pll_config
 *
 * Description:
 *   Configure PLL clocking.
 *
 * Input Parameters:
 *   pllcfg - Describes the new PLL clock configuration
 *
 * Returned Value:
 *   Zero (OK) is returned a success;  A negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

static int s32k3xx_pll_config(const struct cgm_pll_config_s *pllcfg)
{
  uint32_t regval;
  int32_t retval;
  uint32_t timeout;

  /* Enable FXOSC peripheral */

  retval = s32k3xx_fxosc_peripheral(true);

  if (retval < 0)
    {
      return retval;
    }

  /* Enable FXOSC for XTAL */

  regval = getreg32(S32K3XX_FXOSC_CTRL);

  regval &= ~(FXOSC_CTRL_OSC_BYP);

  /* FIXME make EOCV and GM_SEL configurable */

  regval |= (FXOSC_CTRL_COMP_EN | FXOSC_CTRL_EOCV(157) |
             FXOSC_CTRL_GM_SEL_0_7016X | FXOSC_CTRL_OSCON);

  putreg32(regval, S32K3XX_FXOSC_CTRL);

  /* Enable PLL peripheral */

  retval = s32k3xx_pll_peripheral(true);

  if (retval < 0)
    {
      return retval;
    }

  /* Setup PLL Divider */

  regval = (PLL_DV_RDIV_DIV(pllcfg->prediv) | PLL_DV_MFI(pllcfg->mult) |
            PLL_DV_ODIV2_DIV(pllcfg->postdiv));

  putreg32(regval, S32K3XX_PLL_DV);

  /* Setup PLL Fractional Divider */

  regval = getreg32(S32K3XX_PLL_FD);

  regval &= ~(PLL_FD_SDMEN | PLL_FD_SDM2);

  putreg32(regval, S32K3XX_PLL_FD);

  /* Setup PLL Frequency Modulation */

  regval = getreg32(S32K3XX_PLL_FM);

  regval &= ~(PLL_FM_STEPSIZE_MASK | PLL_FM_SPREADCTL |
              PLL_FM_STEPNO_MASK);

  regval |= PLL_FM_SSCGBYP | PLL_FM_SPREADCTL;

  putreg32(regval, S32K3XX_PLL_FM);

  /* Set PLL_PHI0 divider */

  if (pllcfg->phi0 == CGM_PLL_PHI_DIV_DISABLE)
    {
      putreg32(0, S32K3XX_PLL_ODIV0);
    }
  else
    {
      putreg32((PLL_ODIV_DIV(pllcfg->phi0)
               | PLL_ODIV_DE), S32K3XX_PLL_ODIV0);
    }

  /* Set PLL_PHI1 divider */

  if (pllcfg->phi1 == CGM_PLL_PHI_DIV_DISABLE)
    {
      putreg32(0, S32K3XX_PLL_ODIV1);
    }
  else
    {
      putreg32((PLL_ODIV_DIV(pllcfg->phi1)
               | PLL_ODIV_DE), S32K3XX_PLL_ODIV1);
    }

  /* enable PLL */

  putreg32(0, S32K3XX_PLL_CR);

  timeout = 1000;

  do
    {
      timeout--;
    }
  while ((getreg32(S32K3XX_PLL_SR)
         & PLL_SR_LOCK) == 0);

  if (timeout == 0)
    {
      return -ETIMEDOUT;
    }
  else
    {
      return 0;
    }
}

/****************************************************************************
 * Name: s32k3xx_scs_config
 *
 * Description:
 *   Configure SCS clocking.
 *
 * Input Parameters:
 *   scscfg - Describes the new SCS clock configuration
 *
 * Returned Value:
 *   Zero (OK) is returned a success;  A negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

static int s32k3xx_scs_config(const struct cgm_scs_config_s *scscfg)
{
  if (scscfg->scs_source == CGM_SCS_SOURCE_FIRC)
    {
      putreg32(MC_CGM_MUX_CSC_SELCTL_FIRC
               | MC_CGM_MUX_CSC_CLK_SW,
               S32K3XX_MC_CGM_MUX_0_CSC);
    }
  else if (scscfg->scs_source == CGM_SCS_SOURCE_PLL_PHI0)
    {
      putreg32(MC_CGM_MUX_CSC_SELCTL_PLL_PHI0_CLK
               | MC_CGM_MUX_CSC_CLK_SW,
               S32K3XX_MC_CGM_MUX_0_CSC);
    }

  if (scscfg->core_clk.div != CGM_MUX_DISABLE)
    {
      putreg32((MC_CGM_MUX_DC_DE
               | MC_CGM_MUX_DC_DIV(scscfg->core_clk.div)),
               S32K3XX_MC_CGM_MUX_0_DC_0);
    }
  else
    {
      putreg32(0, S32K3XX_MC_CGM_MUX_0_DC_0);
    }

  if (scscfg->aips_plat_clk.div != CGM_MUX_DISABLE)
    {
      putreg32((MC_CGM_MUX_DC_DE
               | MC_CGM_MUX_DC_DIV(scscfg->aips_plat_clk.div)),
               S32K3XX_MC_CGM_MUX_0_DC_1);
    }
  else
    {
      putreg32(0, S32K3XX_MC_CGM_MUX_0_DC_1);
    }

  if (scscfg->aips_slow_clk.div != CGM_MUX_SLOW_DISABLE)
    {
      putreg32((MC_CGM_MUX_DC_DE
               | MC_CGM_MUX_DC_DIV(scscfg->aips_slow_clk.div)),
               S32K3XX_MC_CGM_MUX_0_DC_2);
    }
  else
    {
      putreg32(0, S32K3XX_MC_CGM_MUX_0_DC_2);
    }

  if (scscfg->hse_clk.div != CGM_MUX_DISABLE)
    {
      putreg32((MC_CGM_MUX_DC_DE
               | MC_CGM_MUX_DC_DIV(scscfg->hse_clk.div)),
               S32K3XX_MC_CGM_MUX_0_DC_3);
    }
  else
    {
      putreg32(0, S32K3XX_MC_CGM_MUX_0_DC_3);
    }

  if (scscfg->dcm_clk.div != CGM_MUX_DISABLE)
    {
      putreg32((MC_CGM_MUX_DC_DE
               | MC_CGM_MUX_DC_DIV(scscfg->dcm_clk.div)),
               S32K3XX_MC_CGM_MUX_0_DC_4);
    }
  else
    {
      putreg32(0, S32K3XX_MC_CGM_MUX_0_DC_4);
    }

  if (scscfg->lbist_clk.div != CGM_MUX_DISABLE)
    {
      putreg32((MC_CGM_MUX_DC_DE
               | MC_CGM_MUX_DC_DIV(scscfg->lbist_clk.div)),
               S32K3XX_MC_CGM_MUX_0_DC_5);
    }
  else
    {
      putreg32(0, S32K3XX_MC_CGM_MUX_0_DC_5);
    }

#ifdef CONFIG_S32K3XX_QSPI
  if (scscfg->qspi_mem_clk.div != CGM_MUX_DISABLE)
    {
      putreg32((MC_CGM_MUX_DC_DE
               | MC_CGM_MUX_DC_DIV(scscfg->qspi_mem_clk.div)),
               S32K3XX_MC_CGM_MUX_0_DC_6);
    }
  else
    {
      putreg32(0, S32K3XX_MC_CGM_MUX_0_DC_6);
    }
#endif

  if (scscfg->mux_1_stm0.div == CGM_MUX_DISABLE)
    {
      putreg32(0, S32K3XX_MC_CGM_MUX_1_DC_0);
    }
  else
    {
      putreg32((scscfg->mux_1_stm0.source
               << MC_CGM_MUX_CSS_SELSTAT_SHIFT),
                S32K3XX_MC_CGM_MUX_1_CSC);
      putreg32((MC_CGM_MUX_DC_DIV(scscfg->mux_1_stm0.div)
               | MC_CGM_MUX_DC_DE),
                S32K3XX_MC_CGM_MUX_1_DC_0);
    }

  if (scscfg->mux_3.div == CGM_MUX_DISABLE)
    {
      putreg32(0, S32K3XX_MC_CGM_MUX_3_DC_0);
    }
  else
    {
      putreg32((scscfg->mux_3.source
               << MC_CGM_MUX_CSS_SELSTAT_SHIFT),
                S32K3XX_MC_CGM_MUX_3_CSC);
      putreg32((MC_CGM_MUX_DC_DIV(scscfg->mux_3.div)
               | MC_CGM_MUX_DC_DE),
                S32K3XX_MC_CGM_MUX_3_DC_0);
    }

  if (scscfg->mux_4.div == CGM_MUX_DISABLE)
    {
      putreg32(0, S32K3XX_MC_CGM_MUX_4_DC_0);
    }
  else
    {
      putreg32((scscfg->mux_4.source
                << MC_CGM_MUX_CSS_SELSTAT_SHIFT),
                S32K3XX_MC_CGM_MUX_4_CSC);
      putreg32((MC_CGM_MUX_DC_DIV(scscfg->mux_4.div)
               | MC_CGM_MUX_DC_DE),
                S32K3XX_MC_CGM_MUX_4_DC_0);
    }

#ifdef CONFIG_S32K3XX_ENET
  if (scscfg->mux_7_emac_rx.div == CGM_MUX_DISABLE)
    {
      putreg32(0, S32K3XX_MC_CGM_MUX_7_DC_0);
    }
  else
    {
      putreg32((scscfg->mux_7_emac_rx.source
               << MC_CGM_MUX_CSS_SELSTAT_SHIFT),
                S32K3XX_MC_CGM_MUX_7_CSC);
      putreg32((MC_CGM_MUX_DC_DIV(scscfg->mux_7_emac_rx.div)
               | MC_CGM_MUX_DC_DE),
                S32K3XX_MC_CGM_MUX_7_DC_0);
    }

  if (scscfg->mux_8_emac_tx.div == CGM_MUX_DISABLE)
    {
      putreg32(0, S32K3XX_MC_CGM_MUX_8_DC_0);
    }
  else
    {
      putreg32((scscfg->mux_8_emac_tx.source
               << MC_CGM_MUX_CSS_SELSTAT_SHIFT),
                S32K3XX_MC_CGM_MUX_8_CSC);
      putreg32((MC_CGM_MUX_DC_DIV(scscfg->mux_8_emac_tx.div)
               | MC_CGM_MUX_DC_DE),
                S32K3XX_MC_CGM_MUX_8_DC_0);
    }

  if (scscfg->mux_9_emac_ts.div == CGM_MUX_DISABLE)
    {
      putreg32(0, S32K3XX_MC_CGM_MUX_9_DC_0);
    }
  else
    {
      putreg32((scscfg->mux_9_emac_ts.source
               << MC_CGM_MUX_CSS_SELSTAT_SHIFT),
                S32K3XX_MC_CGM_MUX_9_CSC);
      putreg32((MC_CGM_MUX_DC_DIV(scscfg->mux_9_emac_ts.div)
               | MC_CGM_MUX_DC_DE),
                S32K3XX_MC_CGM_MUX_9_DC_0);
    }
#endif

#ifdef CONFIG_S32K3XX_QSPI
  if (scscfg->mux_10_qspi_sfck.div == CGM_MUX_DISABLE)
    {
      putreg32(0, S32K3XX_MC_CGM_MUX_10_DC_0);
    }
  else
    {
      putreg32((scscfg->mux_10_qspi_sfck.source
               << MC_CGM_MUX_CSS_SELSTAT_SHIFT),
                S32K3XX_MC_CGM_MUX_10_CSC);
      putreg32((MC_CGM_MUX_DC_DIV(scscfg->mux_10_qspi_sfck.div)
                | MC_CGM_MUX_DC_DE),
                S32K3XX_MC_CGM_MUX_10_DC_0);
    }
#endif

  return 0;
}

static void s32k3xx_clkout_config(const struct
                                  cgm_clkout_config_s *clkoutcfg)
{
  if (clkoutcfg->div == CGM_CLKOUT_DIV_DISABLE)
    {
      putreg32(0, S32K3XX_MC_CGM_MUX_6_DC_0);
    }
  else
    {
      putreg32((clkoutcfg->source << MC_CGM_MUX_CSS_SELSTAT_SHIFT),
                S32K3XX_MC_CGM_MUX_6_CSC);
      putreg32((MC_CGM_MUX_DC_DIV(clkoutcfg->div) | MC_CGM_MUX_DC_DE),
                S32K3XX_MC_CGM_MUX_6_DC_0);
    }
}

/****************************************************************************
 * Name: s32k1xx_scg_config
 *
 * Description:
 *   Configure CGM clocking.
 *
 * Input Parameters:
 *   cgmcfg - Describes the new CGM clock configuration
 *
 * Returned Value:
 *   Zero (OK) is returned a success;  A negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

static int s32k3xx_cgm_config(const struct cgm_config_s *cgmcfg)
{
  int ret;

  /* Enable MSCM peripheral */

  s32k3xx_mscm_peripheral(true);

  /* Enable PLL peripheral */

  s32k3xx_pll_peripheral(true);

  /* Configure MC_CGM_MUX0 to run from FIRC clock to allow PLL reconfig. */

  putreg32((MC_CGM_MUX_0_DIV_TRIG_CTRL_TCTL |
            MC_CGM_MUX_0_DIV_TRIG_CTRL_HHEN),
              S32K3XX_MC_CGM_MUX_0_DIV_TRIG_CTRL);
  putreg32((MC_CGM_MUX_DC_DIV(0) | MC_CGM_MUX_DC_DE),
              S32K3XX_MC_CGM_MUX_0_DC_0);
  putreg32((MC_CGM_MUX_DC_DIV(0) | MC_CGM_MUX_DC_DE),
              S32K3XX_MC_CGM_MUX_0_DC_1);
  putreg32((MC_CGM_MUX_DC_DIV(2) | MC_CGM_MUX_DC_DE),
              S32K3XX_MC_CGM_MUX_0_DC_2);
  putreg32((MC_CGM_MUX_DC_DIV(0) | MC_CGM_MUX_DC_DE),
              S32K3XX_MC_CGM_MUX_0_DC_3);
  putreg32((MC_CGM_MUX_DC_DIV(0) | MC_CGM_MUX_DC_DE),
              S32K3XX_MC_CGM_MUX_0_DC_4);
  putreg32((MC_CGM_MUX_DC_DIV(0) | MC_CGM_MUX_DC_DE),
              S32K3XX_MC_CGM_MUX_0_DC_5);
  putreg32((MC_CGM_MUX_DC_DIV(0) | MC_CGM_MUX_DC_DE),
              S32K3XX_MC_CGM_MUX_0_DC_6);

  /* Transition to FIRC */

  ret = s32k3xx_mux_0_clocktransition();

  if (ret >= 0)
    {
      ret = s32k3xx_pll_config(&cgmcfg->pll);
    }

  if (ret >= 0)
    {
      ret = s32k3xx_scs_config(&cgmcfg->scs);
      if (ret >= 0)
        {
          /* Transition to PLL */

          ret = s32k3xx_mux_0_clocktransition();
        }
    }

  if (ret >= 0)
    {
      ret = s32k3xx_mux_x_clocktransition(S32K3XX_MC_CGM_MUX_1_CSC);
    }

  if (ret >= 0)
    {
      ret = s32k3xx_mux_x_clocktransition(S32K3XX_MC_CGM_MUX_3_CSC);
    }

  if (ret >= 0)
    {
      ret = s32k3xx_mux_x_clocktransition(S32K3XX_MC_CGM_MUX_4_CSC);
    }

#ifdef CONFIG_S32K3XX_ENET
  if (ret >= 0)
    {
#  ifdef PIN_EMAC_MII_RMII_TX_CLK
    s32k3xx_pinconfig(PIN_EMAC_MII_RMII_TX_CLK);
#  endif
    ret = s32k3xx_mux_x_clocktransition(S32K3XX_MC_CGM_MUX_7_CSC);
    ret = s32k3xx_mux_x_clocktransition(S32K3XX_MC_CGM_MUX_8_CSC);
    ret = s32k3xx_mux_x_clocktransition(S32K3XX_MC_CGM_MUX_9_CSC);
    }
#endif

#ifdef CONFIG_S32K3XX_QSPI
  if (ret >= 0)
    {
      s32k3xx_mux_x_clocktransition(S32K3XX_MC_CGM_MUX_10_CSC);
    }
#endif

  s32k3xx_clkout_config(&cgmcfg->clkout);

  return ret;
}

/****************************************************************************
 * Name: s32k3xx_clockconfig
 *
 * Description:
 *   Called to initialize the S32K3XX.  This does whatever setup is needed
 *   to put the MCU in a usable state.  This includes the initialization of
 *   clocking using the settings in board.h.  This function also performs
 *   other low-level chip as necessary.
 *
 * Input Parameters:
 *   clkcfg - Describes the new clock configuration
 *
 * Returned Value:
 *   Zero (OK) is returned a success;  A negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

int s32k3xx_clockconfig(const struct clock_configuration_s *clkcfg)
{
  int ret;

  ret = s32k3xx_cgm_config(&clkcfg->cgm);
  if (ret >= 0)
    {
        s32k3xx_periphclocks(clkcfg->pcc.count, clkcfg->pcc.pclks);
    }

  return 0;
}

/****************************************************************************
 * Name: s32k3xx_get_freq
 *
 * Description:
 *    clock frequency from a clock source.
 *
 * Input Parameters:
 *   clksrc - The requested clock source.
 *
 * Returned Value:
 *   The frequency of the requested clock source.
 *
 ****************************************************************************/

uint32_t s32k3xx_get_freq(enum clock_names_e clksrc)
{
  uint32_t regval;

  switch (clksrc)
    {
      case CORE_CLK:
        {
          regval = getreg32(S32K3XX_MC_CGM_MUX_0_DC_0);
          if ((regval & MC_CGM_MUX_DC_DE) == 0)
            {
              return 0;
            }
          else
            {
              regval = ((regval & MC_CGM_MUX_DC_DIV_MASK)
                        >> MC_CGM_MUX_DC_DIV_SHIFT);
              return s32k3xx_get_scsfreq() / (regval + 1);
            }
        }
        break;

      case SIRC_CLK:
          return s32k3xx_get_sircfreq();

      case FIRC_CLK:
          return s32k3xx_get_fircfreq();

      case AIPS_PLAT_CLK:
        {
          regval = getreg32(S32K3XX_MC_CGM_MUX_0_DC_1);
          if ((regval & MC_CGM_MUX_DC_DE) == 0)
            {
              return 0;
            }
          else
            {
              regval = ((regval & MC_CGM_MUX_DC_DIV_MASK)
                        >> MC_CGM_MUX_DC_DIV_SHIFT);
              return s32k3xx_get_scsfreq() / (regval + 1);
            }
        }
        break;

      case AIPS_SLOW_CLK:
        {
          regval = getreg32(S32K3XX_MC_CGM_MUX_0_DC_2);
          if ((regval & MC_CGM_MUX_DC_DE) == 0)
            {
              return 0;
            }
          else
            {
              regval = ((regval & MC_CGM_MUX_DC_DIV_MASK)
                        >> MC_CGM_MUX_DC_DIV_SHIFT);
              return s32k3xx_get_scsfreq() / (regval + 1);
            }
        }
        break;

      case STM0_CLK:
        {
          regval = getreg32(S32K3XX_MC_CGM_MUX_1_DC_0);
          if ((regval & MC_CGM_MUX_DC_DE) == 0)
            {
              return 0;
            }
          else
            {
              switch (getreg32(S32K3XX_MC_CGM_MUX_1_CSC))
                {
                  case (CGM_CLK_SRC_AIPS_PLAT_CLK
                        << MC_CGM_MUX_CSS_SELSTAT_SHIFT):
                    regval = ((regval & MC_CGM_MUX_DC_DIV_MASK)
                              >> MC_CGM_MUX_DC_DIV_SHIFT);
                    return s32k3xx_get_freq(AIPS_PLAT_CLK) / (regval + 1);

                  case (CGM_CLK_SRC_FXOSC << MC_CGM_MUX_CSS_SELSTAT_SHIFT):
                    regval = ((regval & MC_CGM_MUX_DC_DIV_MASK)
                              >> MC_CGM_MUX_DC_DIV_SHIFT);
                    return s32k3xx_get_freq(FXOSC_CLK) / (regval + 1);

                  case (CGM_CLK_SRC_FIRC << MC_CGM_MUX_CSS_SELSTAT_SHIFT):
                    regval = ((regval & MC_CGM_MUX_DC_DIV_MASK)
                              >> MC_CGM_MUX_DC_DIV_SHIFT);
                    return s32k3xx_get_freq(FIRC_CLK) / (regval + 1);

                  default:
                      return 0;
                }
            }
        }
        break;

      case FLEXCAN0_CLK:
      case FLEXCAN1_CLK:
      case FLEXCAN2_CLK:
        {
          regval = getreg32(S32K3XX_MC_CGM_MUX_3_DC_0);
          if ((regval & MC_CGM_MUX_DC_DE) == 0)
            {
              return 0;
            }
          else
            {
              regval = ((regval & MC_CGM_MUX_DC_DIV_MASK)
                        >> MC_CGM_MUX_DC_DIV_SHIFT);
              return s32k3xx_get_freq(AIPS_PLAT_CLK) / (regval + 1);
            }
        }
        break;

      case FLEXCAN3_CLK:
      case FLEXCAN4_CLK:
      case FLEXCAN5_CLK:
        {
          regval = getreg32(S32K3XX_MC_CGM_MUX_4_DC_0);
          if ((regval & MC_CGM_MUX_DC_DE) == 0)
            {
              return 0;
            }
          else
            {
              regval = ((regval & MC_CGM_MUX_DC_DIV_MASK)
                        >> MC_CGM_MUX_DC_DIV_SHIFT);
              return s32k3xx_get_freq(AIPS_PLAT_CLK) / (regval + 1);
            }
        }
        break;

       default:
        {
          /* Invalid clock source type */

          DEBUGPANIC();
          return 0;
        }
        break;
    }
}

/****************************************************************************
 * Name: s32k3xx_get_coreclk
 *
 * Description:
 *   Return the current value of the CORE clock frequency.
 *
 * Input Parameters:
 *   None
 *
 * Returned Values:
 *   The current value of the CORE clock frequency.  Zero is returned on any
 *   failure.
 *
 ****************************************************************************/

uint32_t s32k3xx_get_coreclk(void)
{
  return s32k3xx_get_freq(CORE_CLK);
}
