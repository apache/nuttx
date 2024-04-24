/****************************************************************************
 * arch/arm64/src/imx9/imx9_clockconfig.c
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

#include <sys/param.h>
#include <sys/types.h>

#include <arch/board/board.h>

#include "barriers.h"

#include "arm64_internal.h"
#include "imx9_ccm.h"
#include "imx9_clockconfig.h"

#include "hardware/imx9_ccm.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The base oscillator frequency is 24MHz */

#define XTAL_FREQ 24000000u

/* Common barrier */

#define mb()       \
  do               \
    {              \
      ARM64_DSB(); \
      ARM64_ISB(); \
    }              \
  while (0)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_IMX9_BOOTLOADER
static int pll_init(uintptr_t reg, bool frac, struct pll_parms_s *parm)
{
  uint32_t val;

  /* Bypass and disable PLL */

  putreg32(PLL_CTRL_CLKMUX_BYPASS, PLL_SET(PLL_CTRL(reg)));
  putreg32(PLL_CTRL_CLKMUX_EN | PLL_CTRL_POWERUP, PLL_CLR(PLL_CTRL(reg)));

  /* Set the integer dividers */

  val = PLL_DIV_RDIV(parm->rdiv) |
        PLL_DIV_MFI(parm->mfi) |
        PLL_DIV_ODIV(parm->odiv);

  putreg32(val, PLL_DIV(reg));

  /* Disable spread spectrum */

  putreg32(PLL_SPREAD_SPECTRUM_ENABLE, PLL_CLR(PLL_SPREAD_SPECTRUM(reg)));

  /* Set the fractional parts */

  if (frac)
    {
      putreg32(PLL_NUMERATOR_MFN(parm->mfn), PLL_NUMERATOR(reg));
      putreg32(PLL_DENOMINATOR_MFD(parm->mfd), PLL_DENOMINATOR(reg));
    }

  /* Power it back up and wait for lock */

  putreg32(PLL_CTRL_POWERUP, PLL_SET(PLL_CTRL(reg)));
  mb();

  while (!(getreg32(PLL_PLL_STATUS(reg)) & PLL_PLL_STATUS_PLL_LOCK));

  /* Enable PLL and its output */

  putreg32(PLL_CTRL_CLKMUX_EN, PLL_SET(PLL_CTRL(reg)));
  putreg32(PLL_CTRL_CLKMUX_BYPASS, PLL_CLR(PLL_CTRL(reg)));
  mb();

  return OK;
}

static int pll_pfd_init(uintptr_t reg, int pfd, struct pfd_parms_s *pfdparm)
{
  uint32_t ctrl;
  uint32_t div;
  uint32_t val;

  /* Determine the PFD register set */

  switch (pfd)
    {
      case 0:
        ctrl = PLL_DFS_CTRL_0(reg);
        div  = PLL_DFS_DIV_0(reg);
        break;

      case 1:
        ctrl = PLL_DFS_CTRL_1(reg);
        div  = PLL_DFS_DIV_1(reg);
        break;

      case 2:
        ctrl = PLL_DFS_CTRL_2(reg);
        div  = PLL_DFS_DIV_2(reg);
        break;

      default:
        return -EINVAL;
    }

  /* Bypass and disable DFS */

  putreg32(PLL_DFS_BYPASS_EN, PLL_SET(ctrl));
  putreg32(PLL_DFS_CLKOUT_EN | PLL_DFS_CLKOUT_DIVBY2_EN | PLL_DFS_ENABLE,
           PLL_CLR(ctrl));

  /* Set the divider */

  val = PLL_DFS_MFI(pfdparm->mfi) | PLL_DFS_MFN(pfdparm->mfn);
  putreg32(val, PLL_VAL(div));

  /* Enable (or disable) the divby2 output */

  if (pfdparm->divby2_en)
    {
      putreg32(PLL_DFS_CLKOUT_DIVBY2_EN, PLL_SET(ctrl));
    }
  else
    {
      putreg32(PLL_DFS_CLKOUT_DIVBY2_EN, PLL_CLR(ctrl));
    }

  /* Enable DFS and wait for lock */

  putreg32(PLL_DFS_ENABLE, PLL_SET(ctrl));
  mb();

  /* Wait until the clock output is valid */

  while (!(getreg32(PLL_DFS_STATUS(reg)) & (1 << pfd)));

  /* Then disable bypass */

  putreg32(PLL_DFS_BYPASS_EN, PLL_CLR(ctrl));
  mb();

  return OK;
}
#endif

static uint32_t calculate_vco_freq(const struct pll_parms_s *parm, bool frac)
{
  /* Base clock is common for all VCO:s */

  if (frac)
    {
      return (uint64_t)XTAL_FREQ * (parm->mfi * parm->mfd + parm->mfn) /
             parm->mfd / parm->rdiv;
    }
  else
    {
      return (uint64_t)XTAL_FREQ * parm->mfi / parm->rdiv;
    }
}

static uint32_t vco_freq_out(uintptr_t reg, bool frac)
{
  struct pll_parms_s parm;
  uint32_t ctrl;
  uint32_t status;
  uint32_t div;

  /* Check if the PLL on or off */

  ctrl = getreg32(PLL_CTRL(reg));
  if ((ctrl & PLL_CTRL_POWERUP) == 0)
    {
      return 0;
    }

  /* Check if the PLL is stable */

  status = getreg32(PLL_PLL_STATUS(reg));
  if ((status & PLL_PLL_STATUS_PLL_LOCK) == 0)
    {
      return 0;
    }

  /* Populate the integer and fractional PLL parameters */

  div       = getreg32(PLL_DIV(reg));
  parm.rdiv = (div & PLL_DIV_RDIV_MASK) >> PLL_DIV_RDIV_SHIFT;
  parm.mfi  = (div & PLL_DIV_MFI_MASK) >> PLL_DIV_MFI_SHIFT;

  /* RDIV values 0 and 1 both mean a divisor of 1 */

  if (parm.rdiv == 0)
    {
      parm.rdiv = 1;
    }

  if (frac)
    {
      /* Fill the fractional parameters */

      parm.mfn   = getreg32(PLL_NUMERATOR(reg)) & PLL_NUMERATOR_MFN_MASK;
      parm.mfn >>= PLL_NUMERATOR_MFN_SHIFT;
      parm.mfd   = getreg32(PLL_DENOMINATOR(reg)) & PLL_DENOMINATOR_MFD_MASK;
      parm.mfd >>= PLL_DENOMINATOR_MFD_SHIFT;
    }

  return calculate_vco_freq(&parm, frac);
}

static uint32_t pll_freq_out(uintptr_t reg, bool frac)
{
  uint32_t ctrl;
  uint32_t div;
  uint32_t vco;

  /* Read the MUX control register and check if bypass mode is enabled */

  ctrl = getreg32(PLL_CTRL(reg));
  if (ctrl & PLL_CTRL_CLKMUX_BYPASS)
    {
      return XTAL_FREQ;
    }

  /* If the mux is disabled output frequency is 0 */

  if ((ctrl & PLL_CTRL_CLKMUX_EN) == 0)
    {
      return 0;
    }

  /* Get input VCO frequency */

  vco = vco_freq_out(reg, frac);
  if (vco == 0)
    {
      /* The VCO is off or unstable */

      return 0;
    }

  /* Calculate the output clock divider */

  div = (getreg32(PLL_DIV(reg)) & PLL_DIV_ODIV_MASK) >> PLL_DIV_ODIV_SHIFT;

  /* According to spec, div0 = 2 and div1 = 3 */

  if (div == 0)
    {
      div = 2;
    }
  else if (div == 1)
    {
      div = 3;
    }

  return vco / div;
}

static uint32_t pll_pfd_freq_out(uintptr_t reg, int pfd, int div2)
{
  struct pfd_parms_s parm;
  uint32_t ctrl;
  uint32_t div;
  uint32_t vco;

  /* Read the correct PFD register set */

  switch (pfd)
    {
      case 0:
        ctrl = getreg32(PLL_DFS_CTRL_0(reg));
        div  = getreg32(PLL_DFS_DIV_0(reg));
        break;

      case 1:
        ctrl = getreg32(PLL_DFS_CTRL_1(reg));
        div  = getreg32(PLL_DFS_DIV_1(reg));
        break;

      case 2:
        ctrl = getreg32(PLL_DFS_CTRL_2(reg));
        div  = getreg32(PLL_DFS_DIV_2(reg));
        break;

      default:
        return 0;
    }

  /* Get input VCO frequency */

  vco = vco_freq_out(reg, true);
  if (vco == 0)
    {
      /* The VCO is off or unstable */

      return 0;
    }

  /* If the DFS part is bypassed, the output is the VCO directly */

  if (ctrl & PLL_DFS_BYPASS_EN)
    {
      return vco;
    }

  /* Check if the DFS part is disabled */

  if ((ctrl & PLL_DFS_ENABLE) == 0)
    {
      return 0;
    }

  /* Populate the DFS parameters */

  parm.mfi = (div & PLL_DFS_MFI_MASK) >> PLL_DFS_MFI_SHIFT;
  parm.mfn = (div & PLL_DFS_MFN_MASK) >> PLL_DFS_MFN_SHIFT;

  return ((uint64_t)vco * 5) / (parm.mfi * 5 + parm.mfn) / div2;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imx9_clockconfig
 *
 * Description:
 *   Called to initialize the i.IMX9.  This does whatever setup is needed to
 *   put the SoC in a usable state.  This includes the initialization of
 *   clocking using the settings in board.h.
 *
 ****************************************************************************/

void imx9_clockconfig(void)
{
#ifdef CONFIG_IMX9_BOOTLOADER
  struct imx9_pll_cfg_s pll_cfgs[] = PLL_CFGS;
  struct imx9_pfd_cfg_s pfd_cfgs[] = PFD_CFGS;
  struct imx9_pll_cfg_s pll_arm    = ARMPLL_CFG;
  int i;

  /* Set the CPU clock */

  putreg32(IMX9_CCM_GPR_SH_CLR(CCM_SHARED_A55_CLK), CCM_GPR_A55_CLK_SEL_PLL);
  pll_init(pll_arm.reg, pll_arm.frac, &pll_arm.parms);
  putreg32(IMX9_CCM_GPR_SH_SET(CCM_SHARED_A55_CLK), CCM_GPR_A55_CLK_SEL_PLL);

  /* Run the PLL configuration */

  for (i = 0; i < nitems(pll_cfgs); i++)
    {
      struct imx9_pll_cfg_s *cfg = &pll_cfgs[i];
      pll_init(cfg->reg, cfg->frac, &cfg->parms);
    }

  /* Run the PFD configuration */

  for (i = 0; i < nitems(pfd_cfgs); i++)
    {
      struct imx9_pfd_cfg_s *cfg = &pfd_cfgs[i];
      pll_pfd_init(cfg->reg, cfg->pfd, &cfg->parms);
    }
#endif
}

/****************************************************************************
 * Name: imx9_get_clock
 *
 * Description:
 *   This function returns the clock frequency of the specified functional
 *   clock.
 *
 * Input Parameters:
 *   clkname   - Identifies the clock of interest
 *   frequency - The location where the peripheral clock frequency will be
 *              returned
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.  -ENODEV is returned if the clock is not enabled or is not
 *   being clocked.
 *
 ****************************************************************************/

int imx9_get_clock(int clkname, uint32_t *frequency)
{
  switch (clkname)
    {
      case OSC_24M:
        *frequency = XTAL_FREQ;
        break;

      case ARM_PLL:
        *frequency = pll_freq_out(IMX9_ARMPLL_BASE, false);
        break;

      case SYS_PLL1_IN:
        *frequency = pll_freq_out(IMX9_SYSPLL_BASE, false);
        break;

      case SYS_PLL1PFD0:
        *frequency = pll_pfd_freq_out(IMX9_SYSPLL_BASE, 0, 1);
        break;

      case SYS_PLL1PFD0DIV2:
        *frequency = pll_pfd_freq_out(IMX9_SYSPLL_BASE, 0, 2);
        break;

      case SYS_PLL1PFD1:
        *frequency = pll_pfd_freq_out(IMX9_SYSPLL_BASE, 1, 1);
        break;

      case SYS_PLL1PFD1DIV2:
        *frequency = pll_pfd_freq_out(IMX9_SYSPLL_BASE, 1, 2);
        break;

      case SYS_PLL1PFD2:
        *frequency = pll_pfd_freq_out(IMX9_SYSPLL_BASE, 2, 1);
        break;

      case SYS_PLL1PFD2DIV2:
        *frequency = pll_pfd_freq_out(IMX9_SYSPLL_BASE, 2, 2);
        break;

      case AUDIO_PLL1OUT:
        *frequency = pll_freq_out(IMX9_AUDIOPLL_BASE, true);
        break;

      case DRAM_PLLOUT:
        *frequency = pll_freq_out(IMX9_DRAMPLL_BASE, true);
        break;

      case VIDEO_PLL1OUT:
        *frequency = pll_freq_out(IMX9_VIDEOPLL_BASE, true);
        break;

      default:
        return -ENODEV;
    }

  return OK;
}

/****************************************************************************
 * Name: imx9_get_rootclock
 *
 * Description:
 *   This function returns the clock frequency of the specified root
 *   functional clock.
 *
 * Input Parameters:
 *   clkroot   - Identifies the peripheral clock of interest
 *   frequency - The location where the peripheral clock frequency will be
 *              returned
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.  -ENODEV is returned if the clock is not enabled or is not
 *   being clocked.
 *
 ****************************************************************************/

int imx9_get_rootclock(int clkroot, uint32_t *frequency)
{
  uint32_t reg;
  uint32_t div;
  uint32_t mux;
  int clk_name;

  if (clkroot <= CCM_CR_COUNT)
    {
      reg = getreg32(IMX9_CCM_CR_CTRL(clkroot));

      if ((reg & CCM_CR_CTRL_OFF) == CCM_CR_CTRL_OFF)
        {
          *frequency = 0;
        }
      else
        {
          mux = (reg & CCM_CR_CTRL_MUX_MASK) >> CCM_CR_CTRL_MUX_SHIFT;
          clk_name = g_ccm_root_mux[clkroot][mux];
          imx9_get_clock(clk_name, frequency);
          div = ((reg & CCM_CR_CTRL_DIV_MASK) >> CCM_CR_CTRL_DIV_SHIFT) + 1;
          *frequency = *frequency / div;
        }

      return OK;
    }

  return -ENODEV;
}
