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

#include "barriers.h"

#include "arm64_internal.h"
#include "imx9_ccm.h"
#include "imx9_clockconfig.h"

#include "hardware/imx9_ccm.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define mb()       \
  do               \
    {              \
      ARM64_DSB(); \
      ARM64_ISB(); \
    }              \
  while (0)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint32_t g_pll_freqs[] =
{
  [OSC_24M]          = 24000000U,   /* 24MHZ OSCILLATOR.     */
  [ARM_PLL]          = 2000000000U, /* ARM PLL               */
  [ARM_PLLOUT]       = 2000000000U, /* ARM PLL OUT           */
  [SYS_PLL1_IN]      = 4000000000U, /* SYSTEM PLL1 IN        */
  [SYS_PLL1PFD0_IN]  = 1000000000U, /* SYSTEM PLL1 PFD0 IN   */
  [SYS_PLL1PFD0]     = 1000000000U, /* SYSTEM PLL1 PFD0      */
  [SYS_PLL1PFD0DIV2] = 500000000U,  /* SYSTEM PLL1 PFD0 DIV2 */
  [SYS_PLL1PFD1_IN]  = 800000000U,  /* SYSTEM PLL1 PFD1 IN   */
  [SYS_PLL1PFD1]     = 800000000U,  /* SYSTEM PLL1 PFD1      */
  [SYS_PLL1PFD1DIV2] = 400000000U,  /* SYSTEM PLL1 PFD1 DIV2 */
  [SYS_PLL1PFD2_IN]  = 625000000U,  /* SYSTEM PLL1 PFD2 IN   */
  [SYS_PLL1PFD2]     = 625000000U,  /* SYSTEM PLL1 PFD2      */
  [SYS_PLL1PFD2DIV2] = 312500000U,  /* SYSTEM PLL1 PFD2 DIV2 */
  [AUDIO_PLL1]       = 0U,          /* AUDIO PLL1            */
  [AUDIO_PLL1OUT]    = 0U,          /* AUDIO PLL1 OUT        */
  [DRAM_PLL]         = 1000000000U, /* DRAM PLL              */
  [DRAM_PLLOUT]      = 1000000000U, /* DRAM PLL OUT          */
  [VIDEO_PLL1]       = 0U,          /* VIDEO PLL1            */
  [VIDEO_PLL1OUT]    = 0U,          /* VIDEO PLL1 OUT        */
  [EXT]              = 0U           /* EXT                   */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_IMX9_PLL

#  error "Missing logic"

static int pll_init(uintptr_t reg, bool frac, struct pll_parms *parm)
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

static int pll_pfd_init(uintptr_t reg, int pfd, struct pfd_parms *pfdparm)
{
  uintptr_t ctrl;
  uintptr_t div;
  uint32_t  val;

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
  putreg32(PLL_DFS_CLKOUT_EN | PLL_DFS_ENABLE, PLL_CLR(ctrl));

  /* Set the divider */

  val = PLL_DFS_MFI(pfdparm->mfi) | PLL_DFS_MFN(pfdparm->mfn);
  putreg32(val, PLL_SET(div));

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

  return OK;
}
#endif

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
  /* REVISIT: Define a clock config and run it. For now we rely on the fact
   * that the boot code + bootloader will have set us up.
   *
   * During boot the ROM code initializes the PLL clocks as follows:
   *
   * - OSC24M       : 24   MHz
   * - ARMPLL       : 2000 MHz
   * - ARMPLL_OUT   : 2000 MHz
   * - DRAMPLL      : 1000 MHz
   * - SYSPLL1      : 4000 MHz
   * - SYSPLL_PFD0  : 1000 MHz
   * - SYSPLL_PFD1  : 800  MHz
   * - SYSPLL_PFD2  : 625  MHz
   * - AUDIOPLL     : OFF
   * - AUDIOPLL_OUT : OFF
   * - VIDEOPLL     : OFF
   * - VIDEOPLL_OUT : OFF
   *
   * After reset all clock sources (OSCPLL) and root clocks (CLOCK_ROOT) are
   * running, but gated (LPCG).
   *
   * By default, all peripheral root clocks are set to the 24 MHz oscillator.
   */
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
  if (clkname >= EXT)
    {
      *frequency = 0;
      return -ENODEV;
    }

  *frequency = g_pll_freqs[clkname];
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
