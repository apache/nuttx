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
