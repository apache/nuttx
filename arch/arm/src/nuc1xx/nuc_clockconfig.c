/****************************************************************************
 * arch/arm/src/nuc1xx/nuc_clockconfig.c
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
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"
#include "hardware/nuc_gcr.h"
#include "hardware/nuc_clk.h"

#include "nuc_clockconfig.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nuc_unlock
 *
 * Description:
 *   Unlock registers
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void nuc_unlock(void)
{
  putreg32(0x59, NUC_GCR_REGWRPROT);
  putreg32(0x16, NUC_GCR_REGWRPROT);
  putreg32(0x88, NUC_GCR_REGWRPROT);
}

/****************************************************************************
 * Name: nuclock
 *
 * Description:
 *   Lok registers
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void nuc_lock(void)
{
  putreg32(0, NUC_GCR_REGWRPROT);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nuc_clockconfig
 *
 * Description:
 *   Called to establish the clock settings based on the values in board.h.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nuc_clockconfig(void)
{
  uint32_t regval;

  /* Unlock registers */

  nuc_unlock();

  /* Enable External 4~24 mhz high speed crystal (And other clocks if needed
   * by other drivers).
   */

  regval  = getreg32(NUC_CLK_PWRCON);
  regval |= CLK_PWRCON_XTL12M_EN;
#ifdef CONFIG_NUC_XTALLO
  regval |= CLK_PWRCON_XTL32K_EN;
#endif
#ifdef CONFIG_NUC_INTHI
  regval |= CLK_PWRCON_OSC22M_EN;
#endif
  putreg32(regval, NUC_CLK_PWRCON);

  /* Delay to assure that crystal input to be stable */

  up_mdelay(5);

  /* Set up the PLL configuration.
   *
   * Feedback divider (FB_DV)   = Determined by settings in board.h
   * Input divider (IN_DV)      = Determined by settings in board.h
   * Output divider (OUT_DV)    = Determined by settings in board.h
   * Power down mode (PD)       = Normal mode (0)
   * Bypass (BP)                = Normal mode (0)
   * FOUT enable (OE)           = PLL FOUT enabled (0)
   * PLL source clock (PLL_SRC) = External high speed crystal (0)
   */

  regval = getreg32(NUC_CLK_PLLCON);
  regval &= ~(CLK_PLLCON_FB_DV_MASK | CLK_PLLCON_IN_DV_MASK |
              CLK_PLLCON_OUT_DV_MASK | CLK_PLLCON_PD | CLK_PLLCON_BP |
              CLK_PLLCON_OE | CLK_PLLCON_PLL_SRC);
  regval |=  (CLK_PLLCON_FB_DV(BOARD_PLL_FB_DV) |
              CLK_PLLCON_IN_DV(BOARD_PLL_IN_DV) |
              CLK_PLLCON_OUT_DV(BOARD_PLL_OUT_DV));
  putreg32(regval, NUC_CLK_PLLCON);

  /* Delay until the PLL output is stable */

  up_mdelay(5);

  /* Set the HCLK divider per settings from the board.h file */

  regval = getreg32(NUC_CLK_CLKDIV);
  regval &= ~CLK_CLKDIV_HCLK_N_MASK;
  regval |=  CLK_CLKDIV_HCLK_N(BOARD_HCLK_N);
  putreg32(regval, NUC_CLK_CLKDIV);

  /* Select the PLL output as the HCLKC source */

  regval = getreg32(NUC_CLK_CLKSEL0);
  regval &= ~CLK_CLKSEL0_HCLK_S_MASK;
  regval |= CLK_CLKSEL0_HCLK_S_PLL;
  putreg32(regval, NUC_CLK_CLKSEL0);
  up_mdelay(1);

  /* Lock the registers */

  nuc_lock();
}
