/****************************************************************************
 * arch/arm/src/nuc1xx/nuc_clockconfig.c
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>

#include <arch/board/board.h>

#include "up_arch.h"

#include "chip.h"
#include "chip/chip/nuc_gcr.h"
#include "chip/chip/nuc_clk.h"

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

/************************************************************************************
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
 ************************************************************************************/

void nuc_clockconfig(void)
{
  uint32_t regval;

  /* Unlock registers */

  nuc_unlock();

  /* Enable External 4~24 mhz high speed crystal (And other clocks if needed by
   * other drivers).
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
   * PLL srouce clock (PLL_SRC) = External high speed crystal (0)
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
