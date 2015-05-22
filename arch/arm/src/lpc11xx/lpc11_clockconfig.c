/****************************************************************************
 * arch/arm/src/lpc11xx/lpc11_clockconfig.c
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
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
#include <debug.h>

#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "up_arch.h"
#include "up_internal.h"
#include "lpc11_clockconfig.h"
#include "chip/lpc11_syscon.h"
#include "chip/lpc111x_iocon.h"
#include "chip/lpc11_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc11_clockconfig
 *
 * Description:
 *   Called to initialize the LPC11xx.  This does whatever setup is needed
 *   to put the SoC in a usable state.  This includes the initialization of
 *   clocking using the settings in board.h.
 *
 ****************************************************************************/

void lpc11_clockconfig(void)
{
  int regval;

  /* Enable the main oscillator (or not) and the frequency range of the main
   * oscillator.
   */

#ifdef CONFIG_LPC11_INTRCOSC
  regval = SYSCON_SYSPLLCLKSEL_IRCOSC;
#else
  regval = SYSCON_SYSPLLCLKSEL_SYSOSC;
#endif
  putreg32(regval, LPC11_SYSCON_SYSPLLCLKSEL);

  /* MSEL = 3 , PSEL = 1 */

  putreg32((SYSCON_SYSPLLCTRL_MSEL_DIV(4) | SYSCON_SYSPLLCTRL_PSEL_DIV2),
           LPC11_SYSCON_SYSPLLCTRL);

  /* Power UP the PLL */

  regval = getreg32(LPC11_SYSCON_PDRUNCFG);
  regval &= ~(SYSCON_PDRUNCFG_SYSPLL_PD);
  putreg32(regval, LPC11_SYSCON_PDRUNCFG);

  /* Inform the core to use PLL as clock */

  putreg32(SYSCON_MAINCLKSEL_PLLOSC, LPC11_SYSCON_SYSPLLCLKUEN);

  /* Use PLL as main clock */

  putreg32(SYSCON_MAINCLKSEL_SYSPLLCLKOUT, LPC11_SYSCON_MAINCLKSEL);

  /* Inform the core of clock update */

  putreg32(SYSCON_MAINCLKUEN_ENA, LPC11_SYSCON_MAINCLKUEN);
}
