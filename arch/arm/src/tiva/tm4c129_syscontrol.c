/****************************************************************************
 * arch/arm/src/tiva/tm4c129_syscontrol.c
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
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

#include <nuttx/init.h>
#include <arch/board/board.h>

#include "up_arch.h"
#include "up_internal.h"
#include "chip.h"
#include "tiva_syscontrol.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define FAST_OSCDELAY   (512*1024)
#define SLOW_OSCDELAY   (4*1024)
#define PLLLOCK_DELAY   (32*1024)

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tiva_delay
 *
 * Description:
 *   Wait for the newly selected oscillator(s) to settle.  This is tricky
 *   because the time that we wait can be significant and is determined by
 *   the previous clock setting, not the one that we are configuring.
 *
 ****************************************************************************/

static inline void tiva_delay(uint32_t delay)
{
  __asm__ __volatile__("1:\n"
                       "\tsubs  %0, #1\n"
                       "\tbne   1b\n"
                       : "=r"(delay) : "r"(delay));
}

/****************************************************************************
 * Name: tiva_oscdelay
 *
 * Description:
 *   Wait for the newly selected oscillator(s) to settle.  This is tricky because
 *   the time that we wait can be significant and is determined by the previous
 *   clock setting, not the one that we are configuring.
 *
 ****************************************************************************/

static inline void tiva_oscdelay(uint32_t rcc, uint32_t rcc2)
{
  /* Wait for the oscillator  to stabilize.  A smaller delay is used if the
   * current clock rate is very slow.
   */

  uint32_t delay = FAST_OSCDELAY;
#warning Missing logic

  /* Then delay that number of loops */

  tiva_delay(delay);
}

/****************************************************************************
 * Name: tiva_pll_lock
 *
 * Description:
 *   The new RCC values have been selected... wait for the PLL to lock on
 *
 ****************************************************************************/

static inline void tiva_pll_lock(void)
{
  volatile uint32_t delay;

  /* Loop until the lock is achieved or until a timeout occurs */

  for (delay = PLLLOCK_DELAY; delay > 0; delay--)
    {
      /* Check if the PLL is locked on */

      if ((getreg32(TIVA_SYSCON_RIS) & SYSCON_RIS_PLLLRIS) != 0)
        {
          /* Yes.. return now */

          return;
        }
    }

  /* If we get here, then PLL lock was not achieved */
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tiva_clockconfig
 *
 * Description:
 *   Called to change to new clock based on desired pllfreq0, pllfreq1, and
 *   sysdiv settings.  This is use to set up the initial clocking but can be
 *   used later to support slow clocked, low power consumption modes.
 *
 *   The pllfreq0 and pllfreq1 settings derive from the PLL M, N, and Q
 *   values to generate Fvco like:
 *
 *     Fin  = Fxtal / (Q + 1 )(N + 1) -OR- Fpiosc / (Q + 1)(N + 1)
 *     Mdiv = Mint + (MFrac / 1024)
 *     Fvco = Fin * Mdiv
 *
 * When the PLL is active, the system clock frequency (SysClk) is calculated
 * using the following equation:
 *
 *   SysClk = Fvco/ (sysdiv + 1)
 *
 * See the helper macros M2PLLFREQ0(mint,mfrac) and QN2PLLFREQ1(q,n).
 *
 * NOTE: The input clock to the PLL may be either the external crystal
 * (Fxtal) or PIOSC (Fpiosc).  This logic supports only the external
 * crystal as the PLL source clock.
 *
 ****************************************************************************/

void tiva_clockconfig(uint32_t pllfreq0, uint32_t pllfreq1, uint32_t sysdiv)
{
#warning Missing logic
}

/****************************************************************************
 * Name: up_clockconfig
 *
 * Description:
 *   Called early in the boot sequence (before .data and .bss are available)
 *   in order to configure initial clocking.
 *
 ****************************************************************************/

void up_clockconfig(void)
{
  uint32_t pllfreq0;
  uint32_t pllfreq1;

  /* Set the clocking to run with the default settings provided in the board.h
   * header file
   */

  pllfreq0 = M2PLLFREQ0(BOARD_PLL_MINT, BOARD_PLL_MFRAC);
  pllfreq1 = QN2PLLFREQ1(BOARD_PLL_Q, BOARD_PLL_N);
  tiva_clockconfig(pllfreq0, pllfreq1, BOARD_PLL_SYSDIV);
}
