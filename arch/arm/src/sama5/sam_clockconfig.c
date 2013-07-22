/****************************************************************************
 * arch/arm/src/sama5/sam_clockconfig.c
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
#include <debug.h>

#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "up_arch.h"
#include "up_internal.h"

#include "sam_clockconfig.h"
#include "chip/sam_pmc.h"
#include "chip/sam_wdt.h"
#include "chip/sam_matrix.h"

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
 * Name: sam_wdtsetup
 *
 * Description:
 *   Disable the watchdog timer
 *
 ****************************************************************************/

static inline void sam_wdtsetup(void)
{
  putreg32(WDT_MR_WDDIS, SAM_WDT_MR);
}

/****************************************************************************
 * Name: sam_pmcwait
 *
 * Description:
 *   Wait for the specide PMC status bit to become "1"
 *
 ****************************************************************************/

static void sam_pmcwait(uint32_t bit)
{
  /* There is no timeout on this wait.  Why not?  Because the symptoms there
   * is no fallback if the wait times out and if the wait does time out, it
   * can be very difficult to determine what happened.  Much better to just
   * hang here.
   */

  while ((getreg32(SAM_PMC_SR) & bit) == 0);
}

/****************************************************************************
 * Name: sam_enablemosc
 *
 * Description:
 *   Enable the main osciallator
 *
 ****************************************************************************/

static inline void sam_enablemosc(void)
{
  uint32_t regval;

  /* Switch from the internal 12MHz RC to the main external oscillator */

  if ((getreg32(SAM_PMC_CKGR_MOR) & PMC_CKGR_MOR_MOSCSEL) == 0)
    {
      /* Enable main external oscillator */

      regval  = getreg32(SAM_PMC_CKGR_MOR);
      regval |= PMC_CKGR_MOR_MOSCXTEN | PMC_CKGR_MOR_KEY;
      putreg32(regval, SAM_PMC_CKGR_MOR);

      /* Wait for the main clock to become ready  */

      while ((getreg32(SAM_PMC_CKGR_MCFR) & PMC_CKGR_MCFR_MAINFRDY) == 0);

      /* Disable external OSC 12 MHz bypass */

      regval  = getreg32(SAM_PMC_CKGR_MOR);
      regval &= ~PMC_CKGR_MOR_MOSCXTBY;
      regval |= PMC_CKGR_MOR_KEY;
      putreg32(regval, SAM_PMC_CKGR_MOR);

      /* Switch main clock source to the external oscillator */

      regval  = getreg32(SAM_PMC_CKGR_MOR);
      regval |= (PMC_CKGR_MOR_MOSCSEL | PMC_CKGR_MOR_KEY);
      putreg32(regval, SAM_PMC_CKGR_MOR);

      /* Wait for the main clock status change for the external oscillator
       * selection.
       */

      sam_pmcwait(PMC_INT_MOSCSELS);

      /* And handle the case where MCK is running on main CLK */

      sam_pmcwait(PMC_INT_MCKRDY);
    }
}

/****************************************************************************
 * Name: sam_selectmosc
 *
 * Description:
 *   Select the main oscillator as the input clock for processor clock (PCK)
 *   and the main clock (MCK).  The PCK and MCK differ only by the MDIV
 *   divisor that permits the MCK to run at a lower rate.
 *
 ****************************************************************************/

static inline void sam_selectmosc(void)
{
  uint32_t regval;

  /* Select the main oscillator as the input clock for PCK and MCK */

  regval  = getreg32(SAM_PMC_MCKR);
  regval &= ~PMC_MCKR_CSS_MASK;
  regval |= PMC_MCKR_CSS_MAIN;
  putreg32(regval, SAM_PMC_MCKR);

  /* Wait for main clock to be ready */

  sam_pmcwait(PMC_INT_MCKRDY);
}

/****************************************************************************
 * Name: sam_pllasetup
 *
 * Description:
 *   Select the main oscillator as the input clock for processor clock (PCK)
 *   and the main clock (MCK).  The PCK and MCK differ only by the MDIV
 *   divisor that permits the MCK to run at a lower rate.
 *
 ****************************************************************************/

static inline void sam_pllasetup(void)
{
  uint32_t regval;

  /* Configure PLLA */

  regval = (BOARD_CKGR_PLLAR_DIV | BOARD_CKGR_PLLAR_COUNT |
            BOARD_CKGR_PLLAR_OUT | BOARD_CKGR_PLLAR_MUL |
            PMC_CKGR_PLLAR_ONE);
  putreg32(regval, SAM_PMC_CKGR_PLLAR);

  /* Set the PLL Charge Pump Current Register to zero */

  putreg32(0, SAM_PMC_PLLICPR);

  /* And wait for the PLL to lock on */

  sam_pmcwait(PMC_INT_LOCKA);
}

/****************************************************************************
 * Name: sam_plladivider
 *
 * Description:
 *   Configure MCK PLLA divider
 *
 ****************************************************************************/

static inline void sam_plladivider(void)
{
  uint32_t regval;

  /* Is the PLLA divider currently set? */

  regval = getreg32(SAM_PMC_MCKR);
  if ((regval & PMC_MCKR_PLLADIV2) != 0)
    {
#if BOARD_PMC_MCKR_PLLADIV == 0
      /* The divider is set and we are configured to clear it */

      regval &= ~PMC_MCKR_PLLADIV2;
#else
      /* The divider is already set */

      return;
#endif
    }
  else
    {
#if BOARD_PMC_MCKR_PLLADIV == 0
      /* The divider is already cleared */

      return;
#else
      /* The divider is clear and we are configured to set it */

      regval |= PMC_MCKR_PLLADIV2;
#endif
    }

  /* We changed the PLLA divider.  Wait for the main clock to be ready again */

  sam_pmcwait(PMC_INT_MCKRDY);
}

/****************************************************************************
 * Name: sam_mckprescaler
 *
 * Description:
 *   Configure main clock (MCK) Prescaler
 *
 ****************************************************************************/

static inline void sam_mckprescaler(void)
{
  uint32_t regval;

  /* Set the main clock prescaler */

  regval  = getreg32(SAM_PMC_MCKR);
  regval &= ~PMC_MCKR_PRES_MASK;
  regval |= BOARD_PMC_MCKR_PRES;
  putreg32(regval, SAM_PMC_MCKR);

  /* Wait for the main clock to be ready again */

  sam_pmcwait(PMC_INT_MCKRDY);
}

/****************************************************************************
 * Name: sam_mckdivider
 *
 * Description:
 *   Configure main clock (MCK) divider (MDIV).  This divider allows the MCK
 *   to run at a lower rate then PCK.
 *
 ****************************************************************************/

static inline void sam_mckdivider(void)
{
  uint32_t regval;

  /* Set the main clock divider */

  regval  = getreg32(SAM_PMC_MCKR);
  regval &= ~PMC_MCKR_MDIV_MASK;
  regval |= BOARD_PMC_MCKR_MDIV;
  putreg32(regval, SAM_PMC_MCKR);

  /* Wait for the main clock to be ready again */

  sam_pmcwait(PMC_INT_MCKRDY);
}

/****************************************************************************
 * Name: sam_selectplla
 *
 * Description:
 *   Select the PLLA output as the input clock for PCK and MCK.
 *
 ****************************************************************************/

static inline void sam_selectplla(void)
{
  uint32_t regval;

  /* Select the PLLA output as the main clock input */

  regval  = getreg32(SAM_PMC_MCKR);
  regval &= ~PMC_MCKR_CSS_MASK;
  regval |= PMC_MCKR_CSS_PLLA;
  putreg32(regval, SAM_PMC_MCKR);

  /* Wait for the main clock to be ready again */

  sam_pmcwait(PMC_INT_MCKRDY);
}

/****************************************************************************
 * Name: sam_upllsetup
 *
 * Description:
 *   Select the PLLA output as the input clock for PCK and MCK.
 *
 ****************************************************************************/

static inline void sam_upllsetup(void)
{
#ifdef CONFIG_USBDEV
  uint32_t regval;

  /* Setup UTMI for USB and wait for LOCKU */

  regval = getreg32(SAM_PMC_CKGR_UCKR);
  regval |= (BOARD_CKGR_UCKR_UPLLCOUNT | PMC_CKGR_UCKR_UPLLEN);
  putreg32(regval, SAM_PMC_CKGR_UCKR);

  sam_pmcwait(PMC_INT_LOCKU);
#endif
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_clockconfig
 *
 * Description:
 *   Called to initialize the SAM3/4.  This does whatever setup is needed to
 *   put the SoC in a usable state.  This includes the initialization of
 *   clocking using the settings in board.h.  (After power-on reset, the SAM3/4
 *   is initially running on a 12MHz internal RC clock).  This function also
 *   performs other low-level chip initialization of the chip including master
 *   clock, IRQ & watchdog configuration.
 *
 ****************************************************************************/

void sam_clockconfig(void)
{
  /* Configure the watchdog timer */

  sam_wdtsetup();

  /* Initialize clocking */
  /* Enable main oscillator (if it has not already been selected) */

  sam_enablemosc();

  /* Select the main oscillator as the input clock for processor clock (PCK)
   * and the main clock (MCK).  The PCK and MCK differ only by the MDIV
   * divisor that permits the MCK to run at a lower rate.
   */

  sam_selectmosc();

  /* Setup PLLA */

  sam_pllasetup();

  /* Configure the MCK PLLA divider. */

  sam_plladivider();

  /* Configure the MCK Prescaler */

  sam_mckprescaler();

  /* Configure MCK Divider */

  sam_mckdivider();

  /* Finally, elect the PLLA output as the input clock for PCK and MCK. */

  sam_selectplla();

  /* Setup UTMI for USB */

  sam_upllsetup();
}
