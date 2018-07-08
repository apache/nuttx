/****************************************************************************
 * arch/arm/src/lpc54628/lpc54_clockconfig.c
 *
 *   Copyright (C) 2017-2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Parts of this file were adapted from sample code provided for the LPC54xx
 * family from NXP which has a compatible BSD license.
 *
 *   Copyright (c) 2016, Freescale Semiconductor, Inc.
 *   Copyright (c) 2016 - 2017 , NXP
 *   All rights reserved.
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

#include "chip/lpc54_syscon.h"
#include "lpc54_power.h"
#include "lpc54_clockconfig.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc54_setvoltage
 *
 * Description:
 *   Set voltage for PLL frequency.
 *
 ****************************************************************************/

static void lpc54_setvoltage(uint32_t freq)
{
  if (freq == 12000000)
    {
      putreg32(0x21e, 0x40020040);
      putreg32(4, LPC54_SYSCON_PDRUNCFGSET0);
    }
  else if (freq == 48000000)
    {
      putreg32(0x31e, 0x40020040);
      putreg32(4, LPC54_SYSCON_PDRUNCFGSET0);
    }
  else
    {
      putreg32(4, LPC54_SYSCON_PDRUNCFGCLR0);
    }
}

/****************************************************************************
 * Name: lpc54_power_pll
 *
 * Description:
 *   Provide VD3 power to the PLL
 *
 ****************************************************************************/

static void lpc54_power_pll(void)
{
  lpc54_vd3_powerup();
  while ((getreg32(0x40020054) & (1 << 6)) == 0)
    {
    }
}

/****************************************************************************
 * Name: lpc54_set_flash_waitstates
 *
 * Description:
 *   Set the FLASH wait states for the passed frequency
 *
 ****************************************************************************/

static void lpc54_set_flash_waitstates(uint32_t freq)
{
  uint32_t regval;

  regval  = getreg32(LPC54_SYSCON_FLASHCFG);
  regval &= ~SYSCON_FLASHCFG_FLASHTIM_MASK;

  if (freq <= 12000000)
    {
      regval |= SYSCON_FLASHCFG_FLASHTIM(1);
    }
  else if (freq <= 24000000)
    {
      regval |= SYSCON_FLASHCFG_FLASHTIM(2);
    }
  else if (freq <= 36000000)
    {
      regval |= SYSCON_FLASHCFG_FLASHTIM(3);
    }
  else if (freq <= 60000000)
    {
      regval |= SYSCON_FLASHCFG_FLASHTIM(4);
    }
  else if (freq <= 96000000)
    {
      regval |= SYSCON_FLASHCFG_FLASHTIM(5);
    }
  else if (freq <= 120000000)
    {
      regval |= SYSCON_FLASHCFG_FLASHTIM(6);
    }
  else if (freq <= 144000000)
    {
      regval |= SYSCON_FLASHCFG_FLASHTIM(7);
    }
  else if (freq <= 168000000)
    {
      regval |= SYSCON_FLASHCFG_FLASHTIM(8);
    }
  else
    {
      regval |= SYSCON_FLASHCFG_FLASHTIM(9);
    }

  putreg32(regval, LPC54_SYSCON_FLASHCFG);
}

/****************************************************************************
 * Name: lpc54_configure_pll
 *
 * Description:
 *   Configure the PLL.
 *
 *****************************************************************************/

static void lpc54_configure_pll(FAR const struct pll_setup_s *pllsetup)
{
  /* Enable power VD3 for PLLs */

  lpc54_power_pll();

  /* Power off PLL during setup changes */

  lpc54_syspll_powerdown();

  /* Write PLL setup data */

  putreg32(pllsetup->pllctrl, LPC54_SYSCON_SYSPLLCTRL);
  putreg32(pllsetup->pllndec, LPC54_SYSCON_SYSPLLNDEC);
  putreg32(pllsetup->pllndec | SYSCON_SYSPLLNDEC_NREQ,
           LPC54_SYSCON_SYSPLLNDEC);
  putreg32(pllsetup->pllpdec, LPC54_SYSCON_SYSPLLPDEC);
  putreg32(pllsetup->pllpdec | SYSCON_SYSPLLPDEC_PREQ,
           LPC54_SYSCON_SYSPLLPDEC);
  putreg32(pllsetup->pllmdec, LPC54_SYSCON_SYSPLLMDEC);
  putreg32(pllsetup->pllmdec | SYSCON_SYSPLLMDEC_MREQ,
           LPC54_SYSCON_SYSPLLMDEC);

  /* Flags for lock or power on */

  if ((pllsetup->pllflags & (PLL_SETUPFLAG_POWERUP | PLL_SETUPFLAG_WAITLOCK)) != 0)
    {
      /* If turning the PLL back on, perform the following sequence to
       * accelerate PLL lock.
       */

      volatile uint32_t delay;
      uint32_t maxcco = (1 << 18) | 0x5dd2; /* CCO = 1.6Ghz + MDEC enabled */
      uint32_t ssctrl = getreg32(LPC54_SYSCON_SYSPLLMDEC) & ~SYSCON_SYSPLLMDEC_MREQ;

      /* Initialize and power up PLL */

      putreg32(maxcco, LPC54_SYSCON_SYSPLLMDEC);
      lpc54_syspll_powerup();

      /* Set MREQ to activate */

      putreg32(maxcco | SYSCON_SYSPLLMDEC_MREQ, LPC54_SYSCON_SYSPLLMDEC);

      /* Delay for 72 uSec @ 12Mhz */

      for (delay = 0; delay < 172; delay++)
        {
        }

      /* Clear MREQ to prepare for restoring MREQ */

      putreg32(ssctrl, LPC54_SYSCON_SYSPLLMDEC);

      /* set original value back and activate */

      putreg32(ssctrl | SYSCON_SYSPLLMDEC_MREQ, LPC54_SYSCON_SYSPLLMDEC);

      /* Enable PLL */

      lpc54_syspll_powerup();
    }

  /* Wait for the lock? */

  if ((pllsetup->pllflags & PLL_SETUPFLAG_WAITLOCK) != 0)
    {
      while ((getreg32(LPC54_SYSCON_SYSPLLSTAT) & SYSCON_SYSPLLSTAT_LOCK) == 0)
        {
        }
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc54_clockconfig
 *
 * Description:
 *   Called to initialize the LPC54628.  This does whatever setup is needed
 *   to put the MCU in a usable state.  This includes the initialization of
 *   clocking using the settings in board.h.  This function also performs
 *   other low-level chip as necessary.
 *
 *****************************************************************************/

void lpc54_clockconfig(FAR const struct pll_setup_s *pllsetup)
{
  /* Set up the clock sources */
  /* Power up the FRO 12MHz clock source */

  lpc54_fro_powerup();

  /* Switch to FRO 12MHz first to ensure we can change voltage without
   * accidentally being below the voltage for current speed.
   */

  putreg32(SYSCON_MAINCLKSELA_FRO12, LPC54_SYSCON_MAINCLKSELA);
  putreg32(SYSCON_MAINCLKSELB_MAINCLKSELA, LPC54_SYSCON_MAINCLKSELB);

  /* Set the voltage for the find PLL output frequency.  This assumes
   * that the PLL output frequncy is >=12MHz.
   */

  lpc54_setvoltage(pllsetup->pllfout);

  /* Set up the FLASH wait states for the core
   *
   * REVISIT:  Should this be the PLL output frequency (Main clock) or
   * the AHB clock?
   */

  lpc54_set_flash_waitstates(pllsetup->pllfout);

  /* Set up the PLL clock source as specified by PLL configuration. */

  putreg32(pllsetup->pllclksel, LPC54_SYSCON_SYSPLLCLKSEL);

  /* Check if the selected PLL clock source is clk_in, the external clock
   * that comes from a crystal oscillator.
   */

  if (pllsetup->pllclksel == SYSCON_SYSPLLCLKSEL_CLKIN)
    {
      /* If so, provide power for the external clock.
       *
       *   VD2ANA - Analog supply for System Oscillator
       *   SYSOSC - System oscillator power.
       *
       * REVISIT:  This was not necessary for the LPC54628 but kxjiang
       * <kxjiangs@gmail.com> reports this to be necessary for the
       * LPC54608 when the CLKIN source is selected.  This has not been
       * re-verified on the LPC54628.
       *
       * REVISIT:  Note that these power supplies are not disabled if a
       * different clock source is selected.  Should they be?
       */

      putreg32(SYSCON_PDRUNCFG0_VD2ANA, LPC54_SYSCON_PDRUNCFGCLR0);
      putreg32(SYSCON_PDRUNCFG1_SYSOSC, LPC54_SYSCON_PDRUNCFGCLR1);
    }

  /* Configure the PLL */

  lpc54_configure_pll(pllsetup);

  /* Set up the AHB/CPU clock divider */

  putreg32(pllsetup->ahbdiv, LPC54_SYSCON_AHBCLKDIV);

  /* Switch System clock to SYS PLL */

  putreg32(SYSCON_MAINCLKSELB_PLLCLK, LPC54_SYSCON_MAINCLKSELB);
  putreg32(SYSCON_MAINCLKSELA_FRO12, LPC54_SYSCON_MAINCLKSELA);
}
