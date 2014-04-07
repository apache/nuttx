/****************************************************************************
 * arch/arm/src/tiva/tiva_syscontrol.c
 *
 *   Copyright (C) 2009-2014 Gregory Nutt. All rights reserved.
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

#if defined(LM4F) || defined(TM4C)
#  define RCC_XTALMASK  (SYSCON_RCC_XTAL_MASK | SYSCON_RCC_OSCSRC_MASK | \
                         SYSCON_RCC_PWRDN)
#  define RCC2_XTALMASK (SYSCON_RCC2_OSCSRC2_MASK | SYSCON_RCC2_PWRDN2 | \
                         SYSCON_RCC2_DIV400 | SYSCON_RCC2_USERCC2)
#  define RCC_DIVMASK   (SYSCON_RCC_SYSDIV_MASK | SYSCON_RCC_USESYSDIV | \
                         SYSCON_RCC_MOSCDIS)
#  define RCC2_DIVMASK  (SYSCON_RCC2_SYSDIV2LSB | SYSCON_RCC2_SYSDIV2_MASK)
#else
#  define RCC_OSCMASK   (SYSCON_RCC_IOSCDIS | SYSCON_RCC_MOSCDIS)
#  define RCC_XTALMASK  (SYSCON_RCC_XTAL_MASK | SYSCON_RCC_OSCSRC_MASK | \
                         SYSCON_RCC_PWRDN)
#  define RCC2_XTALMASK (SYSCON_RCC2_OSCSRC2_MASK | SYSCON_RCC2_PWRDN2 | \
                         SYSCON_RCC2_SYSDIV2_MASK | SYSCON_RCC2_USERCC2)
#  define RCC_DIVMASK   (SYSCON_RCC_SYSDIV_MASK | SYSCON_RCC_USESYSDIV | \
                         SYSCON_RCC_IOSCDIS | SYSCON_RCC_MOSCDIS)
#  define RCC2_DIVMASK  (SYSCON_RCC2_SYSDIV2_MASK)
#endif

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
 *   Wait for the newly selected oscillator(s) to settle.  This is tricky because
 *   the time that we wait can be significant and is determined by the previous
 *   clock setting, not the one that we are configuring.
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

  /* Are we currently using RCC2? */

  if ((rcc2 & SYSCON_RCC2_USERCC2) != 0)
    {
      uint32_t rcc2src = rcc2 & SYSCON_RCC2_OSCSRC2_MASK;
      if ((rcc2src == SYSCON_RCC2_OSCSRC2_LFIOSC) ||
          (rcc2src == SYSCON_RCC2_OSCSRC2_32768HZ))
        {
          delay = SLOW_OSCDELAY;
        }
    }

  /* No.. using OSCSRC in RCC */

  else
    {
      uint32_t rccsrc  = rcc  & SYSCON_RCC_OSCSRC_MASK;
      if (rccsrc == SYSCON_RCC_OSCSRC_LFIOSC)
        {
          delay = SLOW_OSCDELAY;
        }
    }

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
 *   Called to change to new clock based on desired rcc and rcc2 settings.
 *   This is use to set up the initial clocking but can be used later to
 *   support slow clocked, low power consumption modes.
 *
 ****************************************************************************/

void tiva_clockconfig(uint32_t newrcc, uint32_t newrcc2)
{
  uint32_t rcc;
  uint32_t rcc2;

  /* Get the current values of the RCC and RCC2 registers */

  rcc  = getreg32(TIVA_SYSCON_RCC);
  rcc2 = getreg32(TIVA_SYSCON_RCC2);

  /* We are probably using the main oscillator.  The main oscillator is
   * disabled on reset and so probably must be enabled here.  The internal
   * oscillator is enabled on reset and if that is selected, most likely
   * nothing needs to be done.
   */

#if defined(LM4F) || defined(TM4C)
  if ((rcc & SYSCON_RCC_MOSCDIS) != 0 && (newrcc & SYSCON_RCC_MOSCDIS) == 0)
    {
      uint32_t dummy;

      /* According to TM4C123GH6PM datasheet page 231 item 5.3 we must perform
       * the following steps to initialize and configure TM4C123G chip to use
       * a PLL based system clock.
       *
       * 1. Bypass the PLL and system clock divider by setting the BYPASS bit
       *    and clearing the USESYS bit in the RCC register.
       *
       * 2. Select the crystal value (XTAL) and oscillator source (OSCSRC),
       *    and clear the PWRDN bit in RCC/RCC2. Setting the XTAL field
       *    automatically pulls valid PLL configuration data for the appropriate
       *    crystal, and clearing the PWRDN bit powers and enables the PLL and
       *    its output.
       *
       * 3. Select the desired system divider (SYSDIV) in RCC/RCC2 and set the
       *    USESYS bit in RCC. The SYSDIV field determines the system
       *    frequency for the microcontroller.
       *
       * 4. Wait for the PLL to lock by polling the PLLLRIS bit in the Raw
       *    Interrupt Status (RIS) register.
       *
       * 5. Enable use of the PLL by clearing the BYPASS bit in RCC/RCC2.
       */

      /* Step 1 - Temporarily bypass the PLL and system clock dividers */

      rcc  |= SYSCON_RCC_BYPASS;
      rcc  &= ~(SYSCON_RCC_USESYSDIV);

      rcc2 |= SYSCON_RCC2_BYPASS2;

      /* According to TM4C123GH6PM datasheet we must write RCC register prior
       * to writing the RCC2 register.
       */

      putreg32(rcc, TIVA_SYSCON_RCC);
      dummy = getreg32(TIVA_SYSCON_RCC);
      UNUSED(dummy);
      putreg32(rcc2, TIVA_SYSCON_RCC2);

      /* Step 2 - Set the new crystal value, oscillator source and PLL
       * configuration.
       */

      rcc  &= ~RCC_XTALMASK;
      rcc  |= (newrcc & RCC_XTALMASK);

      rcc2 &= ~RCC2_XTALMASK;
      rcc2 |= (newrcc2 & RCC2_XTALMASK);

      /* Write the new RCC/RCC2 values.
       *
       * LM4F120 Data Sheet:  "Write the RCC register prior to writing the
       * RCC2 register. If a subsequent write to the RCC register is required,
       * include another register access after writing the RCC register and
       * before writing the RCC2 register."
       */

      putreg32(rcc, TIVA_SYSCON_RCC);
      dummy = getreg32(TIVA_SYSCON_RCC);
      UNUSED(dummy);
      putreg32(rcc2, TIVA_SYSCON_RCC2);

      /* Wait for the new crystal value and oscillator source to take effect */

      tiva_delay(16);

      /* Step 3 - Set the requested system divider and disable the non-
       * selected oscillators.
       */

      rcc  &= ~RCC_DIVMASK;
      rcc  |= (newrcc & RCC_DIVMASK);

      rcc2 &= ~RCC2_DIVMASK;
      rcc2 |= (newrcc2 & RCC2_DIVMASK);

      putreg32(rcc, TIVA_SYSCON_RCC);
      dummy = getreg32(TIVA_SYSCON_RCC);
      UNUSED(dummy);
      putreg32(rcc2, TIVA_SYSCON_RCC2);

      /* Step 4 - Will the PLL output be used to clock the system? */

      if ((newrcc & SYSCON_RCC_BYPASS) == 0)
        {
          /* Yes, wait until l the PLL is locked */

          tiva_pll_lock();

          /* Step 5 - Then enable the PLL */

          rcc  &= ~SYSCON_RCC_BYPASS;
          rcc2 &= ~SYSCON_RCC2_BYPASS2;

          putreg32(rcc, TIVA_SYSCON_RCC);
          dummy = getreg32(TIVA_SYSCON_RCC);
          UNUSED(dummy);
          putreg32(rcc2, TIVA_SYSCON_RCC2);
        }
    }
#else
  if (((rcc & SYSCON_RCC_MOSCDIS) != 0 && (newrcc & SYSCON_RCC_MOSCDIS) == 0) ||
      ((rcc & SYSCON_RCC_IOSCDIS) != 0 && (newrcc & SYSCON_RCC_IOSCDIS) == 0))
    {
      /* Temporarily bypass the PLL and system clock dividers */

      rcc |= SYSCON_RCC_BYPASS;
      rcc &= ~(SYSCON_RCC_USESYSDIV);
      putreg32(rcc, TIVA_SYSCON_RCC);

      rcc2 |= SYSCON_RCC2_BYPASS2;
      putreg32(rcc2, TIVA_SYSCON_RCC2);

      /* Enable any selected oscillators (but don't disable any yet) */

      rcc &= (~RCC_OSCMASK | (newrcc & RCC_OSCMASK));
      putreg32(rcc, TIVA_SYSCON_RCC);

      /* Wait for the newly selected oscillator(s) to settle.  This is tricky because
       * the time that we wait can be significant and is determined by the previous
       * clock setting, not the one that we are configuring.
       */

      tiva_oscdelay(rcc, rcc2);

      /* Set the new crystal value, oscillator source and PLL configuration */

      rcc  &= ~RCC_XTALMASK;
      rcc  |= (newrcc & RCC_XTALMASK);

      rcc2 &= ~RCC2_XTALMASK;
      rcc2 |= (newrcc2 & RCC2_XTALMASK);

      /* Clear the PLL lock interrupt */

      putreg32(SYSCON_MISC_PLLLMIS, TIVA_SYSCON_MISC);

      /* Write the new RCC/RCC2 values.
       *
       * Original LM3S Logic: Order depends upon whether RCC2 or RCC is
       * currently enabled.
       */

      putreg32(rcc, TIVA_SYSCON_RCC);
      putreg32(rcc2, TIVA_SYSCON_RCC2);

      /* Wait for the new crystal value and oscillator source to take effect */

      tiva_delay(16);

      /* Set the requested system divider and disable the non-selected osciallators */

      rcc  &= ~RCC_DIVMASK;
      rcc  |= (newrcc & RCC_DIVMASK);

      rcc2 &= ~RCC2_DIVMASK;
      rcc2 |= (newrcc2 & RCC2_DIVMASK);

      /* Will the PLL output be used to clock the system? */

      if ((newrcc & SYSCON_RCC_BYPASS) == 0)
        {
          /* Yes, wait until the PLL is locked */

          tiva_pll_lock();

          /* Then enable the PLL */

          rcc  &= ~SYSCON_RCC_BYPASS;
          rcc2 &= ~SYSCON_RCC2_BYPASS2;
        }

      /* Now we can set the final RCC/RCC2 values */

      putreg32(rcc, TIVA_SYSCON_RCC);
      putreg32(rcc2, TIVA_SYSCON_RCC2);

      /* Wait for the system divider to be effective */

      tiva_delay(6);
    }
#endif
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
#ifdef CONFIG_LM_REVA2
  /* Some early LM3 silicon returned an increase LDO voltage or 2.75V to work
   * around a PLL bug
   */

  putreg32(SYSCON_LPDOPCTL_2750MV, TIVA_SYSCON_LDOPCTL);
#endif

  /* Set the clocking to run with the default settings provided in the board.h
   * header file
   */

  tiva_clockconfig(TIVA_RCC_VALUE, TIVA_RCC2_VALUE);
}
