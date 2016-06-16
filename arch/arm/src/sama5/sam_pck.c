/****************************************************************************
 * arch/arm/src/sama5/sam_pck.c
 *
 *   Copyright (C) 2013-2014 Gregory Nutt. All rights reserved.
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

#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <arch/board/board.h>

#include "chip/sam_pinmap.h"

#include "up_arch.h"
#include "sam_pio.h"
#include "sam_isi.h"

#include "sam_pck.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: sam_pck_configure
 *
 * Description:
 *   Configure a programmable clock output.  The selected PCK is programmed
 *   to the selected frequency using either PLLA or the MCK as the source
 *   clock (depending on the value of the selected frequency).  The clock
 *   is initially disabled.  You must call sam_pck_enable() to enable the
 *   clock after it has been configured.
 *
 * Input Parameters:
 *   pckid - Identifies the programmable clock output (0, 1, or 2)
 *   clocksrc - MCK or SCK.  If MCK is selected, the logic will automatically
 *     select the PLLACK clock if it seems like a better choice.
 *   frequency - Defines the desired frequency.  The exact frequency may
 *     not be attainable.  In this case, frequency is interpreted to be
 *     a not-to-exceed frequency.
 *
 * Returned Value:
 *   The actual frequency of the clock output.
 *
 ****************************************************************************/

uint32_t sam_pck_configure(enum pckid_e pckid, enum pckid_clksrc_e clksrc,
                           uint32_t frequency)
{
  uint32_t regval;
  uint32_t clkin;
  uint32_t actual;
#ifdef SAMA5_HAVE_PCK_INT_PRES
  uint32_t pres;
#endif

  /* Pick a clock source.  Several are possible but only MCK, PLLA, the
   * MAINCK,or SCK are supported here.
   */

   switch (clksrc)
    {
    case PCKSRC_MCK: /* Source clock = MCK or PLLACK */
      {
        /* Pick either the MCK or the PLLACK, whichever will best realize
         * the target frequency.
         */

        DEBUGASSERT(BOARD_MCK_FREQUENCY < BOARD_PLLA_FREQUENCY);

        /* Pick the PLLACK if it seems like a better choice */

        if (frequency <= BOARD_MCK_FREQUENCY ||
            frequency < BOARD_PLLA_FREQUENCY / 64)
          {
            regval = PMC_PCK_CSS_MCK;
            clkin  = BOARD_MCK_FREQUENCY;
          }
        else
          {
            regval = PMC_PCK_CSS_PLLA;
            clkin  = BOARD_PLLA_FREQUENCY;
          }
      }
      break;

    case PCKSRC_MAINCK: /* Source clock = MAIN clock */
      regval = PMC_PCK_CSS_MAIN;
      clkin  = BOARD_MAINCK_FREQUENCY;
      break;

    case PCKSRC_SCK: /* Source clock = SCK */
      regval = PMC_PCK_CSS_SLOW;
      clkin  = BOARD_SLOWCLK_FREQUENCY;
      break;

    default:
      _err("ERROR: Unknown clock source\n");
      return 0;
    }

#ifdef SAMA5_HAVE_PCK_INT_PRES
  /* Programmable Clock frequency is selected clock freqency divided by PRES + 1 */

  pres = clkin / frequency;
  if (pres < 1)
    {
      pres = 1;
    }
  else if (pres > 256)
    {
      pres = 256;
    }

  regval |= PMC_PCK_PRES(pres - 1);
  actual  = clkin / pres;

#else
  /* The the larger smallest divisor that does not exceed the requested
   * frequency.
   */

  if (frequency >= clkin)
    {
      regval |= PMC_PCK_PRES_DIV1;
      actual  = clkin;
    }
  else if (frequency >= (clkin >> 1))
    {
      regval |= PMC_PCK_PRES_DIV2;
      actual  = clkin >> 1;
    }
  else if (frequency >= (clkin >> 2))
    {
      regval |= PMC_PCK_PRES_DIV4;
      actual  = clkin >> 2;
    }
  else if (frequency >= (clkin >> 3))
    {
      regval |= PMC_PCK_PRES_DIV8;
      actual  = clkin >> 3;
    }
  else if (frequency >= (clkin >> 4))
    {
      regval |= PMC_PCK_PRES_DIV16;
      actual  = clkin >> 4;
    }
  else if (frequency >= (clkin >> 5))
    {
      regval |= PMC_PCK_PRES_DIV32;
      actual  = clkin >> 5;
    }
  else if (frequency >= (clkin >> 6))
    {
      regval |= PMC_PCK_PRES_DIV64;
      actual  = clkin >> 6;
    }
  else
    {
      serr("ERROR: frequency cannot be realized.\n");
      serr("       frequency=%lu clkin=%lu\n",
           (unsigned long)frequency, (unsigned long)clkin);
      return 0;
    }
#endif

  /* Disable the programmable clock, configure the PCK output pin, then set
   * the selected configuration.
   */

  switch (pckid)
    {
    case PCK0:
      putreg32(PMC_PCK0, SAM_PMC_SCDR);
#ifdef PIO_PMC_PCK0
      (void)sam_configpio(PIO_PMC_PCK0);
#endif
      putreg32(regval, SAM_PMC_PCK0);
      break;

    case PCK1:
      putreg32(PMC_PCK1, SAM_PMC_SCDR);
#ifdef PIO_PMC_PCK1
      (void)sam_configpio(PIO_PMC_PCK1);
#endif
      putreg32(regval, SAM_PMC_PCK1);
      break;

    case PCK2:
      putreg32(PMC_PCK2, SAM_PMC_SCDR);
#ifdef PIO_PMC_PCK2
      (void)sam_configpio(PIO_PMC_PCK2);
#endif
      putreg32(regval, SAM_PMC_PCK2);
      break;

    default:
      return -EINVAL;
    }

  /* And return the actual frequency */

  return actual;
}

/****************************************************************************
 * Function: sam_pck_enable
 *
 * Description:
 *   Enable or disable a programmable clock output.
 *
 * Input Parameters:
 *   pckid - Identifies the programmable clock output (0, 1, or 2)
 *   enable - True: enable the clock output, False: disable the clock output
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void sam_pck_enable(enum pckid_e pckid, bool enable)
{
  uintptr_t regaddr;
  uint32_t  regval;

  /* Select the bit in the PMC_SDER or PMC_SCER corresponding to the
   * programmable clock.
   */

  regval = PMC_PCKN(pckid);

  /* Select the SDER or SCER */

  regaddr = enable ? SAM_PMC_SCER : SAM_PMC_SCDR;

  /* And do the deead */

  putreg32(regval, regaddr);
}
