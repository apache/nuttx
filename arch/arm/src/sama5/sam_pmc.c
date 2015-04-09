/****************************************************************************
 * arch/arm/src/sama5/sam_pmc.c
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * References:
 *
 *   SAMA5D3 Series Data Sheet
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
 * 3. Neither the name NuttX nor the names of its contributors may
 *    be used to endorse or promote products derived from this software
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

#include <arch/board/board.h>

#include "up_arch.h"

#include "chip.h"

#ifdef CONFIG_ARCH_HAVE_SDIO
#  include "chip/sam_hsmci.h"
#endif

#include "chip/sam_pmc.h"
#include "sam_pmc.h"

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
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_pllack_frequency
 *
 * Description:
 *   Given the Main Clock frequency that provides the input to PLLA, return
 *   the frequency of the PPA output clock, PLLACK
 *
 * Assumptions:
 *   PLLA is enabled.  If the PLL is is disabled, either at the input divider
 *   or the output multiplier, the value zero is returned.
 *
 ****************************************************************************/

uint32_t sam_pllack_frequency(uint32_t mainclk)
{
  uint32_t regval;
#ifdef SAMA5_HAVE_PLLAR_DIV
  uint32_t diva;
#endif
  uint32_t mula;
  uint32_t pllack;

  /* Get the PLLA configuration.  We will multiply (and possibly divide)
   * the Main Clock to get the PLLA output clock (PLLACK).
   */

  regval = getreg32(SAM_PMC_CKGR_PLLAR);
  pllack = mainclk;

#ifdef SAMA5_HAVE_PLLAR_DIV
  /* Get the PLLA divider (DIVA)
   *
   *   DIVA = 0:     Divider output is 0
   *   DIVA = 1:     Divider is bypassed
   *   DIVA = 2-255: Divider output is the selected clock divided by DIVA
   */

  diva = (regval & PMC_CKGR_PLLAR_DIV_MASK) >> PMC_CKGR_PLLAR_DIV_SHIFT;
  if (diva > 1)
    {
     pllack /= diva;
    }
  else if (diva < 1)
    {
      return 0;
    }
#endif

 /* Get the PLLA multiplier (MULA)
  *
  *   MULA = 0: PLLA is deactivated
  *   MULA > 0: The PLLA Clock frequency is the PLLA input frequency
  *              multiplied by MULA + 1.
  */

  mula = (regval & PMC_CKGR_PLLAR_MUL_MASK) >> PMC_CKGR_PLLAR_MUL_SHIFT;
  if (mula > 0)
    {
      pllack *= (mula + 1);
    }
  else
    {
      return 0;
    }

  return pllack;
}

/****************************************************************************
 * Name: sam_plladiv2_frequency
 *
 * Description:
 *   The PLLACK input to most clocking may or may not be divided by two.
 *   This function will return the possibly divided PLLACK clock input
 *   frequency.
 *
 * Assumptions:
 *   See sam_pllack_frequency.
 *
 ****************************************************************************/

uint32_t sam_plladiv2_frequency(uint32_t mainclk)
{
  uint32_t regval;
  uint32_t pllack;

  /* Get the PLLA output clock */

  pllack = sam_pllack_frequency(mainclk);
  if (pllack == 0)
    {
      return 0;
    }

  /* Check if the PLLACK output is divided by 2 */

  regval = getreg32(SAM_PMC_MCKR);
  if ((regval & PMC_MCKR_PLLADIV2) != 0)
    {
      pllack >>= 1;
    }

  return pllack;
}

/****************************************************************************
 * Name: sam_pck_frequency
 *
 * Description:
 *   Given the Main Clock frequency that provides the input to PLLA, return
 *   the frequency of the processor clock (PCK).
 *
 * Assumptions:
 *   PLLA is enabled and the either the main clock or the PLLA output clock
 *   (PLLACK) provides the input to the MCK prescaler.
 *
 ****************************************************************************/

uint32_t sam_pck_frequency(uint32_t mainclk)
{
  uint32_t regval;
  uint32_t pres;
  uint32_t pck;

  /* Get the input source selection to the master/processor clock divider */

  regval = getreg32(SAM_PMC_MCKR);
  switch (regval & PMC_MCKR_CSS_MASK)
    {
    case PMC_MCKR_CSS_MAIN: /* Main Clock */
      /* Use the Main Clock frequency */

      pck = mainclk;
      break;

    case PMC_MCKR_CSS_PLLA: /* PLLA Clock */
      /* Use the PLLA output clock */

      pck = sam_plladiv2_frequency(mainclk);
      if (pck == 0)
        {
          return 0;
        }
      break;

    case PMC_MCKR_CSS_SLOW: /* Slow Clock */
    case PMC_MCKR_CSS_UPLL: /* UPLL Clock */
    default:
      return 0;
    }

  /* Get the PCK frequency which is given by the selected input clock
   * divided by a power-of-two prescaler.
   *
   * PRES = 0: Selected clock
   * PRES = n > 0: Selected clock divided by 2**n
   */

  pres = (regval & PMC_MCKR_PRES_MASK) >> PMC_MCKR_PRES_SHIFT;
  return pck >> pres;
}

/****************************************************************************
 * Name: sam_mck_frequency
 *
 * Description:
 *   Given the Main Clock frequency that provides the input to PLLA, return
 *   the frequency of the PPA output clock, PLLACK
 *
 * Assumptions:
 *   PLLA is enabled and the either the main clock or the PLLA output clock
 *   (PLLACK) provides the input to the MCK prescaler.
 *
 ****************************************************************************/

uint32_t sam_mck_frequency(uint32_t mainclk)
{
  uint32_t regval;
  uint32_t mdiv;
  uint32_t mck;

  /* The MCK frequency is equivalent to the PCK clock frequency with an
   * additional divider.
   */

  mck = sam_pck_frequency(mainclk);
  if (mck == 0)
    {
      return 0;
    }

  /* MDIV = n: Master Clock is Prescaler Output Clock divided by encoded value */

  regval = getreg32(SAM_PMC_MCKR);
  switch (regval & PMC_MCKR_MDIV_MASK)
    {
    case PMC_MCKR_MDIV_PCKDIV1:
      return mck;

    case PMC_MCKR_MDIV_PCKDIV2:
      mdiv = 2;
      break;

    case PMC_MCKR_MDIV_PCKDIV3:
      mdiv = 3;
      break;

    case PMC_MCKR_MDIV_PCKDIV4:
      mdiv = 4;
      break;

    default:
      return 0;
    }

  return mck / mdiv;
}
