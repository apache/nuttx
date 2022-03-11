/****************************************************************************
 * arch/arm/src/sama5/sam_pmc.c
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

/* References:
 *   SAMA5D3 Series Data Sheet
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"

#ifdef CONFIG_ARCH_HAVE_SDIO
#  include "hardware/sam_hsmci.h"
#endif

#include "hardware/sam_pmc.h"
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

  /* MDIV = n:
   * Master Clock is Prescaler Output Clock divided by encoded value
   */

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
