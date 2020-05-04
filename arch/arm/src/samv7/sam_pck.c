/****************************************************************************
 * arch/arm/src/samv7/sam_pck.c
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

#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <arch/board/board.h>

#include "hardware/sam_pinmap.h"

#include "arm_arch.h"
#include "sam_gpio.h"

#include "sam_pck.h"

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
  uint32_t pres;

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
      clkin  = BOARD_MAINOSC_FREQUENCY;
      break;

    case PCKSRC_SCK: /* Source clock = SCK */
      regval = PMC_PCK_CSS_SLOW;
      clkin  = BOARD_SLOWCLK_FREQUENCY;
      break;

    default:
      _err("ERROR: Unknown clock source\n");
      return 0;
    }

  /* Programmable Clock frequency is selected clock frequency divided by PRES + 1 */

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

  /* Disable the programmable clock, configure the PCK output pin, then set
   * the selected configuration.
   */

  switch (pckid)
    {
    case PCK0:
      putreg32(PMC_PCK0, SAM_PMC_SCDR);
#ifdef GPIO_PMC_PCK0
      cam_configgpio(GPIO_PMC_PCK0);
#endif
      putreg32(regval, SAM_PMC_PCK0);
      break;

    case PCK1:
      putreg32(PMC_PCK1, SAM_PMC_SCDR);
#ifdef GPIO_PMC_PCK1
      cam_configgpio(GPIO_PMC_PCK1);
#endif
      putreg32(regval, SAM_PMC_PCK1);
      break;

    case PCK2:
      putreg32(PMC_PCK2, SAM_PMC_SCDR);
#ifdef GPIO_PMC_PCK2
      cam_configgpio(GPIO_PMC_PCK2);
#endif
      putreg32(regval, SAM_PMC_PCK2);
      break;

    case PCK3:
      putreg32(PMC_PCK3, SAM_PMC_SCDR);
#ifdef GPIO_PMC_PCK3
      cam_configgpio(GPIO_PMC_PCK3);
#endif
      putreg32(regval, SAM_PMC_PCK3);
      break;

    case PCK4:
      putreg32(PMC_PCK4, SAM_PMC_SCDR);
#ifdef GPIO_PMC_PCK4
      cam_configgpio(GPIO_PMC_PCK4);
#endif
      putreg32(regval, SAM_PMC_PCK4);
      break;

    case PCK5:
      putreg32(PMC_PCK5, SAM_PMC_SCDR);
#ifdef GPIO_PMC_PCK5
      cam_configgpio(GPIO_PMC_PCK5);
#endif
      putreg32(regval, SAM_PMC_PCK5);
      break;

    case PCK6:
      putreg32(PMC_PCK6, SAM_PMC_SCDR);
#ifdef GPIO_PMC_PCK6
      cam_configgpio(GPIO_PMC_PCK6);
#endif
      putreg32(regval, SAM_PMC_PCK6);
      break;

    default:
      return -EINVAL;
    }

  /* And return the actual frequency */

  return actual;
}

/****************************************************************************
 * Function: sam_pck_frequency
 *
 * Description:
 *   Return the frequency if the programmable clock
 *
 * Input Parameters:
 *   pckid - Identifies the programmable clock output (0, 1, .., 6)
 *
 * Returned Value:
 *   The frequency of the programmable clock (which may or may not be
 *   enabled).
 *
 ****************************************************************************/

uint32_t sam_pck_frequency(enum pckid_e pckid)
{
  uintptr_t regaddr;
  uint32_t regval;
  uint32_t clkin;
  uint32_t presc;

  /* Get the programmable clock configuration */

  regaddr = SAM_PMC_PCK((int)pckid);
  regval  = getreg32(regaddr);

  /* Get the frequency of the clock source */

  switch (regval & PMC_PCK_CSS_MASK)
    {
    case PMC_PCK_CSS_SLOW: /* Slow Clock */
      clkin = BOARD_SLOWCLK_FREQUENCY;
      break;

    case PMC_PCK_CSS_MAIN: /* Main Clock */
      clkin = BOARD_MAINOSC_FREQUENCY;
      break;

    case PMC_PCK_CSS_PLLA: /* PLLA Clock */
      clkin = BOARD_PLLA_FREQUENCY;
      break;

#ifdef BOARD_UPLL_FREQUENCY
    case PMC_PCK_CSS_UPLL: /* Divided UPLL Clock */
      clkin = BOARD_UPLL_FREQUENCY;
      break;
#endif

    case PMC_PCK_CSS_MCK:  /* Master Clock */
      clkin = BOARD_MCK_FREQUENCY;
      break;

    default:
      _err("ERROR: Unknown clock source\n");
      return 0;
    }

  /* Get the prescaler value */

  presc = (regval & PMC_PCK_PRES_MASK) >> PMC_PCK_PRES_SHIFT;
  return clkin / (presc + 1);
}

/****************************************************************************
 * Function: sam_pck_enable
 *
 * Description:
 *   Enable or disable a programmable clock output.
 *
 * Input Parameters:
 *   pckid - Identifies the programmable clock output (0, 1, .., 6)
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

  regval = PMC_PCK(pckid);

  /* Select the SDER or SCER */

  regaddr = enable ? SAM_PMC_SCER : SAM_PMC_SCDR;

  /* And do the deed */

  putreg32(regval, regaddr);
}

/****************************************************************************
 * Function: sam_pck_isenabled
 *
 * Description:
 *   Return true if the programmable clock is enabled.
 *
 * Input Parameters:
 *   pckid - Identifies the programmable clock output (0, 1, .., 6)
 *
 * Returned Value:
 *   True if the specified programmable clock is enabled
 *
 ****************************************************************************/

bool sam_pck_isenabled(enum pckid_e pckid)
{
  uint32_t  mask;

  /* Select the bit in the PMC_SCSR corresponding to the programmable clock. */

  mask = PMC_PCK(pckid);

  /* Return true if the bit is set */

  return (getreg32(SAM_PMC_SCSR) & mask) != 0;
}
