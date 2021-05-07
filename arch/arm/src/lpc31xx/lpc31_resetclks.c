/****************************************************************************
 * arch/arm/src/lpc31xx/lpc31_resetclks.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <arch/board/board.h>

#include "lpc31_cgudrvr.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

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
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc31_resetclks
 *
 * Description:
 *   Put all clocks into a known, initial state
 *
 ****************************************************************************/

void lpc31_resetclks(void)
{
  uint32_t regaddr;
  uint32_t regval;
  int bcrndx;
  int esrndx;
  int i;

  /* Switch all domain reference clocks to FFAST */

  for (i = 0; i < CGU_NDOMAINS; i++)
    {
      /* Switch reference clock in to FFAST */

      lpc31_selectfreqin((enum lpc31_domainid_e)i, CGU_FS_FFAST);

      /* Check if the domain has a BCR */

      bcrndx = lpc31_bcrndx((enum lpc31_domainid_e)i);
      if (bcrndx != BCRNDX_INVALID)
        {
          /* Yes.. disable all BCRs */

          putreg32(0, LPC31_CGU_BCR(bcrndx));
        }
    }

  /* Disable all clocks except those that are necessary */

  for (i = CLKID_FIRST; i <= CLKID_LAST; i++)
    {
      /* Check if this clock has an ESR register */

      esrndx = lpc31_esrndx((enum lpc31_clockid_e)i);
      if (esrndx != ESRNDX_INVALID)
        {
          /* Yes.. Clear the clocks ESR to deselect fractional divider */

          putreg32(0, LPC31_CGU_ESR(esrndx));
        }

      /* Enable external enabling for all possible clocks to conserve power */

      lpc31_enableexten((enum lpc31_clockid_e)i);

      /* Set enable-out's for only the following clocks */

      regaddr = LPC31_CGU_PCR(i);
      regval  = getreg32(regaddr);
      if (i == (int)CLKID_ARM926BUSIFCLK || i == (int)CLKID_MPMCCFGCLK)
        {
          regval |=  CGU_PCR_ENOUTEN;
        }
      else
        {
          regval &= ~CGU_PCR_ENOUTEN;
        }

      putreg32(regval, regaddr);

      /* Set/clear the RUN bit in the PCR register of  all clocks, depending
       * upon if the clock is needed by the board logic or not
       */

      lpc31_defclk((enum lpc31_clockid_e)i);
    }

  /* Disable all fractional dividers */

  for (i = 0; i < CGU_NFRACDIV; i++)
    {
      regaddr = LPC31_CGU_FDC(i);
      regval  = getreg32(regaddr);
      regval &= ~CGU_FDC_RUN;
      putreg32(regval, regaddr);
    }
}
