/****************************************************************************
 * arch/arm/src/lpc31xx/lpc31_setfreqin.c
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
 *   - NXP UM10314 LPC3130/31 User manual Rev. 1.01 — 9 September 2009
 *   - NXP lpc313x.cdl.drivers.zip example driver code
 */

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
 * Name: lpc31_selectfreqin
 *
 * Description:
 *   Set the base frequency source selection for with a clock domain
 *
 ****************************************************************************/

void lpc31_selectfreqin(enum lpc31_domainid_e dmnid, uint32_t finsel)
{
  uint32_t scraddr = LPC31_CGU_SCR(dmnid);
  uint32_t fs1addr = LPC31_CGU_FS1(dmnid);
  uint32_t fs2addr = LPC31_CGU_FS2(dmnid);
  uint32_t scrbits;

  /* Get the frequency selection from the switch configuration register (SCR)
   * for this domain.
   */

  scrbits = getreg32(scraddr) & ~(CGU_SCR_ENF1 | CGU_SCR_ENF2);

  /* If FS1 is currently enabled set the reference clock to FS2 and
   * enable FS2
   */

  if ((getreg32(LPC31_CGU_SSR(dmnid)) & CGU_SSR_FS1STAT) != 0)
    {
      /* Check if the selected frequency, FS1, is same as requested */

      if ((getreg32(fs1addr) & CGU_FS_MASK) != finsel)
        {
          /* No.. Set up FS2 */

          putreg32(finsel, fs2addr);
          putreg32(scrbits | CGU_SCR_ENF2, scraddr);
        }
    }

  /* FS1 is not currently enabled, check if the selected frequency, FS2,
   * is same as requested
   */

  else if ((getreg32(fs2addr) & CGU_FS_MASK) != finsel)
    {
      /* No.. Set up FS1 */

      putreg32(finsel, fs1addr);
      putreg32(scrbits | CGU_SCR_ENF1, scraddr);
    }
}
