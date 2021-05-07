/****************************************************************************
 * arch/arm/src/lpc31xx/lpc31_defclk.c
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
#include <stdbool.h>

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
 * Name: lpc31_defclk
 *
 * Description:
 *   Enable the specified clock if it is one of the default clocks needed
 *   by the board.
 *
 ****************************************************************************/

bool lpc31_defclk(enum lpc31_clockid_e clkid)
{
  uint32_t regaddr;
  uint32_t regval;
  bool     enable;

  /* Check if this clock should be enabled.  This is determined by
   * 3 bitsets provided by board-specific logic in board/board.h.
   */

  if ((int)clkid < 32)
    {
      enable = ((BOARD_CLKS_0_31 & (1 << (int)clkid)) != 0);
    }
  else if ((int)clkid < 64)
    {
      enable = ((BOARD_CLKS_32_63 & (1 << ((int)clkid - 32))) != 0);
    }
  else
    {
      enable = ((BOARD_CLKS_64_92 & (1 << ((int)clkid - 64))) != 0);
    }

  /* Then set/clear the RUN bit in the PCR register for this clock
   * accordingly.
   */

  regaddr = LPC31_CGU_PCR((int)clkid);
  regval  = getreg32(regaddr);
  if (enable)
    {
      regval |= CGU_PCR_RUN;
    }
  else
    {
      regval &= ~CGU_PCR_RUN;
    }

  putreg32(regval, regaddr);
  return enable;
}
