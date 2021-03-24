/****************************************************************************
 * arch/arm/src/sama5/sam_hsmci_clkdiv.c
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

#include "chip.h"
#include "hardware/sam_hsmci.h"
#include "sam_hsmci.h"

#if defined(CONFIG_SAMA5_HSMCI0) || defined(CONFIG_SAMA5_HSMCI1) || \
    defined(CONFIG_SAMA5_HSMCI2)

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
 * Name: sam_hsmci_clkdiv
 *
 * Description:
 *   Multimedia Card Interface clock (MCCK or MCI_CK) is Master Clock (MCK)
 *   divided by (2*(CLKDIV) + CLOCKODD + 2).
 *
 *     CLKFULLDIV = 2*CLKDIV + CLOCKODD;
 *     MCI_SPEED  = MCK / (CLKFULLDIV + 2)
 *     CLKFULLDIV = MCK / MCI_SPEED - 2
 *
 *     CLKDIV     = CLKFULLDIV >> 1
 *     CLOCKODD   = CLKFULLDIV & 1
 *
 *   Where CLKDIV has a range of 0-255.
 *
 *   NOTE: The primary use of this function is for cases where the clock
 *   frequencies are not known a priori and so HSMCI clock dividers must
 *   be determined dynamically.  This is the case, for example, when we
 *   execute out of SDRAM.  In that case, the clocking was set up by the
 *   bootloader that brought us into SDRAM and it is that bootloader that
 *   has configured the clocking.
 *
 * Input Parameters:
 *   target - The target SD frequency
 *
 * Returned Value:
 *   A bitset containing the CLKDIV and CLKODD bits as needed to configure
 *   the HSMCI clock output.
 *
 ****************************************************************************/

uint32_t sam_hsmci_clkdiv(uint32_t target)
{
  uint32_t clkfulldiv;
  uint32_t ret;

  /* Get the largest divisor does not exceed the target value */

  clkfulldiv = (BOARD_MCK_FREQUENCY + target - 1) / target;

  if (clkfulldiv > 2)
    {
     clkfulldiv -= 2;
    }
  else
    {
     clkfulldiv = 0;
    }

  if (clkfulldiv > 511)
    {
      clkfulldiv = 511;
    }

  ret = (clkfulldiv >> 1) << HSMCI_MR_CLKDIV_SHIFT;
  if ((clkfulldiv & 1) != 0)
    {
      ret |= HSMCI_MR_CLKODD;
    }

  return ret;
}

#endif /* CONFIG_SAMA5_HSMCI0 || CONFIG_SAMA5_HSMCI1 || CONFIG_SAMA5_HSMCI2 */
