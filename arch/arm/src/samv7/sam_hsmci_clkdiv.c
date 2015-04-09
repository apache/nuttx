/****************************************************************************
 * arch/arm/src/samv7/sam_pmc.c
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * References:
 *
 *   SAMV7D3 Series Data Sheet
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

#include "chip.h"
#include "chip/sam_hsmci.h"
#include "sam_hsmci.h"

#ifdef CONFIG_SAMV7_HSMCI0

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
 * Input parameters:
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

#endif /* CONFIG_SAMV7_HSMCI */
