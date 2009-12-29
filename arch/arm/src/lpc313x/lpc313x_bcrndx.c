/************************************************************************
 * arch/arm/src/lpc313x/lpc313x_bcrndx.c
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
 *
 * References:
 *   - UM10314 LPC3130/31 User manual Rev. 1.01 — 9 September 2009
 *   - lpc313x.cdl.drivers.zip example driver code
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
 ************************************************************************/

/************************************************************************
 * Included Files
 ************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>

#include "up_arch.h"
#include "lpc313x_cgudrvr.h"

/************************************************************************
 * Pre-processor Definitions
 ************************************************************************/

/************************************************************************
 * Private Data
 ************************************************************************/

/************************************************************************
 * Private Functions
 ************************************************************************/

/************************************************************************
 * Public Functions
 ************************************************************************/

/************************************************************************
 * Name: lp313x_esrndx
 *
 * Description:
 *   Given a clock ID, return the index of the corresponding ESR
 *   register (or ESRNDX_INVALID if there is no ESR associated with
 *   this clock ID).  Indexing of ESRs differs slightly from the clock
 *   ID:  There are 92 clock IDs but only 89 ESR regisers. There are no
 *  ESR registers for :
 *
 *
 *  CLKID_I2SRXBCK0         Clock ID 87: I2SRX_BCK0
 *  CLKID_I2SRXBCK1,        Clock ID 88: I2SRX_BCK1
 *
 * and
 *
 *  CLKID_SYSCLKO           Clock ID 91: SYSCLK_O
 *
 ************************************************************************/

int lp313x_ncrndx(enum lpc313x_domainid_e dmnid)
{
  switch (dmnid)
    {
      /* BCR0-3 correspond directly to domains 0-3 */

      case DOMAINID_SYS:        /* Domain 0: SYS_BASE */
      case DOMAINID_AHB0APB0:   /* Domain 1: AHB0APB0_BASE */
      case DOMAINID_AHB0APB1:   /* Domain 2: AHB0APB1_BASE */
      case DOMAINID_AHB0APB2:   /* Domain 3: AHB0APB2_BASE */
        return (int)dmnid;

      /* There is a BCR register corresponding to domain 7, but it is at
       * index 4
       */

      case DOMAINID_CLK1024FS: /* Domain 7: CLK1024FS_BASE */
        return 4;

      default:                 /* There is no BCR register for the other
                                * domains. */
        break;
    }
  return BCRNDX_INVALID;
}
