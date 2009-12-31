/****************************************************************************
 * arch/arm/src/lpc313x/lpc313x_setfdiv.c
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

#include <stdint.h>
#include <stdbool.h>

#include <arch/board/board.h>

#include "lpc313x_cgu.h"
#include "lpc313x_cgudrvr.h"

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
 * Name: lpc313x_setfdiv
 *
 * Description:
 *   Set/reset subdomain frequency containing the specified clock using the
 *   provided divider settings
 *
 ****************************************************************************/

void lpc313x_setfdiv(enum lpc313x_domainid_e dmnid,
                     enum lpc313x_clockid_e clkid,
                     const struct lpc313x_fdivconfig_s *fdiv)
{
  uint32_t regaddr;
  unsigned int basefreq;
  int fdcndx;
  int bcrndx;
  
  /* Get the frequency divider associated with this clock */

  fdcndx = lpc313x_fdcndx(clkid, dmnid);

  /* Does this clock have a frequency divicer? */

  if (fdcndx != FDCNDX_INVALID)
    {
      /* Yes.. Save the current reference frequency selection */

      regaddr  = LPC313X_CGU_SSR_OFFSET((int)dmnid);
      basefreq = (getreg32(regaddr) & CGU_SSR_FS_MASK) >> CGU_SSR_FS_SHIFT;
      
      /* Switch domain to FFAST input */

      lpc313x_selectfreqin(dmnid, CGU_FS_FFAST);

      /* Get the index of the associated BCR register.  Does this domain
       * have a BCR?
       */

      bcrndx = lp313x_bcrndx(dmnid);
      if (bcrndx != BCRNDX_INVALID)
        {

          /* Yes... Disable the BCR */

          regaddr = LPC313X_CGU_BCR(bcrndx);
          putreg32(0, regaddr);
        }

      /* Change fractional divider to the provided settings */

      lpc313x_fdivinit(fdcndx, fdiv, true);

      /* Re-enable the BCR (if one is associated with this domain) */

      if (bcrndx != BCRNDX_INVALID)
        {
          regaddr = LPC313X_CGU_BCR(bcrndx);
          putreg32(CGU_BCR_FDRUN, regaddr);
        }

      /* Switch the domain back to its original base frequency */

      lpc313x_selectfreqin(dmnid, basefreq);
    }
}
