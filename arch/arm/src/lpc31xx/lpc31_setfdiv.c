/****************************************************************************
 * arch/arm/src/lpc31xx/lpc31_setfdiv.c
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

#include "lpc31_cgu.h"
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
 * Name: lpc31_setfdiv
 *
 * Description:
 *   Set/reset subdomain frequency containing the specified clock using the
 *   provided divider settings
 *
 ****************************************************************************/

void lpc31_setfdiv(enum lpc31_domainid_e dmnid,
                     enum lpc31_clockid_e clkid,
                     const struct lpc31_fdivconfig_s *fdiv)
{
  uint32_t regaddr;
  unsigned int basefreq;
  int fdcndx;
  int bcrndx;

  /* Get the frequency divider associated with this clock */

  fdcndx = lpc31_fdcndx(clkid, dmnid);

  /* Does this clock have a frequency divider? */

  if (fdcndx != FDCNDX_INVALID)
    {
      /* Yes.. Save the current reference frequency selection */

      regaddr  = LPC31_CGU_SSR((int)dmnid);
      basefreq = (getreg32(regaddr) & CGU_SSR_FS_MASK) >> CGU_SSR_FS_SHIFT;

      /* Switch domain to FFAST input */

      lpc31_selectfreqin(dmnid, CGU_FS_FFAST);

      /* Get the index of the associated BCR register.  Does this domain
       * have a BCR?
       */

      bcrndx = lpc31_bcrndx(dmnid);
      if (bcrndx != BCRNDX_INVALID)
        {
          /* Yes... Disable the BCR */

          regaddr = LPC31_CGU_BCR(bcrndx);
          putreg32(0, regaddr);
        }

      /* Change fractional divider to the provided settings */

      lpc31_fdivinit(fdcndx, fdiv, true);

      /* Re-enable the BCR (if one is associated with this domain) */

      if (bcrndx != BCRNDX_INVALID)
        {
          regaddr = LPC31_CGU_BCR(bcrndx);
          putreg32(CGU_BCR_FDRUN, regaddr);
        }

      /* Switch the domain back to its original base frequency */

      lpc31_selectfreqin(dmnid, basefreq);
    }
}
