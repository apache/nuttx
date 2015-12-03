/************************************************************************************
 * arch/arm/src/samv7/sam_pck.h
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
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARPCKNG IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_SAMV7_SAM_PCK_H
#define __ARCH_ARM_SRC_SAMV7_SAM_PCK_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "chip/sam_pmc.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/************************************************************************************
 * Public Types
 ************************************************************************************/
/* Identifies the programmable clock */

enum pckid_e
{
  PCK0 = 0,
  PCK1,
  PCK2,
  PCK3,
  PCK4,
  PCK5,
  PCK6,
};

enum pckid_clksrc_e
{
  PCKSRC_MCK = 0,  /* Source clock is the master clock (MCK) or PLLA output (PLLACK) */
  PCKSRC_MAINCK,   /* Source clock is the main clock (probably the XTAL) */
  PCKSRC_SCK       /* Source clock is the slow clock (SCK) */
};

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

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
                           uint32_t frequency);

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

uint32_t sam_pck_frequency(enum pckid_e pckid);

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

void sam_pck_enable(enum pckid_e pckid, bool enable);

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

bool sam_pck_isenabled(enum pckid_e pckid);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_SAMV7_SAM_PCK_H */
