/****************************************************************************
 * arch/arm/src/sama5/sam_pmc.h
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_SAMA5_SAM_PMC_H
#define __ARCH_ARM_SRC_SAMA5_SAM_PMC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

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
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_pllack_frequency
 *
 * Description:
 *   Given the Main Clock frequency that provides the input to PLLA, return
 *   the frequency of the PPA output clock, PLLACK
 *
 * Assumptions:
 *   PLLA is enabled.  If the PLL is is disabled, either at the input divider
 *   or the output multiplier, the value zero is returned.
 *
 ****************************************************************************/

uint32_t sam_pllack_frequency(uint32_t mainclk);

/****************************************************************************
 * Name: sam_plladiv2_frequency
 *
 * Description:
 *   The PLLACK input to most clocking may or may not be divided by two.
 *   This function will return the possibly divided PLLACK clock input
 *   frequency.
 *
 * Assumptions:
 *   See sam_pllack_frequency.
 *
 ****************************************************************************/

uint32_t sam_plladiv2_frequency(uint32_t mainclk);

/****************************************************************************
 * Name: sam_pck_frequency
 *
 * Description:
 *   Given the Main Clock frequency that provides the input to PLLA, return
 *   the frequency of the processor clock (PCK).
 *
 * Assumptions:
 *   PLLA is enabled and the either the main clock or the PLLA output clock
 *   (PLLACK) provides the input to the MCK prescaler.
 *
 ****************************************************************************/

uint32_t sam_pck_frequency(uint32_t mainclk);

/****************************************************************************
 * Name: sam_mck_frequency
 *
 * Description:
 *   Given the Main Clock frequency that provides the input to PLLA, return
 *   the frequency of the PPA output clock, PLLACK
 *
 * Assumptions:
 *   PLLA is enabled and the either the main clock or the PLLA output clock
 *   (PLLACK) provides the input to the MCK prescaler.
 *
 ****************************************************************************/

uint32_t sam_mck_frequency(uint32_t mainclk);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_SAMA5_SAM_PMC_H */
