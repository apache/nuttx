/****************************************************************************
 * arch/arm/src/tiva/tiva_sysctrl.h
 *
 *   Copyright (C) 2009-2010, 2013-2014 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_TIVA_TIVA_SYSCONTROL_H
#define __ARCH_ARM_SRC_TIVA_TIVA_SYSCONTROL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_ARCH_CHIP_TM4C129
/* Helpers for use with the TM4C129 version of tiva_clock_reconfigure() */

#  define M2PLLFREQ0(mint,mfrac) \
     ((uint32_t)((mint) << SYSCON_PLLFREQ0_MINT_SHIFT) | \
      (uint32_t)((mfrac) << SYSCON_PLLFREQ0_MFRAC_SHIFT))

#  define QN2PLLFREQ1(q,n) \
     ((uint32_t)(((n) - 1) << SYSCON_PLLFREQ1_N_SHIFT) | \
      (uint32_t)(((q) - 1) << SYSCON_PLLFREQ1_Q_SHIFT))
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Data
 ****************************************************************************/

#if defined(__cplusplus)
extern "C"
{
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#if defined(CONFIG_ARCH_CHIP_TM4C129)
/****************************************************************************
 * Name: tiva_clock_reconfigure
 *
 * Description:
 *   Called to change to new clock based on desired pllfreq0, pllfreq1, and
 *   sysdiv settings.  This is use to set up the initial clocking but can be
 *   used later to support slow clocked, low power consumption modes.
 *
 *   The pllfreq0 and pllfreq1 settings derive from the PLL M, N, and Q
 *   values to generate Fvco like:
 *
 *     Fin  = Fxtal / Q / N -OR- Fpiosc / Q / N
 *     Mdiv = Mint + (MFrac / 1024)
 *     Fvco = Fin * Mdiv
 *
 *   When the PLL is active, the system clock frequency (SysClk) is
 *   calculated using the following equation:
 *
 *     SysClk = Fvco/ sysdiv
 *
 *   NOTE: The input clock to the PLL may be either the external crystal
 *   (Fxtal) or PIOSC (Fpiosc).  This logic supports only the external
 *   crystal as the PLL source clock.
 *
 * Input Parameters:
 *   pllfreq0 - PLLFREQ0 register value (see helper macro M2PLLFREQ0()
 *   pllfreq1 - PLLFREQ1 register value (see helper macro QN2PLLFREQ1()
 *   sysdiv   - Fvco divider value
 *
 * Returned Value:
 *   The resulting SysClk frequency
 *
 ****************************************************************************/

uint32_t tiva_clock_reconfigure(uint32_t pllfreq0, uint32_t pllfreq1,
                                uint32_t sysdiv);

#else
/****************************************************************************
 * Name: tiva_clock_reconfigure
 *
 * Description:
 *   Called to change to new clock based on desired rcc and rcc2 settings.
 *   This is use to set up the initial clocking but can be used later to
 *   support slow clocked, low power consumption modes.
 *
 ****************************************************************************/

void tiva_clock_reconfigure(uint32_t newrcc, uint32_t newrcc2);
#endif

/****************************************************************************
 * Name: tiva_clock_configure
 *
 * Description:
 *   Called early in the boot sequence (before .data and .bss are available)
 *   in order to configure initial clocking.
 *
 ****************************************************************************/

void tiva_clock_configure(void);

#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_TIVA_TIVA_SYSCONTROL_H */
