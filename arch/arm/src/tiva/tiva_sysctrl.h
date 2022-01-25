/****************************************************************************
 * arch/arm/src/tiva/tiva_sysctrl.h
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

#ifndef __ARCH_ARM_SRC_TIVA_TIVA_SYSCTRL_H
#define __ARCH_ARM_SRC_TIVA_TIVA_SYSCTRL_H

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
#endif /* __ARCH_ARM_SRC_TIVA_TIVA_SYSCTRL_H */
