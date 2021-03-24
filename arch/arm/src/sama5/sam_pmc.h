/****************************************************************************
 * arch/arm/src/sama5/sam_pmc.h
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
 * Public Functions Prototypes
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
