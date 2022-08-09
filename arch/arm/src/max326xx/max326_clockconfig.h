/****************************************************************************
 * arch/arm/src/max326xx/max326_clockconfig.h
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

#ifndef __ARCH_ARM_SRC_MAX326XX_MAX326_CLOCKCONFIG_H
#define __ARCH_ARM_SRC_MAX326XX_MAX326_CLOCKCONFIG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/* Include the MCU-specific structure can be used to define a clock
 * configuration.
 */

#if defined(CONFIG_ARCH_FAMILY_MAX32620) || defined(CONFIG_ARCH_FAMILY_MAX32630)
#  include "max32620_30/max32620_30_clockconfig.h"
#elif defined(CONFIG_ARCH_FAMILY_MAX32660)
#  include "max32660/max32660_clockconfig.h"
#else
#  error "Unsupported MAX326XX family"
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

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* This describes the initial clock configuration.  g_initial_clock_setup
 * must be provided by MCU-specific logic.
 */

EXTERN const struct clock_setup_s g_initial_clock_setup;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: max326_clockconfig
 *
 * Description:
 *   Called to initialize the MAX3266xx.  This does whatever setup is needed
 *   to put the MCU in a usable state.  This includes the initialization of
 *   clocking using the settings in board.h.  This function also performs
 *   other low-level chip as necessary.
 *
 ****************************************************************************/

void max326_clockconfig(const struct clock_setup_s *clksetup);

/****************************************************************************
 * Name: max326_hfio_frequency
 *
 * Description:
 *   Return the High-Frequency Internal Oscillator (HFIO) frequency.
 *
 ****************************************************************************/

uint32_t max326_hfio_frequency(void);

/****************************************************************************
 * Name: max326_cpu_frequency
 *
 * Description:
 *   Return the current CPU frequency.
 *
 ****************************************************************************/

uint32_t max326_cpu_frequency(void);

/****************************************************************************
 * Name: max326_pclk_frequency
 *
 * Description:
 *   Return the current peripheral clock frequency.
 *
 ****************************************************************************/

uint32_t max326_pclk_frequency(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_MAX326XX_MAX326_CLOCKCONFIG_H */
