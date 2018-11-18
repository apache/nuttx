/****************************************************************************
 * arch/arm/src/max326xx/max326_clockconfig.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_MAX326XX_MAX326_CLOCKCONFIG_H
#define __ARCH_ARM_SRC_MAX326XX_MAX326_CLOCKCONFIG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Include the MCU-specific structure can be used to define a clock
 * configuration. */

#if defined(CONFIG_ARCH_FAMILY_MAX32620) || defined(CONFIG_ARCH_FAMILY_MAX32630)
#  include "max32620_30/max32620_30_clockconfig.h"
#elif defined(CONFIG_ARCH_FAMILY_MAX32660)
#  include "max32660/max32660_clockconfig.h"
#else
#  error "Unsupported MAX326XX family"
#endif

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

void max326_clockconfig(FAR const struct clock_setup_s *clksetup);

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
