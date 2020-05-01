/****************************************************************************
 * arch/arm/src/samd5e5/sam_gclk.h
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

#ifndef __ARCH_ARM_SRC_SAMD5E5_SAM_GCLK_H
#define __ARCH_ARM_SRC_SAMD5E5_SAM_GCLK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include "arm_arch.h"
#include "sam_config.h"
#include "hardware/sam_gclk.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/* This structure describes the configuration of one GCLK */

struct sam_gclk_config_s
{
  uint8_t enable     : 1;    /* True:  Enable GCLK */
  uint8_t oov        : 1;    /* True:  Clock output selection */
  uint8_t oe         : 1;    /* True:  Output enable */
  uint8_t runstdby   : 1;    /* True:  Run in standby */
  uint8_t source;            /* GCLK clock source:
                              *   0  XOSC 0 oscillator input
                              *   1  XOSC 1 oscillator input
                              *   2  Generator input pad
                              *   3  Generic clock generator 1 output
                              *   4  OSCULP32K oscillator output
                              *   5  XOSC32K oscillator output
                              *   6  DFLL oscillator output
                              *   7  DPLL0 output
                              *   8  DPLL1 output */
  uint16_t div;              /* Division factor: 8-bits for all but GCLK1 */
};

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

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

/****************************************************************************
 * Name: sam_gclk_configure
 *
 * Description:
 *   Configure a single GCLK(s) based on settings in the config structure.
 *
 * Input Parameters:
 *   gclk   - GCLK index
 *   config - An instance of struct sam_gclkconfig describing the GCLK
 *            configuration.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void sam_gclk_configure(int gclk, FAR const struct sam_gclk_config_s *config);

/****************************************************************************
 * Name: sam_gclk_chan_enable
 *
 * Description:
 *  Configure and enable a GCLK peripheral channel.
 *
 * Input Parameters:
 *   channel - Index of the GCLK channel to be enabled
 *   srcgen  - The GCLK source generator index
 *   wrlock  - True: set writelock
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void sam_gclk_chan_enable(uint8_t channel, uint8_t srcgen, bool wrlock);

/****************************************************************************
 * Name: sam_gclk_chan_disable
 *
 * Description:
 *  Disable a GCLK peripheral channel.
 *
 * Input Parameters:
 *   channel - Index of the GCLK channel to be disabled
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void sam_gclk_chan_disable(uint8_t channel);

/****************************************************************************
 * Name: sam_gclk_chan_locked
 *
 * Description:
 *  Return true if the GCLK cannot be configured because the wrtlock is set
 *  int the PCHCTRL register.
 *
 * Input Parameters:
 *   channel - Index of the GCLK channel to be checked
 *
 * Returned Value:
 *   True if the the wrtlock bit is set in the channel's PCHCTRL register.
 *
 ****************************************************************************/

static inline bool sam_gclk_chan_locked(uint8_t channel)
{
  uint32_t regaddr;
  uint32_t regval;

  /* Get the address of the peripheral channel control register */

  regaddr = SAM_GCLK_PCHCTRL(channel);

  /* Get content of the peripheral channel control register */

  regval = getreg32(regaddr);

  /* Return true if the WRTLOCK bit is set in the register */

  return (regval & GCLK_PCHCTRL_WRTLOCK) != 0;
}

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_SAMD5E5_SAM_GCLK_H */
