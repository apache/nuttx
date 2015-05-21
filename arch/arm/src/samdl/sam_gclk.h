/****************************************************************************
 * arch/arm/src/samdl/sam_gclk.h
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
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_SAMDL_SAM_GCLK_H
#define __ARCH_ARM_SRC_SAMDL_SAM_GCLK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include "sam_config.h"

#if defined(CONFIG_ARCH_FAMILY_SAMD20)
#  include "chip/samd_gclk.h"
#elif defined(CONFIG_ARCH_FAMILY_SAML21)
#  include "chip/saml_gclk.h"
#else
#  error Unrecognized SAMD/L architecture
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/* This structure describes the configuration of one GCLK */

struct sam_gclkconfig_s
{
  uint8_t  gclk;        /* Clock generator */
  bool     runstandby;  /* Run clock in standby */
  bool     output;      /* Output enable */
  uint8_t  clksrc;      /* Encoded clock source */
  uint16_t prescaler;   /* Prescaler value */
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
 * Name: sam_gclk_config
 *
 * Description:
 *   Configure a single GCLK(s) based on settings in the config structure.
 *
 * Input Parameters:
 *   config - An instance of struct sam_gclkconfig describing the GCLK
 *            configuration.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void sam_gclk_config(FAR const struct sam_gclkconfig_s *config);

/****************************************************************************
 * Name: sam_gclk_chan_enable
 *
 * Description:
 *  Configure and enable a GCLK peripheral channel.
 *
 * Input Parameters:
 *   channel - Index of the GCLK channel to be enabled
 *   srcgen  - The GCLK source generator index
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_FAMILY_SAML21
void sam_gclk_chan_enable(uint8_t channel, uint8_t srcgen);
#endif

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

#ifdef CONFIG_ARCH_FAMILY_SAML21
void sam_gclk_chan_disable(uint8_t channel);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_SAMDL_SAM_GCLK_H */
