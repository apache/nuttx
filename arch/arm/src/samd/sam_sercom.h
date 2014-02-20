/****************************************************************************
 * arch/arm/src/samd/sam_sercom.h
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

#ifndef __ARCH_ARM_SRC_SAMD_SAM_SERCOM_H
#define __ARCH_ARM_SRC_SAMD_SAM_SERCOM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>

#include "up_arch.h"
#include "sam_config.h"
#include "chip/sam_pm.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
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
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: sercom_enable
 *
 * Description:
 *   Enable clocking to a SERCOM module in PM
 *
 * Assumptions/Limitation:
 *   This operation is global and non-atomic.  The caller should disable
 *   interrupts prior to calling this function.
 *
 ****************************************************************************/

static inline void sercom_enable(int sercom)
{
  uint32_t regval;

  /* Enable clocking to the SERCOM module in PM */

  regval  = getreg32(SAM_PM_APBCMASK);
  regval |= PM_APBCMASK_SERCOM(sercom);
  putreg32(regval, SAM_PM_APBCMASK);
}

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/
/****************************************************************************
 * Name: sercom_coreclk_configure
 *
 * Description:
 *   Configure the SERCOM core source clock.
 *
 *   Two generic clocks are used by the SERCOM: GCLK_SERCOMx_CORE and
 *   GCLK_SERCOMx_SLOW.  The core clock (GCLK_SERCOMx_CORE) is required to
 *   clock the SERCOM while operating as a master, while the slow clock
 *   (GCLK_SERCOM_SLOW) is only required for certain functions.  SERCOM
 *   modules must share the same slow GCLK channel ID.
 *
 *   The baud-rate generator runs off the GCLK_SERCOMx_CORE clock (or,
 *   optionally, external clock).
 *
 ****************************************************************************/

void sercom_coreclk_configure(int sercom, int gclkgen, bool wrlock);

/****************************************************************************
 * Name: sercom_slowclk_configure
 *
 * Description:
 *   Configure the SERCOM slow source clock.
 *
 *   Two generic clocks are used by the SERCOM: GCLK_SERCOMx_CORE and
 *   GCLK_SERCOMx_SLOW.  The core clock (GCLK_SERCOMx_CORE) is required to
 *   clock the SERCOM while operating as a master, while the slow clock
 *   (GCLK_SERCOM_SLOW) is only required for certain functions.  SERCOM
 *   modules must share the same slow GCLK channel ID.
 *
 ****************************************************************************/

void sercom_slowclk_configure(int gclkgen);

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_SAMD_SAM_SERCOM_H */
