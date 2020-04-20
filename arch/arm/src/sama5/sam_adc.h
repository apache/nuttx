/****************************************************************************
 * arch/arm/src/sama5/sam_adc.h
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_SAMA5_SAM_ADC_H
#define __ARCH_ARM_SRC_SAMA5_SAM_ADC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/analog/adc.h>

#include "hardware/sam_adc.h"

#ifdef CONFIG_SAMA5_ADC

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_SCHED_WORKQUEUE
#  error Work queue support is required (CONFIG_SCHED_WORKQUEUE)
#endif

#ifndef CONFIG_DEBUG_FEATURES
#  undef CONFIG_SAMA5_ADC_REGDEBUG
#endif

/* ADC channels 0-3 or 0-4 are not available to the ADC driver if touchscreen
 * support is enabled.
 */

#ifdef CONFIG_SAMA5_TSD
#  undef CONFIG_SAMA5_ADC_CHAN0
#  undef CONFIG_SAMA5_ADC_CHAN1
#  undef CONFIG_SAMA5_ADC_CHAN2
#  undef CONFIG_SAMA5_ADC_CHAN3
#  ifdef CONFIG_SAMA5_TSD_5WIRE
#    undef CONFIG_SAMA5_ADC_CHAN4
#  endif
#endif

/* Do we have any ADC channels enabled?  If not, then the ADC driver may
 * still need to exist to support the touchscreen.
 */

#undef SAMA5_ADC_HAVE_CHANNELS
#if defined(CONFIG_SAMA5_ADC_CHAN0) || defined(CONFIG_SAMA5_ADC_CHAN1) || \
    defined(CONFIG_SAMA5_ADC_CHAN2) || defined(CONFIG_SAMA5_ADC_CHAN3) || \
    defined(CONFIG_SAMA5_ADC_CHAN4) || defined(CONFIG_SAMA5_ADC_CHAN5) || \
    defined(CONFIG_SAMA5_ADC_CHAN6) || defined(CONFIG_SAMA5_ADC_CHAN7) || \
    defined(CONFIG_SAMA5_ADC_CHAN8) || defined(CONFIG_SAMA5_ADC_CHAN9) || \
    defined(CONFIG_SAMA5_ADC_CHAN10) || defined(CONFIG_SAMA5_ADC_CHAN11)
#  define SAMA5_ADC_HAVE_CHANNELS 1
#elif !defined(CONFIG_SAMA5_TSD)
#  error "No ADC channels nor touchscreen"
#endif

/****************************************************************************
 * Public Types
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
 * Public Functions Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: sam_adcinitialize
 *
 * Description:
 *   Initialize the ADC
 *
 * Returned Value:
 *   Valid can device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

FAR struct adc_dev_s *sam_adc_initialize(void);

/****************************************************************************
 * Interfaces exported from the ADC to the touchscreen driver
 ****************************************************************************/

/****************************************************************************
 * Name: sam_adc_lock
 *
 * Description:
 *   Get exclusive access to the ADC interface
 *
 ****************************************************************************/

struct sam_adc_s;
int sam_adc_lock(FAR struct sam_adc_s *priv);

/****************************************************************************
 * Name: sam_adc_unlock
 *
 * Description:
 *   Relinquish the lock on the ADC interface
 *
 ****************************************************************************/

void sam_adc_unlock(FAR struct sam_adc_s *priv);

/****************************************************************************
 * Name: sam_adc_getreg
 *
 * Description:
 *   Read any 32-bit register using an absolute address.
 *
 ****************************************************************************/

#ifdef CONFIG_SAMA5_ADC_REGDEBUG
uint32_t sam_adc_getreg(FAR struct sam_adc_s *priv, uintptr_t address);
#else
#  define sam_adc_getreg(handle,addr) getreg32(addr)
#endif

/****************************************************************************
 * Name: sam_adc_putreg
 *
 * Description:
 *   Write to any 32-bit register using an absolute address.
 *
 ****************************************************************************/

#ifdef CONFIG_SAMA5_ADC_REGDEBUG
void sam_adc_putreg(FAR struct sam_adc_s *priv, uintptr_t address,
                    uint32_t regval);
#else
#  define sam_adc_putreg(handle,addr,val) putreg32(val,addr)
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_SAMA5_ADC */
#endif /* __ARCH_ARM_SRC_SAMA5_SAM_ADC_H */
