/****************************************************************************
 * arch/arm/src/lpc17xx/lpc17_adc.h
 *
 *   Copyright (C) 2010, 2012, 2013 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_LPC17XX_LPC17_ADC_H
#define __ARCH_ARM_SRC_LPC17XX_LPC17_ADC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip/lpc17_adc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/
/* If CONFIG_ADC_CHANLIST is enabled, then the platform specific code must do
 * two things:  (1) define CONFIG_ADC_NCHANNELS in the configuration file and
 * (2) provide an array g_adc_chanlist[] with the channel numbers matching
 * the ADC0_MASK within the board-specific library.
 */

#ifdef CONFIG_ADC_CHANLIST 
#  if !defined(CONFIG_ADC_NCHANNELS)
#    error "CONFIG_ADC_CHANLIST must defined in this configuration"
#  elif CONFIG_ADC_NCHANNELS < 1
#    error "The value of CONFIG_ADC_NCHANNELS is invalid"
#  endif
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


/* The errata that states: "A/D Global Data register should not be used with
 * burst mode or hardware triggering".  The configuration option
 * CONFIG_ADC_CHANLIST is a workaround for this errata.  If this option is
 * selected, then the ADC driver will grab from the individual channel
 * registers rather than from the global data register as this is the stated
 * workaround in the errata.
 *
 * If this option is enabled, then the platform specific code must do two
 * things:  (1) define CONFIG_ADC_NCHANNELS in the configuration file and
 * (2) provide an array g_adc_chanlist[] with the channel numbers matching
 * the ADC0_MASK within the board-specific library.
 */

#ifdef CONFIG_ADC_CHANLIST 
EXTERN uint8_t g_adc_chanlist[CONFIG_ADC_NCHANNELS];
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc17_adcinitialize
 *
 * Description:
 *   Initialize the adc
 *
 * Returned Value:
 *   Valid can device structure reference on succcess; a NULL on failure
 *
 ****************************************************************************/

#ifdef CONFIG_LPC17_ADC
FAR struct adc_dev_s *lpc17_adcinitialize(void);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ARCH_ARM_SRC_LPC17XX_LPC17_ADC_H */
