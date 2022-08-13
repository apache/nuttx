/****************************************************************************
 * arch/arm/src/lpc17xx_40xx/lpc17_40_adc.h
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

#ifndef __ARCH_ARM_SRC_LPC17XX_40XX_LPC17_40_ADC_H
#define __ARCH_ARM_SRC_LPC17XX_40XX_LPC17_40_ADC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/lpc17_40_adc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* If CONFIG_LPC17_40_ADC_CHANLIST is enabled, then the platform specific
 * code must do two things:  (1) define CONFIG_LPC17_40_ADC_NCHANNELS in the
 * configuration file and (2) provide an array g_adc_chanlist[] with the
 * channel numbers matching the ADC0_MASK within the board-specific library.
 */

#ifdef CONFIG_LPC17_40_ADC_CHANLIST
#  if !defined(CONFIG_LPC17_40_ADC_NCHANNELS)
#    error "CONFIG_LPC17_40_ADC_CHANLIST must defined in this configuration"
#  elif CONFIG_LPC17_40_ADC_NCHANNELS < 1
#    error "The value of CONFIG_LPC17_40_ADC_NCHANNELS is invalid"
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
 * CONFIG_LPC17_40_ADC_CHANLIST is a workaround for this errata.  If this
 * option is selected, then the ADC driver will grab from the individual
 * channel registers rather than from the global data register as this is the
 * stated workaround in the errata.
 *
 * If this option is enabled, then the platform specific code must do two
 * things:
 * (1) define CONFIG_LPC17_40_ADC_NCHANNELS in the configuration file
 * (2) provide an array g_adc_chanlist[] with the channel numbers
 *     matching the ADC0_MASK within the board-specific library.
 */

#ifdef CONFIG_LPC17_40_ADC_CHANLIST
EXTERN uint8_t g_adc_chanlist[CONFIG_LPC17_40_ADC_NCHANNELS];
#endif

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: lpc17_40_adcinitialize
 *
 * Description:
 *   Initialize the ADC
 *
 * Returned Value:
 *   Valid ADC device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

#ifdef CONFIG_LPC17_40_ADC
struct adc_dev_s *lpc17_40_adcinitialize(void);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ARCH_ARM_SRC_LPC17XX_40XX_LPC17_40_ADC_H */
