/****************************************************************************
 * arch/arm/src/sama5/sam_tsd.h
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

#ifndef __ARCH_ARM_SRC_SAMA5_SAM_TSD_H
#define __ARCH_ARM_SRC_SAMA5_SAM_TSD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/sam_adc.h"

#if defined(CONFIG_SAMA5_ADC) && defined(CONFIG_SAMA5_TSD)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_SAMA_TSD_RXP
#  define CONFIG_SAMA_TSD_RXP 6
#endif

#if (defined CONFIG_SAMA5_ADC && defined CONFIG_SAMA5_ADC_SWTRIG) || !defined CONFIG_SAMA5_ADC
  /* Only allow Pendet triggering in limited circumstances */
#  define SAMA5_TSD_PENDET_TRIG_ALLOWED
#endif

/* Touchscreen interrupt event sets
 *
 *   ADC_INT_XRDY           TSD Measure XPOS Ready Interrupt
 *   ADC_INT_YRDY           TSD Measure YPOS Ready Interrupt
 *   ADC_INT_PRDY           TSD Measure Pressure Ready Interrupt
 *   ADC_INT_PEN            Pen Contact Interrupt
 *   ADC_INT_NOPEN          No Pen Contact Interrupt
 *   ADC_SR_PENS            Pen detect Status (Not an interrupt)
 */

#define ADC_TSD_CMNINTS     (ADC_INT_XRDY | ADC_INT_YRDY | ADC_INT_PRDY | ADC_INT_NOPEN)
#define ADC_TSD_ALLINTS     (ADC_TSD_CMNINTS | ADC_INT_PEN)
#define ADC_TSD_ALLSTATUS   (ADC_TSD_ALLINTS | ADC_SR_PENS)
#define ADC_TSD_RELEASEINTS ADC_TSD_CMNINTS

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
 * Name: sam_tsd_register
 *
 * Description:
 *   Configure the SAMA5 touchscreen.  This will register the driver as
 *   /dev/inputN where N is the minor device number
 *
 * Input Parameters:
 *   dev   - The ADC device handle received from sam_adc_initialize()
 *   minor - The input device minor number
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

struct sam_adc_s;
int sam_tsd_register(struct sam_adc_s *adc, int minor);

/****************************************************************************
 * Interfaces exported from the touchscreen to the ADC driver
 ****************************************************************************/

/****************************************************************************
 * Name: sam_tsd_interrupt
 *
 * Description:
 *   Handles ADC interrupts associated with touchscreen channels
 *
 * Input Parameters:
 *   pending - Current set of pending interrupts being handled
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void sam_tsd_interrupt(uint32_t pending);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_SAMA5_ADC && CONFIG_SAMA5_TSD */
#endif /* __ARCH_ARM_SRC_SAMA5_SAM_TSD_H */
