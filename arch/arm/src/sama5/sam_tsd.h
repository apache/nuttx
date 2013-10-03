/****************************************************************************
 * arch/arm/src/sama5/sam_tsd.h
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

#ifndef __ARCH_ARM_SRC_SAMA5_SAM_TSD_H
#define __ARCH_ARM_SRC_SAMA5_SAM_TSD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip/sam_adc.h"

#if defined(CONFIG_SAMA5_ADC) && defined(CONFIG_SAMA5_TSD)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/

#ifdef CONFIG_SAMA_TSD_RXP
#  define CONFIG_SAMA_TSD_RXP 6
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
 * Public Functions
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
int sam_tsd_register(FAR struct sam_adc_s *adc, int minor);

/****************************************************************************
 * Interfaces exported from the touchscreen to the ADC driver
 ****************************************************************************/
/****************************************************************************
 * Name: sam_tsd_interrupt
 *
 * Description:
 *   Handles ADC interrupts associated with touchscreen channels
 *
 * Input parmeters:
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
