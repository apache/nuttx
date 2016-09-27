/****************************************************************************
 * arch/arm/src/samv7/sam_dac.h
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_SAMV7_SAM_DAC_H
#define __ARCH_ARM_SRC_SAMV7_SAM_DAC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/analog/dac.h>
#include "chip/sam_dacc.h"

#if defined(CONFIG_SAMV7_DAC0) || defined(CONFIG_SAMV7_DAC1)

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/
/* Default configuration settings may be overridden in the board configuration
 * file.
 */

#if !defined(CONFIG_SAMV7_DAC_DMA_BUFFER_SIZE)
#  define CONFIG_SAMV7_DAC_DMA_BUFFER_SIZE 8
#elif CONFIG_SAMV7_DAC_DMA_BUFFER_SIZE > 65535
#  warning "CONFIG_SAMV7_DAC_DMA_BUFFER_SIZE value does not fit into uint16_t, limiting it to 65535"
#  undef  CONFIG_SAMV7_DAC_DMA_BUFFER_SIZE
#  define CONFIG_SAMV7_DAC_DMA_BUFFER_SIZE (65535)
#endif

#if !defined(CONFIG_SAMV7_DAC_TRIGGER_FREQUENCY)
#  define CONFIG_SAMV7_DAC_TRIGGER_FREQUENCY 8000
#endif

/* PRESCAL = (MCK / DACClock) - 2
 *
 * Given:
 *   MCK      = 150MHz
 *   DACClock = 16MHz
 * Then:
 *   PRESCAL  = 7
 */

#if !defined(CONFIG_SAMV7_DAC_PRESCAL)
#define CONFIG_SAMV7_DAC_PRESCAL          (7)
#elif CONFIG_SAMV7_DAC_PRESCAL > 15
#  warning "Maximum valid CONFIG_SAMV7_DAC_PRESCAL value is 15"
#endif

#if !defined(CONFIG_SAMV7_DAC_TRIGGER_SELECT)
#define CONFIG_SAMV7_DAC_TRIGGER_SELECT (3)
#elif CONFIG_SAMV7_DAC_TRIGGER_SELECT < 1 || CONFIG_SAMV7_DAC_TRIGGER_SELECT > 3
#  warning "Only CONFIG_SAMV7_DAC_TRIGGER_SELECT == [1-3] is supported"
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
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
 * Name: sam_dac_initialize
 *
 * Description:
 *   Initialize the DAC
 *
 * Input Parameters:
 *   intf - The DAC interface number.
 *
 * Returned Value:
 *   Valid DAC device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct dac_dev_s;
FAR struct dac_dev_s *sam_dac_initialize(int intf);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_SAMV7_DAC0 || CONFIG_SAMV7_DAC1 */
#endif /* __ARCH_ARM_SRC_SAMV7_SAM_DAC_H */
