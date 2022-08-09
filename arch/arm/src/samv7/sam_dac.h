/****************************************************************************
 * arch/arm/src/samv7/sam_dac.h
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

#ifndef __ARCH_ARM_SRC_SAMV7_SAM_DAC_H
#define __ARCH_ARM_SRC_SAMV7_SAM_DAC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/analog/dac.h>
#include "hardware/sam_dacc.h"

#if defined(CONFIG_SAMV7_DAC0) || defined(CONFIG_SAMV7_DAC1)

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

/* Default configuration settings may be overridden in the board
 * configuration file.
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
struct dac_dev_s *sam_dac_initialize(int intf);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_SAMV7_DAC0 || CONFIG_SAMV7_DAC1 */
#endif /* __ARCH_ARM_SRC_SAMV7_SAM_DAC_H */
