/****************************************************************************
 * arch/arm/src/stm32h5/stm32_adc.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __ARCH_ARM_SRC_STM32H5_STM32_ADC_H
#define __ARCH_ARM_SRC_STM32H5_STM32_ADC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/analog/adc.h>
#include "chip.h"
#include "hardware/stm32_adc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if defined(CONFIG_STM32H5_ADC1) || defined(CONFIG_STM32H5_ADC2)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: stm32h5_adc_initialize
 *
 * Description:
 *    Initialize the ADC.
 *
 * Input Parameters:
 *    intf      - Could be {1,2} for ADC1, ADC2
 *    chanlist  - The list of channels
 *    nchannels - Number of channels
 *
 * Returned Value:
 *    Valid ADC device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct adc_dev_s;
struct adc_dev_s *stm32h5_adc_initialize(int intf,
                                         const uint8_t *chanlist,
                                         int nchannels);
#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif /* __ASSEMBLY__ */

#endif /* CONFIG_STM32H5_ADC1 || CONFIG_STM32H5_ADC2*/
#endif /* __ARCH_ARM_SRC_STM32H5_STM32_ADC_H */
