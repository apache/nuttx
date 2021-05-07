/****************************************************************************
 * arch/arm/src/stm32/hardware/stm32_adc.h
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

#ifndef __ARCH_ARM_SRC_STM32_HARDWARE_STM32_ADC_H
#define __ARCH_ARM_SRC_STM32_HARDWARE_STM32_ADC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/* There are 2 main types of ADC IP cores among STM32 chips:
 *   1. STM32 ADC IPv1:
 *     a) basic version for F1 and F37x
 *     b) extended version for F2, F4, F7, L1:
 *   2. STM32 ADC IPv2:
 *     a) basic version for F0 and L0
 *     b) extended version for F3 (without F37x), G4, H7, L4, L4+
 *
 *   We also distinguish these variants:
 *   1. The modified STM32 ADC IPv1 core for the L1 family, which differs
 *      too much to keep it in the same file as ADC IPv1.
 *   2. The modified STM32 ADC IPv2 core for the G4 family, which differs
 *      too much to keep it in the same file as ADC IPv2.
 */

#if defined(CONFIG_STM32_HAVE_IP_ADC_V1) && \
    defined(CONFIG_STM32_HAVE_IP_ADC_V2)
#  error Only one STM32 ADC IP version must be selected
#endif

#if defined(CONFIG_STM32_HAVE_IP_ADC_V1)
#  if defined(CONFIG_STM32_STM32L15XX)
#    include "stm32_adc_v1l1.h"   /* Special case for L1 */
#  else
#    include "stm32_adc_v1.h"
#  endif
#elif defined(CONFIG_STM32_HAVE_IP_ADC_V2)
#  if defined(CONFIG_STM32_STM32G4XXX)
#    include "stm32_adc_v2g4.h"   /* Special case for G4 */
#  else
#    include "stm32_adc_v2.h"
#  endif
#else
#  error "STM32 ADC IP version not specified"
#endif

#endif /* __ARCH_ARM_SRC_STM32_HARDWARE_STM32_ADC_H */
