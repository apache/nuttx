/****************************************************************************
 * arch/arm/src/stm32/hardware/stm32_dac.h
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

#ifndef __ARCH_ARM_SRC_STM32_HARDWARE_STM32_DAC_H
#define __ARCH_ARM_SRC_STM32_HARDWARE_STM32_DAC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/* There are 2 main types of DAC IP cores among STM32 chips:
 *   1. STM32 DAC IPv1: F1, F2, F3, F4, F7, L1, L4
 *   2. STM32 DAC IPv2: G4
 */

#if defined(CONFIG_STM32_HAVE_IP_DAC_V1) && \
    defined(CONFIG_STM32_HAVE_IP_DAC_V2)
#  error Only one STM32 DAC IP version must be selected
#endif

#if defined(CONFIG_STM32_HAVE_IP_DAC_V1)
#  include "stm32_dac_v1.h"
#elif defined(CONFIG_STM32_HAVE_IP_DAC_V2)
#  if defined(CONFIG_STM32_STM32G4XXX)
#    include "stm32gxxxxx_dac.h"   /* Special case for G4 */
#  else
#    error "STM32 DAC device not supported"
#  endif
#else
#  error "STM32 DAC IP version not specified"
#endif

#endif /* __ARCH_ARM_SRC_STM32_HARDWARE_STM32_DAC_H */
