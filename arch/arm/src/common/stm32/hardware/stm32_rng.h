/****************************************************************************
 * arch/arm/src/common/stm32/hardware/stm32_rng.h
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

#ifndef __ARCH_ARM_SRC_COMMON_STM32_HARDWARE_STM32_RNG_H
#define __ARCH_ARM_SRC_COMMON_STM32_HARDWARE_STM32_RNG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#if (defined(CONFIG_STM32_HAVE_IP_RNG_M0_V1) + \
     defined(CONFIG_STM32_HAVE_IP_RNG_M3M4_V1)) > 1
#  error Only one STM32 RNG IP version must be selected
#endif

#if defined(CONFIG_STM32_HAVE_IP_RNG_M0_V1)
#  include "hardware/stm32_rng_m0.h"
#elif defined(CONFIG_STM32_HAVE_IP_RNG_M3M4_V1)
#  include "hardware/stm32_rng_v1.h"
#else
#  error "Unsupported STM32 RNG"
#endif

#endif /* __ARCH_ARM_SRC_COMMON_STM32_HARDWARE_STM32_RNG_H */
