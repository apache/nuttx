/****************************************************************************
 * arch/arm/src/common/stm32/hardware/stm32_dmamux.h
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

#ifndef __ARCH_ARM_SRC_COMMON_STM32_HARDWARE_STM32_DMAMUX_H
#define __ARCH_ARM_SRC_COMMON_STM32_HARDWARE_STM32_DMAMUX_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#if defined(CONFIG_STM32_HAVE_IP_DMA_V1_7CH_DMAMUX)
#  include "hardware/stm32_dmamux_7ch.h"
#elif defined(CONFIG_STM32_HAVE_IP_DMA_V1_8CH_DMAMUX)
#  include "hardware/stm32_dmamux_16ch.h"
#else
#  error "Unsupported STM32 DMAMUX"
#endif

#endif /* __ARCH_ARM_SRC_COMMON_STM32_HARDWARE_STM32_DMAMUX_H */
