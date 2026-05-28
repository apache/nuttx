/****************************************************************************
 * arch/arm/src/common/stm32/stm32_gpio.h
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

#ifndef __ARCH_ARM_SRC_COMMON_COMPAT_STM32_GPIO_H
#define __ARCH_ARM_SRC_COMMON_COMPAT_STM32_GPIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#if defined(CONFIG_STM32_HAVE_IP_GPIO_M0_V1)
#  include "stm32_gpio_m0_v1.h"
#elif defined(CONFIG_STM32_HAVE_IP_GPIO_M3M4_V1)
#  include "stm32_gpio_m3m4_v1v2.h"
#else
#  error "Unsupported STM32 GPIO"
#endif

#endif /* __ARCH_ARM_SRC_COMMON_COMPAT_STM32_GPIO_H */
