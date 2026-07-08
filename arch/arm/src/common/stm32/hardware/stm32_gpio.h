/****************************************************************************
 * arch/arm/src/common/stm32/hardware/stm32_gpio.h
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

#ifndef __ARCH_ARM_SRC_COMMON_STM32_HARDWARE_STM32_GPIO_H
#define __ARCH_ARM_SRC_COMMON_STM32_HARDWARE_STM32_GPIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#if defined(CONFIG_STM32_HAVE_IP_GPIO_M0_V1)
#  include "hardware/stm32_gpio_v2_m0.h"
#elif defined(CONFIG_STM32_STM32L15XX)
#  include "hardware/stm32l15xxx_gpio.h"
#elif defined(CONFIG_STM32_STM32F10XX)
#  include "hardware/stm32f10xxx_gpio.h"
#elif defined(CONFIG_STM32_STM32F20XX)
#  include "hardware/stm32f20xxx_gpio.h"
#elif defined(CONFIG_STM32_STM32F30XX) || defined(CONFIG_STM32_STM32F33XX) || \
      defined(CONFIG_STM32_STM32F37XX)
#  include "hardware/stm32f30xxx_gpio.h"
#elif defined(CONFIG_STM32_STM32F4XXX)
#  include "hardware/stm32f40xxx_gpio.h"
#elif defined(CONFIG_STM32_STM32G4XXX)
#  include "hardware/stm32g4xxxx_gpio.h"
#else
#  error "Unsupported STM32 GPIO"
#endif

#endif /* __ARCH_ARM_SRC_COMMON_STM32_HARDWARE_STM32_GPIO_H */
