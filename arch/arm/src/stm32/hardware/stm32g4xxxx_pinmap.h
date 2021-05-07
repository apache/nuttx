/****************************************************************************
 * arch/arm/src/stm32/hardware/stm32g4xxxx_pinmap.h
 *
 *  Licensed to the Apache Software Foundation (ASF) under one or more
 *  contributor license agreements.  See the NOTICE file distributed with
 *  this work for additional information regarding copyright ownership.  The
 *  ASF licenses this file to you under the Apache License, Version 2.0 (the
 *  "License"); you may not use this file except in compliance with the
 *  License.  You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 *  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 *  License for the specific language governing permissions and limitations
 *  under the License.
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_STM32_HARDWARE_STM32G4XXXX_PINMAP_H
#define __ARCH_ARM_SRC_STM32_HARDWARE_STM32G4XXXX_PINMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "stm32_gpio.h"

#if defined(CONFIG_STM32_STM32G4XXK)
#  include "stm32g4xxk_pinmap.h"
#elif defined(CONFIG_STM32_STM32G4XXC)
#  include "stm32g4xxc_pinmap.h"
#elif defined(CONFIG_STM32_STM32G4XXR)
#  include "stm32g4xxr_pinmap.h"
#elif defined(CONFIG_STM32_STM32G4XXM)
#  include "stm32g4xxm_pinmap.h"
#elif defined(CONFIG_STM32_STM32G4XXV)
#  include "stm32g4xxv_pinmap.h"
#elif defined(CONFIG_STM32_STM32G4XXP)
#  include "stm32g4xxp_pinmap.h"
#elif defined(CONFIG_STM32_STM32G4XXQ)
#  include "stm32g4xxq_pinmap.h"
#else
#  error "Unknown STM32G4xxxx chip!"
#endif

#endif /* __ARCH_ARM_SRC_STM32_HARDWARE_STM32G4XXXX_PINMAP_H */
