/****************************************************************************
 * arch/arm/src/stm32wb/hardware/stm32wb_pinmap.h
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

#ifndef __ARCH_ARM_SRC_STM32WB_HARDWARE_STM32WB_PINMAP_H
#define __ARCH_ARM_SRC_STM32WB_HARDWARE_STM32WB_PINMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

#if defined(CONFIG_STM32WB_STM32WB10) || defined(CONFIG_STM32WB_STM32WB15) || \
    defined(CONFIG_STM32WB_STM32WB30) || defined(CONFIG_STM32WB_STM32WB35) || \
    defined(CONFIG_STM32WB_STM32WB50) || defined(CONFIG_STM32WB_STM32WB55)
#  if defined(CONFIG_STM32WB_USE_LEGACY_PINMAP)
#    include "hardware/stm32wbxx_pinmap_legacy.h"
#  else
#    include "hardware/stm32wbxx_pinmap.h"
#  endif
#else
#  error "Unsupported STM32WB Pin map"
#endif

#endif /* __ARCH_ARM_SRC_STM32WB_HARDWARE_STM32WB_PINMAP_H */
