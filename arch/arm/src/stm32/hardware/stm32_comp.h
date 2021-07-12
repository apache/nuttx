/****************************************************************************
 * arch/arm/src/stm32/hardware/stm32_comp.h
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

#ifndef __ARCH_ARM_SRC_STM32_HARDWARE_STM32_COMP_H
#define __ARCH_ARM_SRC_STM32_HARDWARE_STM32_COMP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

#ifdef CONFIG_STM32_COMP

/* Include the correct COMP register definitions for
 * selected STM32 COMP IP core:
 */

/* If more than one COMP IP ensure that only one is selected */

#if defined(CONFIG_STM32_HAVE_IP_COMP_V1)
#  if defined(CONFIG_STM32_STM32F33XX)
#    include "stm32f33xxx_comp.h"
#  else
#    error "Device not supported."
#  endif
#elif defined(CONFIG_STM32_HAVE_IP_COMP_V2)
#  if defined(CONFIG_STM32_STM32G4XXX)
#    include "stm32g4xxxx_comp.h"
#  else
#    error "Device not supported."
#  endif
#else
#  error "STM32 COMP IP not supported."
#endif

#endif /* CONFIG_STM32_COMP */

#endif /* __ARCH_ARM_SRC_STM32_HARDWARE_STM32_COMP_H */
