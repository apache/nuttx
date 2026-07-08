/****************************************************************************
 * arch/arm/src/common/stm32/stm32_qencoder.h
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

#ifndef __ARCH_ARM_SRC_COMMON_COMPAT_STM32_QENCODER_H
#define __ARCH_ARM_SRC_COMMON_COMPAT_STM32_QENCODER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

#ifdef CONFIG_SENSORS_QENCODER

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Timer devices may be used for different purposes.  One special purpose is
 * as a quadrature encoder input device.  If CONFIG_STM32_TIMn is defined
 * then CONFIG_STM32_TIMn_QE indicates that timer "n" is intended to be used
 * as a quadrature encoder.
 */

#ifndef CONFIG_STM32_TIM1
#  undef CONFIG_STM32_TIM1_QE
#endif
#ifndef CONFIG_STM32_TIM2
#  undef CONFIG_STM32_TIM2_QE
#endif
#ifndef CONFIG_STM32_TIM3
#  undef CONFIG_STM32_TIM3_QE
#endif
#ifndef CONFIG_STM32_TIM4
#  undef CONFIG_STM32_TIM4_QE
#endif
#ifndef CONFIG_STM32_TIM5
#  undef CONFIG_STM32_TIM5_QE
#endif
#ifndef CONFIG_STM32_TIM8
#  undef CONFIG_STM32_TIM8_QE
#endif

/* Basic and small general-purpose timers do not support encoder mode. */

#undef CONFIG_STM32_TIM6_QE
#undef CONFIG_STM32_TIM7_QE
#undef CONFIG_STM32_TIM9_QE
#undef CONFIG_STM32_TIM10_QE
#undef CONFIG_STM32_TIM11_QE
#undef CONFIG_STM32_TIM12_QE
#undef CONFIG_STM32_TIM13_QE
#undef CONFIG_STM32_TIM14_QE
#undef CONFIG_STM32_TIM15_QE
#undef CONFIG_STM32_TIM16_QE
#undef CONFIG_STM32_TIM17_QE

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_qeinitialize
 *
 * Description:
 *   Initialize a quadrature encoder interface.  This function must be called
 *   from board-specific logic.
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/qe0"
 *   tim     - The timer number to use.
 *
 * Returned Value:
 *   Zero on success; A negated errno value is returned on failure.
 *
 ****************************************************************************/

int stm32_qeinitialize(const char *devpath, int tim);

#ifdef CONFIG_STM32_QENCODER_INDEX_PIN
/****************************************************************************
 * Name: stm32_qe_index_init
 *
 * Description:
 *   Register the encoder index pin to a given Qencoder timer
 *
 * Input Parameters:
 *   tim  - The qenco timer number
 *   gpio - gpio pin configuration
 *
 * Returned Value:
 *   Zero on success; A negated errno value is returned on failure.
 *
 ****************************************************************************/

int stm32_qe_index_init(int tim, uint32_t gpio);
#endif

#endif /* CONFIG_SENSORS_QENCODER */

#endif /* __ARCH_ARM_SRC_COMMON_COMPAT_STM32_QENCODER_H */
