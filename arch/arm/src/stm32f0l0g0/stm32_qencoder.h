/****************************************************************************
 * arch/arm/src/stm32f0l0g0/stm32_qencoder.h
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

#ifndef __ARCH_ARM_SRC_STM32F0L0G0_STM32_QENCODER_H
#define __ARCH_ARM_SRC_STM32F0L0G0_STM32_QENCODER_H

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
 * as a quadrature encoder input device.  If CONFIG_STM32F0L0G0_TIMn is
 * defined then the CONFIG_STM32F0L0G0_TIMn_QE must also be defined to
 * indicate that timer "n" is intended to be used for as a quadrature
 * encoder.
 */

#ifndef CONFIG_STM32F0L0G0_TIM1
#  undef CONFIG_STM32F0L0G0_TIM1_QE
#endif
#ifndef CONFIG_STM32F0L0G0_TIM2
#  undef CONFIG_STM32F0L0G0_TIM2_QE
#endif
#ifndef CONFIG_STM32F0L0G0_TIM3
#  undef CONFIG_STM32F0L0G0_TIM3_QE
#endif
#ifndef CONFIG_STM32F0L0G0_TIM4
#  undef CONFIG_STM32F0L0G0_TIM4_QE
#endif

/* Only timers 1-4 can be used as a quadrature encoder (timers with
 * encoder mode support).
 * TIM6, TIM7, TIM14-17 are basic/general purpose timers without encoder
 * capability.
 */

#undef CONFIG_STM32F0L0G0_TIM6_QE
#undef CONFIG_STM32F0L0G0_TIM7_QE
#undef CONFIG_STM32F0L0G0_TIM14_QE
#undef CONFIG_STM32F0L0G0_TIM15_QE
#undef CONFIG_STM32F0L0G0_TIM16_QE
#undef CONFIG_STM32F0L0G0_TIM17_QE

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_qeinitialize
 *
 * Description:
 *   Initialize a quadrature encoder interface.
 *   This function must be called from board-specific logic.
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/qe0"
 *   tim     - The timer number to used.
 *             'tim' must be an element of {1,2,3,4}
 *
 * Returned Value:
 *   Zero on success; A negated errno value is returned on failure.
 *
 ****************************************************************************/

int stm32_qeinitialize(const char *devpath, int tim);

#ifdef CONFIG_STM32F0L0G0_QENCODER_INDEX_PIN
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
#endif /* __ARCH_ARM_SRC_STM32F0L0G0_STM32_QENCODER_H */
