/****************************************************************************
 * arch/arm/src/stm32f7/stm32_qencoder.h
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

#ifndef __ARCH_ARM_SRC_STM32F7_STM32_QENCODER_H
#define __ARCH_ARM_SRC_STM32F7_STM32_QENCODER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

#ifdef CONFIG_SENSORS_QENCODER

/****************************************************************************
 * Included Files
 ****************************************************************************/

/* Timer devices may be used for different purposes.  One special purpose is
 * as a quadrature encoder input device.  If CONFIG_STM32F7_TIMn is defined
 * then the CONFIG_STM32F7_TIMn_QE must also be defined to indicate that
 * timer "n" is intended to be used for as a quadrature encoder.
 */

#ifndef CONFIG_STM32F7_TIM1
#  undef CONFIG_STM32F7_TIM1_QE
#endif
#ifndef CONFIG_STM32F7_TIM2
#  undef CONFIG_STM32F7_TIM2_QE
#endif
#ifndef CONFIG_STM32F7_TIM3
#  undef CONFIG_STM32F7_TIM3_QE
#endif
#ifndef CONFIG_STM32F7_TIM4
#  undef CONFIG_STM32F7_TIM4_QE
#endif
#ifndef CONFIG_STM32F7_TIM5
#  undef CONFIG_STM32F7_TIM5_QE
#endif
#ifndef CONFIG_STM32F7_TIM8
#  undef CONFIG_STM32F7_TIM8_QE
#endif

/* Only timers 2-5, and 1 & 8 can be used as a quadrature encoder
 * (at least for the STM32 F7)
 */

#undef CONFIG_STM32F7_TIM6_QE
#undef CONFIG_STM32F7_TIM7_QE
#undef CONFIG_STM32F7_TIM9_QE
#undef CONFIG_STM32F7_TIM10_QE
#undef CONFIG_STM32F7_TIM11_QE
#undef CONFIG_STM32F7_TIM12_QE
#undef CONFIG_STM32F7_TIM13_QE
#undef CONFIG_STM32F7_TIM14_QE

/* Clock out frequency.
 * This value is used to calculation the timer CLKIN in prescaler value.
 */

#ifndef CONFIG_STM32F7_TIM1_QECLKOUT
#  define CONFIG_STM32F7_TIM1_QECLKOUT 28000000
#endif

#ifndef CONFIG_STM32F7_TIM2_QECLKOUT
#  define CONFIG_STM32F7_TIM2_QECLKOUT 28000000
#endif

#ifndef CONFIG_STM32F7_TIM3_QECLKOUT
#  define CONFIG_STM32F7_TIM3_QECLKOUT 28000000
#endif

#ifndef CONFIG_STM32F7_TIM4_QECLKOUT
#  define CONFIG_STM32F7_TIM4_QECLKOUT 28000000
#endif

#ifndef CONFIG_STM32F7_TIM5_QECLKOUT
#  define CONFIG_STM32F7_TIM5_QECLKOUT 28000000
#endif

#ifndef CONFIG_STM32F7_TIM8_QECLKOUT
#  define CONFIG_STM32F7_TIM8_QECLKOUT 28000000
#endif

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_qeinitialize
 *
 * Description:
 *   Initialize a quadrature encoder interface.
 *    This function must be called from board-specific logic..
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/qe0"
 *   tim     - The timer number to used.
 *             'tim' must be an element of {1,2,3,4,5,8}
 *
 * Returned Value:
 *   Zero on success; A negated errno value is returned on failure.
 *
 ****************************************************************************/

int stm32_qeinitialize(const char *devpath, int tim);

#endif /* CONFIG_SENSORS_QENCODER */
#endif /* __ARCH_ARM_SRC_STM32F7_STM32_QENCODER_H */
