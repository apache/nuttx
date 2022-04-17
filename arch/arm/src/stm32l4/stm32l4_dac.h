/****************************************************************************
 * arch/arm/src/stm32l4/stm32l4_dac.h
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

#ifndef __ARCH_ARM_SRC_STM32L4_STM32L4_DAC_H
#define __ARCH_ARM_SRC_STM32L4_STM32L4_DAC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "hardware/stm32l4_dac.h"

#include <nuttx/analog/dac.h>

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Timer devices may be used for different purposes.  One special purpose is
 * to control periodic DAC outputs.  If CONFIG_STM32L4_TIMn is defined then
 * CONFIG_STM32L4_TIMn_DAC must also be defined to indicate that timer "n"
 * is intended to be used for that purpose.
 */

#ifndef CONFIG_STM32L4_TIM1
#  undef CONFIG_STM32L4_TIM1_DAC
#endif
#ifndef CONFIG_STM32L4_TIM2
#  undef CONFIG_STM32L4_TIM2_DAC
#endif
#ifndef CONFIG_STM32L4_TIM3
#  undef CONFIG_STM32L4_TIM3_DAC
#endif
#ifndef CONFIG_STM32L4_TIM4
#  undef CONFIG_STM32L4_TIM4_DAC
#endif
#ifndef CONFIG_STM32L4_TIM5
#  undef CONFIG_STM32L4_TIM5_DAC
#endif
#ifndef CONFIG_STM32L4_TIM6
#  undef CONFIG_STM32L4_TIM6_DAC
#endif
#ifndef CONFIG_STM32L4_TIM7
#  undef CONFIG_STM32L4_TIM7_DAC
#endif
#ifndef CONFIG_STM32L4_TIM8
#  undef CONFIG_STM32L4_TIM8_DAC
#endif
#ifndef CONFIG_STM32L4_TIM15
#  undef CONFIG_STM32L4_TIM15_DAC
#endif
#ifndef CONFIG_STM32L4_TIM16
#  undef CONFIG_STM32L4_TIM16_DAC
#endif
#ifndef CONFIG_STM32L4_TIM17
#  undef CONFIG_STM32L4_TIM17_DAC
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef CONFIG_STM32L4_DAC1_DMA
extern uint16_t stm32l4_dac1_dmabuffer[];
#endif
#ifdef CONFIG_STM32L4_DAC2_DMA
extern uint16_t stm32l4_dac2_dmabuffer[];
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: stm32l4_dacinitialize
 *
 * Description:
 *   Initialize the DAC
 *
 * Input Parameters:
 *   intf - The DAC interface number.
 *
 * Returned Value:
 *   Valid dac device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct dac_dev_s;
struct dac_dev_s *stm32l4_dacinitialize(int intf);

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM_SRC_STM32L4_STM32L4_DAC_H */
