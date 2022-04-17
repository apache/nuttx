/****************************************************************************
 * arch/arm/src/stm32/stm32_dac.h
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

#ifndef __ARCH_ARM_SRC_STM32_STM32_DAC_H
#define __ARCH_ARM_SRC_STM32_STM32_DAC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "hardware/stm32_dac.h"

#include <nuttx/analog/dac.h>

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Timer devices may be used for different purposes.  One special purpose is
 * to control periodic DAC outputs.  If CONFIG_STM32_TIMn is defined then
 * CONFIG_STM32_TIMn_DAC must also be defined to indicate that timer "n" is
 * intended to be used for that purpose.
 */

#ifndef CONFIG_STM32_TIM1
#  undef CONFIG_STM32_TIM1_DAC
#endif
#ifndef CONFIG_STM32_TIM2
#  undef CONFIG_STM32_TIM2_DAC
#endif
#ifndef CONFIG_STM32_TIM3
#  undef CONFIG_STM32_TIM3_DAC
#endif
#ifndef CONFIG_STM32_TIM4
#  undef CONFIG_STM32_TIM4_DAC
#endif
#ifndef CONFIG_STM32_TIM5
#  undef CONFIG_STM32_TIM5_DAC
#endif
#ifndef CONFIG_STM32_TIM6
#  undef CONFIG_STM32_TIM6_DAC
#endif
#ifndef CONFIG_STM32_TIM7
#  undef CONFIG_STM32_TIM7_DAC
#endif
#ifndef CONFIG_STM32_TIM8
#  undef CONFIG_STM32_TIM8_DAC
#endif
#ifndef CONFIG_STM32_TIM9
#  undef CONFIG_STM32_TIM9_DAC
#endif
#ifndef CONFIG_STM32_TIM10
#  undef CONFIG_STM32_TIM10_DAC
#endif
#ifndef CONFIG_STM32_TIM11
#  undef CONFIG_STM32_TIM11_DAC
#endif
#ifndef CONFIG_STM32_TIM12
#  undef CONFIG_STM32_TIM12_DAC
#endif
#ifndef CONFIG_STM32_TIM13
#  undef CONFIG_STM32_TIM13_DAC
#endif
#ifndef CONFIG_STM32_TIM14
#  undef CONFIG_STM32_TIM14_DAC
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* IOCTL commands specific to this driver  */

enum dac_io_cmds
{
  IO_DMABUFFER_INIT = 0,
};

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
 * Name: stm32_dacinitialize
 *
 * Description:
 *   Initialize the DAC
 *
 * Input Parameters:
 *   intf - The DAC interface number.
 *
 * Returned Value:
 *   Valid DAC device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct dac_dev_s;
struct dac_dev_s *stm32_dacinitialize(int intf);

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM_SRC_STM32_STM32_DAC_H */
