/****************************************************************************
 * arch/arm/src/stm32h7/stm32_rcc.h
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

#ifndef __ARCH_ARM_SRC_STM32H7_STM32_RCC_H
#define __ARCH_ARM_SRC_STM32H7_STM32_RCC_H

#ifndef __ASSEMBLY__

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

 /* Oscillator frequencies */

#define STM32_HSI_FREQUENCY     64000000u
#define STM32_CSI_FREQUENCY     4000000u
#define STM32_HSI48_FREQUENCY   48000000u
#define STM32_LSI_FREQUENCY     32000u

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/clk/clk.h>
#include <nuttx/clk/clk_provider.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if defined(__cplusplus)
extern "C"
{
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#if defined(__cplusplus)
}
#endif
#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_STM32H7_STM32_RCC_H */
