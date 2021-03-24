/****************************************************************************
 * arch/arm/src/stm32/stm32_bkp.h
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

#ifndef __ARCH_ARM_SRC_STM32_STM32_BKP_H
#define __ARCH_ARM_SRC_STM32_STM32_BKP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/* Only the STM32 F1 family has a dedicated address region for BKP memory.
 * For F2, F3, and F4 parts, the bKP registers lie in the same address
 * region as the RTCC and the definitions in chip/stm32_rtcc.h should be used
 * to access backup registers.
 * NOTE:  These definitions are not interchangeable!
 */

#include "chip.h"
#ifdef CONFIG_STM32_STM32F10XX
#  include "hardware/stm32_bkp.h"
#else
#  include "hardware/stm32_rtcc.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_STM32_STM32_BKP_H */
