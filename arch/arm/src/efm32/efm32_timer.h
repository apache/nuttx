/****************************************************************************
 * arch/arm/src/efm32/efm32_timer.h
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

#ifndef __ARCH_ARM_SRC_EFM32_EFM32_TIMER_H
#define __ARCH_ARM_SRC_EFM32_EFM32_TIMER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/efm32_timer.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

void efm32_timer_reset(uintptr_t base);
void efm32_timer_dumpregs(uintptr_t base, const char *msg);
int efm32_timer_set_freq(uintptr_t base, uint32_t clk_freq, uint32_t freq);

#endif /* __ARCH_ARM_SRC_EFM32_EFM32_TIMER_H */
