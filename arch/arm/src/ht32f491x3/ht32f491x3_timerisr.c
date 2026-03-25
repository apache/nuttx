/****************************************************************************
 * arch/arm/src/ht32f491x3/ht32f491x3_timerisr.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <time.h>

#include <nuttx/arch.h>
#include <nuttx/timers/arch_timer.h>

#include "nvic.h"
#include "clock/clock.h"
#include "arm_internal.h"
#include "systick.h"

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define HT32_SYSTICK_RELOAD ((HT32_HCLK_FREQUENCY / CLK_TCK) - 1)

#if HT32_SYSTICK_RELOAD > 0x00ffffff
#  error HT32_SYSTICK_RELOAD exceeds the SysTick reload field
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#if !defined(CONFIG_ARMV7M_SYSTICK) && !defined(CONFIG_TIMER_ARCH)
static int ht32f491x3_timerisr(int irq, FAR void *context, FAR void *arg)
{
  UNUSED(irq);
  UNUSED(context);
  UNUSED(arg);

  nxsched_process_timer();
  return 0;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void up_timer_initialize(void)
{
  uint32_t regval;

  regval  = getreg32(NVIC_SYSH12_15_PRIORITY);
  regval &= ~NVIC_SYSH_PRIORITY_PR15_MASK;
  regval |= (NVIC_SYSH_PRIORITY_DEFAULT << NVIC_SYSH_PRIORITY_PR15_SHIFT);
  putreg32(regval, NVIC_SYSH12_15_PRIORITY);

#if defined(CONFIG_ARMV7M_SYSTICK) && defined(CONFIG_TIMER_ARCH)
  up_timer_set_lowerhalf(systick_initialize(true, HT32_HCLK_FREQUENCY, -1));
#else
  putreg32(HT32_SYSTICK_RELOAD, NVIC_SYSTICK_RELOAD);

  irq_attach(HT32_IRQ_SYSTICK, ht32f491x3_timerisr, NULL);

  putreg32(NVIC_SYSTICK_CTRL_CLKSOURCE |
           NVIC_SYSTICK_CTRL_TICKINT |
           NVIC_SYSTICK_CTRL_ENABLE,
           NVIC_SYSTICK_CTRL);

  up_enable_irq(HT32_IRQ_SYSTICK);
#endif
}
