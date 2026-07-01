/****************************************************************************
 * arch/arm/src/rtl8720f/ameba_timerisr.c
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

#include <stdint.h>

#include <arch/board/board.h>
#include <nuttx/timers/arch_timer.h>

#include "arm_internal.h"
#include "systick.h"
#include "ameba_irq.h"
#include "nvic.h"

/****************************************************************************
 * External Symbols
 ****************************************************************************/

/* SDK CMSIS core-clock variable, set to the real KM4 CPU frequency (e.g.
 * 240 MHz) by SystemCoreClockUpdate() in ameba_app_start() before NuttX
 * runs.  The ARMv8-M SysTick is clocked from the processor clock
 * (CLKSOURCE=1, the 'true' below), so this is the correct reload base.
 * Reading it at runtime avoids hard-coding a frequency that must track the
 * SDK clock setup.
 */

extern uint32_t SystemCoreClock;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  up_timer_initialize
 *
 * Description:
 *   This function is called during start-up to initialize
 *   the timer hardware.
 *
 ****************************************************************************/

void up_timer_initialize(void)
{
  uint32_t freq = SystemCoreClock;

  /* Set reload register and start the generic ARMv8-M SysTick driver. */

  putreg32((freq / CLK_TCK) - 1, NVIC_SYSTICK_RELOAD);
  up_timer_set_lowerhalf(systick_initialize(true, freq, -1));
}
