/****************************************************************************
 * arch/arm/src/mps/mps_timer.c
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

#include <arch/board/board.h>
#include <nuttx/timers/arch_timer.h>

#include "arm_internal.h"
#include "systick.h"
#include "mps_irq.h"
#include "nvic.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#define SYSTICK_RELOAD ((MPS_SYSTICK_CLOCK / CLK_TCK) - 1)

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
  /* Set reload register, qemu maybe have a bug,
   * if RELOAD is zero, set CTRL is not useful
   */

  putreg32(SYSTICK_RELOAD, NVIC_SYSTICK_RELOAD);
  up_timer_set_lowerhalf(systick_initialize(true, MPS_SYSTICK_CLOCK, -1));
}
