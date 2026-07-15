/****************************************************************************
 * arch/risc-v/src/gd32vw55x/gd32vw55x_timerisr.c
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

#include <assert.h>
#include <stdint.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/timers/arch_alarm.h>

#include "riscv_internal.h"
#include "riscv_mtimer.h"
#include "gd32vw55x_clockconfig.h"
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Nuclei SysTimer: 64-bit MTIMER at +0x0, MTIMERCMP at +0x8.  The clock
 * source is switched to the full system clock (160 MHz) by
 * gd32vw55x_clockconfig().
 */

#define GD32VW55X_MTIME     (GD32VW55X_SYSTIMER_BASE + 0x0000)
#define GD32VW55X_MTIMECMP  (GD32VW55X_SYSTIMER_BASE + 0x0008)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_timer_initialize
 *
 * Description:
 *   This function is called during start-up to initialize
 *   the timer interrupt.
 *
 ****************************************************************************/

void up_timer_initialize(void)
{
  struct oneshot_lowerhalf_s *lower = riscv_mtimer_initialize(
    GD32VW55X_MTIME, GD32VW55X_MTIMECMP,
    RISCV_IRQ_MTIMER, GD32VW55X_MTIME_FREQ);

  DEBUGASSERT(lower);

  up_alarm_set_lowerhalf(lower);
}
