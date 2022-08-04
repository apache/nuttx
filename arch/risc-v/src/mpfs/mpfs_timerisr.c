/****************************************************************************
 * arch/risc-v/src/mpfs/mpfs_timerisr.c
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
#include <time.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <arch/board/board.h>
#include <nuttx/spinlock.h>
#include <nuttx/timers/arch_alarm.h>

#include "hardware/mpfs_clint.h"
#include "riscv_internal.h"
#include "riscv_mtimer.h"

#include "mpfs.h"
#include "mpfs_clockconfig.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MTIMER_FREQ MPFS_MSS_RTC_TOGGLE_CLK

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
  /* what is our timecmp address for this hart */

  uintptr_t hart_id = riscv_mhartid();

  struct oneshot_lowerhalf_s *lower = riscv_mtimer_initialize(
    MPFS_CLINT_MTIME, MPFS_CLINT_MTIMECMP0 + hart_id * sizeof(uintptr_t),
    RISCV_IRQ_TIMER, MTIMER_FREQ);

  DEBUGASSERT(lower);

  up_alarm_set_lowerhalf(lower);
}
