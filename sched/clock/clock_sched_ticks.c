/****************************************************************************
 * sched/clock/clock_sched_ticks.c
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

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/seqlock.h>

#include "clock/clock.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static volatile clock_t g_system_ticks = INITIAL_SYSTEM_TIMER_TICKS;
static seqcount_t g_system_tick_lock = SEQLOCK_INITIALIZER;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: clock_increase_sched_ticks
 *
 * Description:
 *   Increment the scheduler tick counter. This function should be called
 *   each time the real-time clock interrupt occurs, indicating the passage
 *   of one or more scheduling ticks.
 *
 * Input Parameters:
 *   ticks - The number of ticks to increment (typically 1)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void clock_increase_sched_ticks(clock_t ticks)
{
  irqstate_t flags;

  /* Increment the per-tick scheduler counter */

  flags = write_seqlock_irqsave(&g_system_tick_lock);
  g_system_ticks += ticks;
  write_sequnlock_irqrestore(&g_system_tick_lock, flags);
}

/****************************************************************************
 * Name: clock_get_sched_ticks
 *
 * Description:
 *   Return the current value of the scheduler tick counter. This counter
 *   only increases while the scheduler is running, and is independent of
 *   the real-time clock.
 *
 * Returned Value:
 *   The current number of scheduler ticks.
 *
 ****************************************************************************/

clock_t clock_get_sched_ticks(void)
{
  clock_t ret;
  unsigned int seq;

  do
    {
      seq = read_seqbegin(&g_system_tick_lock);
      ret = g_system_ticks;
    }
  while (read_seqretry(&g_system_tick_lock, seq));

  return ret;
}
