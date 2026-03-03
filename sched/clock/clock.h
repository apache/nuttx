/****************************************************************************
 * sched/clock/clock.h
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

#ifndef __SCHED_CLOCK_CLOCK_H
#define __SCHED_CLOCK_CLOCK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <nuttx/clock.h>
#include <nuttx/compiler.h>
#include <nuttx/spinlock_type.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* 32-bit mask for 64-bit timer values */

#define TIMER_MASK32 0x00000000ffffffff

/* Configuration ************************************************************/

/* If CONFIG_SYSTEM_TIME64 is selected and the CPU supports long long types,
 * then a 64-bit system time will be used.
 */

#ifndef CONFIG_HAVE_LONG_LONG
#  undef CONFIG_SYSTEM_TIME64
#endif

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef CONFIG_CLOCK_TIMEKEEPING
extern struct timespec  g_basetime;
extern spinlock_t g_basetime_lock;
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: clock_basetime
 *
 * Description:
 *   Get the initial time value from the best source available.
 *
 ****************************************************************************/

int  clock_basetime(FAR struct timespec *tp);

/****************************************************************************
 * Name: clock_initialize
 *
 * Description:
 *   Perform one-time initialization of the timing facilities.
 *
 ****************************************************************************/

void clock_initialize(void);

/****************************************************************************
 * Name: clock_update_sched_ticks
 *
 * Description:
 *   Update the scheduler tick counter to a specific value. This function
 *   directly sets the system tick counter to the given value (rather than
 *   incrementing it), typically used for synchronizing or resetting the
 *   scheduler tick count to a known state.
 *
 * Input Parameters:
 *   ticks - The new value to set for the scheduler tick counter
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void clock_update_sched_ticks(clock_t ticks);

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

void clock_increase_sched_ticks(clock_t ticks);

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

clock_t clock_get_sched_ticks(void);

#ifdef CONFIG_SCHED_CPULOAD_SYSCLK

/****************************************************************************
 * Name: cpuload_init
 *
 * Description:
 *   Initialize the CPU load measurement logic.
 *
 ****************************************************************************/

void cpuload_init(void);
#endif

#endif /* __SCHED_CLOCK_CLOCK_H */
