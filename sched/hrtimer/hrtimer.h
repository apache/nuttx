/****************************************************************************
 * sched/hrtimer/hrtimer.h
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

#ifndef __INCLUDE_SCHED_HRTIMER_HRTIMER_H
#define __INCLUDE_SCHED_HRTIMER_HRTIMER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/hrtimer.h>

/****************************************************************************
 * Pubic Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: hrtimer_initialize
 *
 * Description:
 *   Initialize the high-resolution timer queue for timing subsystem.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void hrtimer_initialize(void);

/****************************************************************************
 * Name: hrtimer_expiry/hrtimer_expiry_tick
 *
 * Description:
 *   This function is called by the timer interrupt handler to handle
 *   if a hrtimer has expired.
 *
 * Input Parameters:
 *   nsec/tick - The expiration time in nanoseconds or ticks.
 *   noswitches - True: Disable context switches.
 *
 * Returned Value:
 *   The next expiration time in nanoseconds or ticks.
 *
 ****************************************************************************/

uint64_t hrtimer_expiry(uint64_t nsec, bool noswitches);

static inline_function
clock_t hrtimer_expiry_tick(clock_t tick, bool noswitches)
{
  uint64_t ns   = hrtimer_expiry(tick * NSEC_PER_TICK, noswitches);
  clock_t  next = div_const_roundup(ns, NSEC_PER_TICK);
  return clock_compare(tick, next) ? next - tick : 0u;
}

#endif  /* __INCLUDE_SCHED_HRTIMER_HRTIMER_H */
