/****************************************************************************
 * arch/risc-v/src/common/riscv_perf_cycle.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this args for additional information regarding copyright ownership.  The
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

#include <nuttx/clock.h>
#include <nuttx/config.h>

#include <stdint.h>
#include <time.h>

#include "riscv_internal.h"

#ifdef CONFIG_ARCH_PERF_EVENTS

/****************************************************************************
 * Private Data
 ****************************************************************************/

static unsigned long g_cycle_frequency = ULONG_MAX;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_perf_init
 ****************************************************************************/

void up_perf_init(void *arg)
{
  g_cycle_frequency = (unsigned long)(uintptr_t)arg;
}

/****************************************************************************
 * Name: up_perf_gettime
 ****************************************************************************/

unsigned long up_perf_gettime(void)
{
  return READ_CSR(CSR_CYCLE);
}

/****************************************************************************
 * Name: up_perf_getfreq
 ****************************************************************************/

unsigned long up_perf_getfreq(void)
{
  return g_cycle_frequency;
}

/****************************************************************************
 * Name: up_perf_convert
 ****************************************************************************/

void up_perf_convert(unsigned long elapsed, struct timespec *ts)
{
  DEBUGASSERT(g_cycle_frequency && (g_cycle_frequency != ULONG_MAX));

  ts->tv_sec = elapsed / g_cycle_frequency;
  elapsed -= ts->tv_sec * g_cycle_frequency;
  ts->tv_nsec = elapsed * (NSEC_PER_SEC / g_cycle_frequency);
}
#endif
