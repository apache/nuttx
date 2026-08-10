/****************************************************************************
 * arch/tricore/src/common/tricore_perf.c
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

#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/lib/math32.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CPU_CCTRL        0xFC00  /* Counter Control */
#define CPU_CCNT         0xFC04  /* CPU Clock Cycle Count */

/* CPU_CCTRL Count Enable bit. */

#define CCTRL_COUNT_ENABLE (1 << 1)

#ifdef CONFIG_ARCH_PERF_EVENTS

/****************************************************************************
 * Private Data
 ****************************************************************************/

static unsigned long g_cpu_freq = ULONG_MAX;
static invdiv_param64_t g_invdiv_param;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void tricore_reset_ccnt(void)
{
  uint32_t cctrl = tricore_mfcr(CPU_CCTRL);

  /* Disable and clear the CPU cycle counter */

  cctrl &= ~CCTRL_COUNT_ENABLE;
  tricore_mtcr(CPU_CCTRL, cctrl);
  tricore_mtcr(CPU_CCNT, 0);
  UP_ISB();

  /* Re-enable the CPU cycle counter */

  cctrl |= CCTRL_COUNT_ENABLE;
  tricore_mtcr(CPU_CCTRL, cctrl);
  UP_ISB();
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void up_perf_init(void *arg)
{
  g_cpu_freq = (unsigned long)(uintptr_t)arg;

  invdiv_init_param64(g_cpu_freq, &g_invdiv_param);

  tricore_reset_ccnt();
}

unsigned long up_perf_getfreq(void)
{
  return g_cpu_freq;
}

clock_t up_perf_gettime(void)
{
  return (clock_t)tricore_mfcr(CPU_CCNT);
}

void up_perf_convert(clock_t elapsed, struct timespec *ts)
{
  clock_t left;

  ts->tv_sec  = invdiv_u64(elapsed, &g_invdiv_param);
  left        = elapsed - ts->tv_sec * g_cpu_freq;
  ts->tv_nsec = invdiv_u64(NSEC_PER_SEC * (uint64_t)left, &g_invdiv_param);
}
#endif
