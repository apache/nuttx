/****************************************************************************
 * arch/arm/src/armv7-m/arm_perf.c
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

#include "arm_internal.h"
#include "dwt.h"
#include "itm.h"
#include "nvic.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static unsigned long g_cpu_freq;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void up_perf_init(void *arg)
{
  g_cpu_freq = (unsigned long)(uintptr_t)arg;

  /* Enable ITM and DWT resources, if not left enabled by debugger. */

  modifyreg32(NVIC_DEMCR, 0, NVIC_DEMCR_TRCENA);

  /* Make sure the high speed cycle counter is running.  It will be started
   * automatically only if a debugger is connected.
   */

  putreg32(0xc5acce55, ITM_LAR);
  modifyreg32(DWT_CTRL, 0, DWT_CTRL_CYCCNTENA_MASK);
}

unsigned long up_perf_getfreq(void)
{
  return g_cpu_freq;
}

unsigned long up_perf_gettime(void)
{
  return getreg32(DWT_CYCCNT);
}

void up_perf_convert(unsigned long elapsed, struct timespec *ts)
{
  unsigned long left;

  ts->tv_sec  = elapsed / g_cpu_freq;
  left        = elapsed - ts->tv_sec * g_cpu_freq;
  ts->tv_nsec = NSEC_PER_SEC * (uint64_t)left / g_cpu_freq;
}
