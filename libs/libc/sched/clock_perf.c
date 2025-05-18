/****************************************************************************
 * libs/libc/sched/clock_perf.c
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

#include <nuttx/arch.h>
#include <nuttx/bits.h>
#include <nuttx/clock.h>

#if defined(CONFIG_PERF_OVERFLOW_CORRECTION) && ULONG_MAX != UINT64_MAX

/****************************************************************************
 * Preprocessors
 ****************************************************************************/

#define MASK_LO        GENMASK_ULL(31, 0)
#define MASK_HI        GENMASK_ULL(63, 32)

#define LO(x)          (uint32_t)((x) & MASK_LO)
#define HI(x)          (uint32_t)(((x) & MASK_HI) >> 32)

#define PACK64(hi,lo)  ((MASK_LO & (lo)) | (((uint64_t)(hi)) << 32))
#define CLOCK_T(p)     (LO(p) | ((clock_t)HI(p) << \
                                 CONFIG_ARCH_PERF_COUNT_BITWIDTH))

/****************************************************************************
 * Private Data
 ****************************************************************************/

static atomic64_t g_perf;  /* hi word is overflow, lo word is last */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * perf_gettime
 ****************************************************************************/

clock_t perf_gettime(void)
{
  uint64_t snap;
  uint64_t result;
  clock_t now;

  do
    {
      snap = atomic64_read(&g_perf);
      now = up_perf_gettime();
      result = PACK64(now < LO(snap) ? HI(snap) + 1 : HI(snap), now);
    }
  while (!atomic64_try_cmpxchg(&g_perf, &snap, result));

  return CLOCK_T(result);
}

#else

/****************************************************************************
 * perf_gettime
 ****************************************************************************/

clock_t perf_gettime(void)
{
  return up_perf_gettime();
}

#endif
