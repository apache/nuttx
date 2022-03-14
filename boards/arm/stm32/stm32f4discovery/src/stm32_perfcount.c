/****************************************************************************
 * boards/arm/stm32/stm32f4discovery/src/stm32_perfcount.c
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

#include <time.h>
#include <fixedmath.h>

#include "dwt.h"
#include "arm_internal.h"

#include <nuttx/clock.h>

#include <arch/board/board.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_perf_gettime
 ****************************************************************************/

uint32_t up_perf_gettime(void)
{
  return getreg32(DWT_CYCCNT);
}

/****************************************************************************
 * Name: up_perf_getfreq
 ****************************************************************************/

uint32_t up_perf_getfreq(void)
{
  return STM32_SYSCLK_FREQUENCY;
}

/****************************************************************************
 * Name: up_perf_convert
 ****************************************************************************/

void up_perf_convert(uint32_t elapsed, FAR struct timespec *ts)
{
  b32_t b32elapsed;

  b32elapsed  = itob32(elapsed) / STM32_SYSCLK_FREQUENCY;
  ts->tv_sec  = b32toi(b32elapsed);
  ts->tv_nsec = NSEC_PER_SEC * b32frac(b32elapsed) / b32ONE;
}
