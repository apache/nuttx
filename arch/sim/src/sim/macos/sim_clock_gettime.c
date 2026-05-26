/****************************************************************************
 * arch/sim/src/sim/macos/sim_clock_gettime.c
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

#include <time.h>
#include <errno.h>

#include <nuttx/clock.h>
#include <nuttx/compiler.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Code built for the macOS host uses Darwin's libc clock IDs.  When such
 * code is linked into the NuttX simulator, those Darwin-specific values
 * reach NuttX's clock_gettime() instead of Darwin's one.  Translate
 * the Darwin monotonic clock IDs here so that the common sched/clock
 * implementation does not need to know about them.
 */

#define DARWIN_CLOCK_MONOTONIC_RAW        4
#define DARWIN_CLOCK_MONOTONIC_RAW_APPROX 5
#define DARWIN_CLOCK_MONOTONIC            6
#define DARWIN_CLOCK_UPTIME_RAW           8
#define DARWIN_CLOCK_UPTIME_RAW_APPROX    9

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: clock_gettime
 *
 * Description:
 *   sim/macOS specific override of clock_gettime() that translates Darwin
 *   monotonic clock IDs into NuttX's CLOCK_MONOTONIC before delegating
 *   to nxclock_gettime().
 *
 *   The common POSIX clock_gettime() in sched/clock/clock_gettime.c is
 *   declared as a weak symbol, so this strong definition takes precedence
 *   when CONFIG_HOST_MACOS is enabled.
 *
 ****************************************************************************/

visibility_hidden
int clock_gettime(clockid_t clock_id, FAR struct timespec *tp)
{
  int ret;

  switch (clock_id)
    {
    case DARWIN_CLOCK_MONOTONIC_RAW:
    case DARWIN_CLOCK_MONOTONIC_RAW_APPROX:
    case DARWIN_CLOCK_MONOTONIC:
    case DARWIN_CLOCK_UPTIME_RAW:
    case DARWIN_CLOCK_UPTIME_RAW_APPROX:
      clock_id = CLOCK_MONOTONIC;
      break;

    default:
      break;
    }

  ret = nxclock_gettime(clock_id, tp);
  if (ret < 0)
    {
      set_errno(-ret);
      return ERROR;
    }

  return OK;
}
