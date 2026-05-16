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

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Rust code (and any other code) built for the macOS host uses Darwin's libc
 * clock IDs.  When such code is linked into the NuttX simulator, those
 * Darwin-specific values reach NuttX's clock_gettime() implementation
 * instead of Darwin's one.  Translate them here so that the common
 * sched/clock implementation does not need to know about Darwin clock IDs.
 *
 *   _CLOCK_MONOTONIC  == 6  (Darwin / XNU)
 *   _CLOCK_UPTIME_RAW == 8  (Darwin / XNU)
 */

#define DARWIN_CLOCK_MONOTONIC  6
#define DARWIN_CLOCK_UPTIME_RAW 8

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: clock_gettime
 *
 * Description:
 *   sim/macOS specific override of clock_gettime() that translates Darwin
 *   clock IDs (CLOCK_MONOTONIC=6, CLOCK_UPTIME_RAW=8) into NuttX's
 *   CLOCK_MONOTONIC before delegating to the common nxclock_gettime()
 *   implementation.
 *
 *   This override is required because Rust crates built for the macOS host
 *   call libc::clock_gettime() with Darwin's clock IDs.  When the resulting
 *   object code is linked into the NuttX simulator, the call is resolved to
 *   NuttX's clock_gettime() instead of Darwin's one.
 *
 *   The common POSIX clock_gettime() in sched/clock/clock_gettime.c is
 *   declared as a weak symbol, so this strong definition takes precedence
 *   when CONFIG_HOST_MACOS is enabled.
 *
 ****************************************************************************/

__attribute__((visibility("hidden")))
int clock_gettime(clockid_t clock_id, struct timespec *tp)
{
  int ret;

  if (clock_id == DARWIN_CLOCK_MONOTONIC ||
      clock_id == DARWIN_CLOCK_UPTIME_RAW)
    {
      clock_id = CLOCK_MONOTONIC;
    }

  ret = nxclock_gettime(clock_id, tp);
  if (ret < 0)
    {
      set_errno(-ret);
      return ERROR;
    }

  return OK;
}
