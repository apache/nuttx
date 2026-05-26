/****************************************************************************
 * arch/sim/src/sim/posix/sim_clockid.c
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
#include <nuttx/config.h>
#include <time.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Rust std uses CLOCK_UPTIME_RAW for Instant on Apple targets.  That Darwin
 * clock ID is not part of the NuttX clockid_t namespace, so translate it at
 * the sim/posix boundary instead of teaching the common clock_gettime()
 * implementation about host ABI values.
 *
 * Do not translate Darwin CLOCK_MONOTONIC_RAW (4) or
 * CLOCK_MONOTONIC_RAW_APPROX (5): those values overlap with NuttX
 * CLOCK_BOOTTIME and dynamic clock IDs.
 */

#define DARWIN_CLOCK_MONOTONIC            6
#define DARWIN_CLOCK_UPTIME_RAW           8
#define DARWIN_CLOCK_UPTIME_RAW_APPROX    9

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_translate_clockid
 ****************************************************************************/

clockid_t up_translate_clockid(clockid_t clockid)
{
  switch (clockid)
    {
      case DARWIN_CLOCK_MONOTONIC:
      case DARWIN_CLOCK_UPTIME_RAW:
      case DARWIN_CLOCK_UPTIME_RAW_APPROX:
        return CLOCK_MONOTONIC;

      default:
        return clockid;
    }
}
