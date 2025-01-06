/****************************************************************************
 * sched/clock/clock_gettime.c
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

#include <stdint.h>
#include <time.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/nuttx.h>
#include <nuttx/arch.h>
#include <nuttx/sched.h>
#include <nuttx/spinlock.h>

#include "clock/clock.h"
#include "sched/sched.h"
#ifdef CONFIG_CLOCK_TIMEKEEPING
#  include "clock/clock_timekeeping.h"
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#if CONFIG_SCHED_CRITMONITOR_MAXTIME_THREAD >= 0
static clock_t clock_process_runtime(FAR struct tcb_s *tcb)
{
# ifdef HAVE_GROUP_MEMBERS
  FAR struct task_group_s *group;
  FAR sq_entry_t *curr;
  clock_t runtime = 0;
  irqstate_t flags;

  group = tcb->group;

  flags = spin_lock_irqsave(&group->tg_lock);
  sq_for_every(&group->tg_members, curr)
    {
      tcb = container_of(curr, struct tcb_s, member);

      runtime += tcb->run_time;
    }

  spin_unlock_irqrestore(&group->tg_lock, flags);
  return runtime;
# else  /* HAVE_GROUP_MEMBERS */
  return tcb->run_time;
# endif /* HAVE_GROUP_MEMBERS */
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxclock_gettime
 *
 * Description:
 *   Get the current value of the specified time clock.
 *
 ****************************************************************************/

void nxclock_gettime(clockid_t clock_id, FAR struct timespec *tp)
{
  if (clock_id == CLOCK_MONOTONIC || clock_id == CLOCK_BOOTTIME)
    {
      /* The the time elapsed since the timer was initialized at power on
       * reset.
       */

      clock_systime_timespec(tp);
    }
  else if (clock_id == CLOCK_REALTIME)
    {
#ifndef CONFIG_CLOCK_TIMEKEEPING
      struct timespec ts;
      irqstate_t flags;

      clock_systime_timespec(&ts);

      /* Add the base time to this.  The base time is the time-of-day
       * setting.  When added to the elapsed time since the time-of-day
       * was last set, this gives us the current time.
       */

      flags = spin_lock_irqsave(&g_basetime_lock);
      clock_timespec_add(&g_basetime, &ts, tp);
      spin_unlock_irqrestore(&g_basetime_lock, flags);
#else
      clock_timekeeping_get_wall_time(tp);
#endif
    }
  else
    {
#if CONFIG_SCHED_CRITMONITOR_MAXTIME_THREAD >= 0
      clockid_t clock_type = clock_id & CLOCK_MASK;
      pid_t pid = clock_id >> CLOCK_SHIFT;
      FAR struct tcb_s *tcb;

      if (pid == 0)
        {
          tcb = this_task();
        }
      else
        {
          tcb = nxsched_get_tcb(pid);
        }

      if (tcb)
        {
          if (clock_type == CLOCK_PROCESS_CPUTIME_ID)
            {
              up_perf_convert(clock_process_runtime(tcb), tp);
            }
          else if (clock_type == CLOCK_THREAD_CPUTIME_ID)
            {
              up_perf_convert(tcb->run_time, tp);
            }
        }
#endif
    }
}

/****************************************************************************
 * Name: clock_gettime
 *
 * Description:
 *   Clock Functions based on POSIX APIs
 *
 *   CLOCK_MONOTONIC is an optional under POSIX: "If the Monotonic Clock
 *   option is supported, all implementations shall support a clock_id
 *   of CLOCK_MONOTONIC defined in <time.h>. This clock represents the
 *   monotonic clock for the system. For this clock, the value returned
 *   by clock_gettime() represents the amount of time (in seconds and
 *   nanoseconds) since an unspecified point in the past (for example,
 *   system start-up time, or the Epoch). This point does not change
 *   after system start-up time. The value of the CLOCK_MONOTONIC clock
 *   cannot be set via clock_settime(). This function shall fail if it
 *   is invoked with a clock_id argument of CLOCK_MONOTONIC."
 *
 *   CLOCK_REALTIME - POSIX demands this to be present.  CLOCK_REALTIME
 *   represents the machine's best-guess as to the current wall-clock,
 *   time-of-day time. This means that CLOCK_REALTIME can jump forward and
 *   backward as the system time-of-day clock is changed.
 *
 ****************************************************************************/

int clock_gettime(clockid_t clock_id, FAR struct timespec *tp)
{
  if (tp == NULL || clock_id < 0 || clock_id > CLOCK_BOOTTIME)
    {
      set_errno(EINVAL);
      return ERROR;
    }

  nxclock_gettime(clock_id, tp);
  return OK;
}
