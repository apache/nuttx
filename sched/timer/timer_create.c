/****************************************************************************
 * sched/timer/timer_create.c
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
#include <unistd.h>
#include <time.h>
#include <string.h>
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/wdog.h>
#include <nuttx/kmalloc.h>

#include "timer/timer.h"

#ifndef CONFIG_DISABLE_POSIX_TIMERS

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: timer_allocate
 *
 * Description:
 *   Allocate one POSIX timer and place it into the allocated timer list.
 *
 ****************************************************************************/

static FAR struct posix_timer_s *timer_allocate(void)
{
  FAR struct posix_timer_s *ret;
  irqstate_t flags;
  uint8_t pt_flags;

  /* Try to get a preallocated timer from the free list */

#if CONFIG_PREALLOC_TIMERS > 0
  flags = enter_critical_section();
  ret   = (FAR struct posix_timer_s *)
    sq_remfirst((FAR sq_queue_t *)&g_freetimers);
  leave_critical_section(flags);

  /* Did we get one? */

  if (ret)
    {
      pt_flags = PT_FLAGS_PREALLOCATED;
    }
  else
#endif
    {
      /* Allocate a new timer from the heap */

      ret = (FAR struct posix_timer_s *)
        kmm_malloc(sizeof(struct posix_timer_s));
      pt_flags = 0;
    }

  /* If we have a timer, then put it into the allocated timer list */

  if (ret)
    {
      /* Initialize the timer structure */

      memset(ret, 0, sizeof(struct posix_timer_s));
      ret->pt_flags = pt_flags;

      /* And add it to the end of the list of allocated timers */

      flags = enter_critical_section();
      sq_addlast((FAR sq_entry_t *)ret, (FAR sq_queue_t *)&g_alloctimers);
      leave_critical_section(flags);
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: timer_create
 *
 * Description:
 *   The  timer_create() function creates per-thread timer using the
 *   specified clock, clock_id, as the timing base. The timer_create()
 *   function returns, in the location referenced by timerid, a timer ID of
 *   type timer_t used to identify the timer in timer requests. This timer
 *   ID is unique until the timer is deleted. The particular clock, clock_id,
 *   is defined in <time.h>. The timer whose ID is returned will be in a
 *   disarmed state upon return from timer_create().
 *
 *   The evp argument, if non-NULL, points to a sigevent structure. This
 *   structure is allocated by the called and defines the asynchronous
 *   notification to occur.  If the evp argument is NULL, the effect is as
 *   if the evp argument pointed to a sigevent structure with the
 *   sigev_notify member having the value SIGEV_SIGNAL, the sigev_signo
 *   having a default signal number, and the sigev_value member having the
 *   value of the timer ID.
 *
 *   Each implementation defines a set of clocks that can be used as timing
 *   bases for per-thread timers. All implementations shall support a
 *   clock_id of CLOCK_REALTIME.
 *
 * Input Parameters:
 *   clockid - Specifies the clock to use as the timing base.
 *   evp - Refers to a user allocated sigevent structure that defines the
 *     asynchronous notification.  evp may be NULL (see above).
 *   timerid - The pre-thread timer created by the call to timer_create().
 *
 * Returned Value:
 *   If the call succeeds, timer_create() will return 0 (OK) and update the
 *   location referenced by timerid to a timer_t, which can be passed to the
 *   other per-thread timer calls.  If an error occurs, the function will
 *   return a value of -1 (ERROR) and set errno to indicate the error.
 *
 *   EAGAIN - The system lacks sufficient signal queuing resources to honor
 *     the request.
 *   EAGAIN - The calling process has already created all of the timers it
 *     is allowed by this implementation.
 *   EINVAL - The specified clock ID is not defined.
 *   ENOTSUP - The implementation does not support the creation of a timer
 *     attached to the CPU-time clock that is specified by clock_id and
 *     associated with a thread different thread invoking timer_create().
 *
 * Assumptions:
 *
 ****************************************************************************/

int timer_create(clockid_t clockid, FAR struct sigevent *evp,
                 FAR timer_t *timerid)
{
  FAR struct posix_timer_s *ret;

  /* Sanity checks.  Also, we support only CLOCK_REALTIME */

  if (timerid == NULL || clockid != CLOCK_REALTIME)
    {
      set_errno(EINVAL);
      return ERROR;
    }

  /* Allocate a timer instance to contain the watchdog */

  ret = timer_allocate();
  if (!ret)
    {
      set_errno(EAGAIN);
      return ERROR;
    }

  /* Initialize the timer instance */

  ret->pt_crefs = 1;
  ret->pt_owner = getpid();
  ret->pt_delay = 0;

  /* Was a struct sigevent provided? */

  if (evp)
    {
      /* Yes, copy the entire struct sigevent content */

      memcpy(&ret->pt_event, evp, sizeof(struct sigevent));
    }
  else
    {
      /* "If the evp argument is NULL, the effect is as if the evp argument
       *  pointed to a sigevent structure with the sigev_notify member
       *  having the value SIGEV_SIGNAL, the sigev_signo having a default
       *  signal number, and the sigev_value member having the value of the
       *  timer ID."
       */

      ret->pt_event.sigev_notify            = SIGEV_SIGNAL;
      ret->pt_event.sigev_signo             = SIGALRM;
      ret->pt_event.sigev_value.sival_ptr   = ret;

#ifdef CONFIG_SIG_EVTHREAD
      ret->pt_event.sigev_notify_function   = NULL;
      ret->pt_event.sigev_notify_attributes = NULL;
#endif
    }

  /* Return the timer */

  *timerid = ret;
  return OK;
}

#endif /* CONFIG_DISABLE_POSIX_TIMERS */
