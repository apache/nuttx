/****************************************************************************
 * sched/timer/timer_initialize.c
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
#include <nuttx/compiler.h>

#include <sys/types.h>
#include <time.h>
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/queue.h>
#include <nuttx/trace.h>

#include "timer/timer.h"

#ifndef CONFIG_DISABLE_POSIX_TIMERS

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* These are the preallocated times */

#if CONFIG_PREALLOC_TIMERS > 0
static struct posix_timer_s g_prealloctimers[CONFIG_PREALLOC_TIMERS];
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

#if CONFIG_PREALLOC_TIMERS > 0
/* This is a list of free, preallocated timer structures */

volatile sq_queue_t g_freetimers;
#endif

/* This is a list of instantiated timer structures -- active and inactive.
 * The timers are place on this list by timer_create() and removed from the
 * list by timer_delete() or when the owning thread exits.
 */

volatile sq_queue_t g_alloctimers;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: timer_initialize
 *
 * Description:
 *   Boot up configuration of the POSIX timer facility.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void timer_initialize(void)
{
  sched_trace_begin();

#if CONFIG_PREALLOC_TIMERS > 0
  int i;

  /* Place all of the pre-allocated timers into the free timer list */

  sq_init((FAR sq_queue_t *)&g_freetimers);

  for (i = 0; i < CONFIG_PREALLOC_TIMERS; i++)
    {
      g_prealloctimers[i].pt_flags = PT_FLAGS_PREALLOCATED;
      sq_addlast((FAR sq_entry_t *)&g_prealloctimers[i],
                 (FAR sq_queue_t *)&g_freetimers);
    }
#endif

  /* Initialize the list of allocated timers */

  sq_init((FAR sq_queue_t *)&g_alloctimers);
  sched_trace_end();
}

/****************************************************************************
 * Name: timer_deleteall
 *
 * Description:
 *   This function is called whenever a thread exits.  Any timers owned by
 *   that thread are deleted as though called by timer_delete().
 *
 *   It is provided in this file so that it can be weakly defined but also,
 *   like timer_intitialize(), be brought into the link whenever the timer
 *   resources are referenced.
 *
 * Input Parameters:
 *   pid - the task ID of the thread that exited
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void timer_deleteall(pid_t pid)
{
  FAR struct posix_timer_s *timer;
  FAR struct posix_timer_s *next;
  irqstate_t flags;

  flags = enter_critical_section();
  for (timer = (FAR struct posix_timer_s *)g_alloctimers.head;
       timer != NULL;
       timer = next)
    {
      next = timer->flink;
      if (timer->pt_owner == pid)
        {
          timer_delete((timer_t)timer);
        }
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: timer_gethandle
 *
 * Description:
 *   Returns the posix timer in the activity from the corresponding timerid
 *
 * Input Parameters:
 *   timerid - The pre-thread timer, previously created by the call to
 *     timer_create(), to be be set.
 *
 * Returned Value:
 *   On success, timer_gethandle() returns pointer to the posix_timer_s;
 *   On error, NULL is returned.
 *
 ****************************************************************************/

FAR struct posix_timer_s *timer_gethandle(timer_t timerid)
{
  FAR struct posix_timer_s *timer = NULL;
  FAR sq_entry_t *entry;
  irqstate_t intflags;

  if (timerid != NULL)
    {
      intflags = enter_critical_section();

      sq_for_every(&g_alloctimers, entry)
        {
          if (entry == timerid)
            {
              timer = (FAR struct posix_timer_s *)timerid;
              break;
            }
        }

      leave_critical_section(intflags);
    }

  return timer;
}

#endif /* CONFIG_DISABLE_POSIX_TIMERS */
