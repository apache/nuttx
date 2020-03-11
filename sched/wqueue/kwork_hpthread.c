/****************************************************************************
 * sched/wqueue/work_hpthread.c
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

#include <unistd.h>
#include <sched.h>
#include <string.h>
#include <errno.h>
#include <queue.h>
#include <debug.h>

#include <nuttx/wqueue.h>
#include <nuttx/kthread.h>
#include <nuttx/kmalloc.h>
#include <nuttx/clock.h>

#include "wqueue/wqueue.h"

#ifdef CONFIG_SCHED_HPWORK

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* The state of the kernel mode, high priority work queue(s). */

struct hp_wqueue_s g_hpwork;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: work_hpthread
 *
 * Description:
 *   These are the worker threads that performs the actions placed on the
 *   high priority work queue.
 *
 *   These, along with the lower priority worker thread(s) are the kernel
 *   mode work queues (also build in the flat build).  One of these threads
 *   also performs periodic garbage collection (that would otherwise be
 *   performed by the idle thread if CONFIG_SCHED_WORKQUEUE is not defined).
 *   That will be the higher priority worker thread only if a lower priority
 *   worker thread is available.
 *
 *   All kernel mode worker threads are started by the OS during normal
 *   bring up.  This entry point is referenced by OS internally and should
 *   not be accessed by application logic.
 *
 * Input Parameters:
 *   argc, argv (not used)
 *
 * Returned Value:
 *   Does not return
 *
 ****************************************************************************/

static int work_hpthread(int argc, char *argv[])
{
#if CONFIG_SCHED_HPNTHREADS > 1
  int wndx;
  pid_t me = getpid();
  int i;

  /* Find out thread index by search the workers in g_hpwork */

  for (wndx = 0, i = 0; i < CONFIG_SCHED_HPNTHREADS; i++)
    {
      if (g_hpwork.worker[i].pid == me)
        {
          wndx = i;
          break;
        }
    }

  DEBUGASSERT(i < CONFIG_SCHED_HPNTHREADS);
#endif

  /* Loop forever */

  for (; ; )
    {
#if CONFIG_SCHED_HPNTHREADS > 1
      /* Thread 0 is special.  Only thread 0 performs period garbage collection */

      if (wndx > 0)
        {
          /* The other threads will perform work, waiting indefinitely until
           * signalled for the next work availability.
           */

          work_process((FAR struct kwork_wqueue_s *)&g_hpwork, wndx);
        }
      else
#endif
        {
#ifndef CONFIG_SCHED_LPWORK
          /* First, perform garbage collection.  This cleans-up memory
           * de-allocations that were queued because they could not be freed
           * in that execution context (for example, if the memory was freed
           * from an interrupt handler).
           *
           * NOTE: If the work thread is disabled, this clean-up is
           * performed by the IDLE thread (at a very, very low priority).
           * If the low-priority work thread is enabled, then the garbage
           * collection is done on that thread instead.
           */

          sched_garbage_collection();
#endif

          /* Then process queued work.  work_process will not return until:
           * (1) there is no further work in the work queue, and (2) signal
           * is triggered, or delayed work expires.
           */

          work_process((FAR struct kwork_wqueue_s *)&g_hpwork, 0);
        }
    }

  return OK; /* To keep some compilers happy */
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: work_hpstart
 *
 * Description:
 *   Start the high-priority, kernel-mode worker thread(s)
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   The task ID of the worker thread is returned on success.  A negated
 *   errno value is returned on failure.
 *
 ****************************************************************************/

int work_hpstart(void)
{
  pid_t pid;
  int wndx;

  /* Don't permit any of the threads to run until we have fully initialized
   * g_hpwork.
   */

  sched_lock();

  /* Start the high-priority, kernel mode worker thread(s) */

  sinfo("Starting high-priority kernel worker thread(s)\n");

  for (wndx = 0; wndx < CONFIG_SCHED_HPNTHREADS; wndx++)
    {
      pid = kthread_create(HPWORKNAME, CONFIG_SCHED_HPWORKPRIORITY,
                           CONFIG_SCHED_HPWORKSTACKSIZE,
                           (main_t)work_hpthread,
                           (FAR char * const *)NULL);

      DEBUGASSERT(pid > 0);
      if (pid < 0)
        {
          serr("ERROR: kthread_create %d failed: %d\n", wndx, (int)pid);
          sched_unlock();
          return (int)pid;
        }

      g_hpwork.worker[wndx].pid  = pid;
      g_hpwork.worker[wndx].busy = true;
    }

  sched_unlock();
  return g_hpwork.worker[0].pid;
}

#endif /* CONFIG_SCHED_HPWORK */
