/****************************************************************************
 * sched/wqueue/work_lpthread.c
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

#ifdef CONFIG_SCHED_LPWORK

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* The state of the kernel mode, low priority work queue(s). */

struct lp_wqueue_s g_lpwork;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: work_lpthread
 *
 * Description:
 *   These are the worker thread(s) that performs the actions placed on the
 *   low priority work queue.
 *
 *   These, along with the higher priority worker thread are the kernel mode
 *   work queues (also build in the flat build).
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

static int work_lpthread(int argc, char *argv[])
{
  int wndx = 0;
#if CONFIG_SCHED_LPNTHREADS > 1
  pid_t me = getpid();
  int i;

  /* Find out thread index by search the workers in g_lpwork */

  for (wndx = 0, i = 0; i < CONFIG_SCHED_LPNTHREADS; i++)
    {
      if (g_lpwork.worker[i].pid == me)
        {
          wndx = i;
          break;
        }
    }

  DEBUGASSERT(i < CONFIG_SCHED_LPNTHREADS);
#endif

  /* Loop forever */

  for (; ; )
    {
      /* Then process queued work.  work_process will not return until:
       * (1) there is no further work in the work queue, and (2) signal is
       * triggered, or delayed work expires.
       */

      work_process((FAR struct kwork_wqueue_s *)&g_lpwork, wndx);
    }

  return OK; /* To keep some compilers happy */
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: work_start_lowpri
 *
 * Description:
 *   Start the low-priority, kernel-mode worker thread(s)
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   The task ID of the worker thread is returned on success.  A negated
 *   errno value is returned on failure.
 *
 ****************************************************************************/

int work_start_lowpri(void)
{
  pid_t pid;
  int wndx;

  /* Don't permit any of the threads to run until we have fully initialized
   * g_lpwork.
   */

  sched_lock();

  /* Start the low-priority, kernel mode worker thread(s) */

  sinfo("Starting low-priority kernel worker thread(s)\n");

  for (wndx = 0; wndx < CONFIG_SCHED_LPNTHREADS; wndx++)
    {
      pid = kthread_create(LPWORKNAME, CONFIG_SCHED_LPWORKPRIORITY,
                           CONFIG_SCHED_LPWORKSTACKSIZE,
                           (main_t)work_lpthread,
                           (FAR char * const *)NULL);

      DEBUGASSERT(pid > 0);
      if (pid < 0)
        {
          serr("ERROR: kthread_create %d failed: %d\n", wndx, (int)pid);
          sched_unlock();
          return (int)pid;
        }

      g_lpwork.worker[wndx].pid  = pid;
      g_lpwork.worker[wndx].busy = true;
    }

  sched_unlock();
  return g_lpwork.worker[0].pid;
}

#endif /* CONFIG_SCHED_LPWORK */
