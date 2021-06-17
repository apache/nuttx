/****************************************************************************
 * sched/wqueue/kwork_thread.c
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
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <queue.h>
#include <debug.h>

#include <nuttx/wqueue.h>
#include <nuttx/kthread.h>

#include "wqueue/wqueue.h"

#if defined(CONFIG_SCHED_WORKQUEUE)

/****************************************************************************
 * Public Data
 ****************************************************************************/

#if defined(CONFIG_SCHED_HPWORK)
/* The state of the kernel mode, high priority work queue(s). */

struct hp_wqueue_s g_hpwork;
#endif /* CONFIG_SCHED_HPWORK */

#if defined(CONFIG_SCHED_LPWORK)
/* The state of the kernel mode, low priority work queue(s). */

struct lp_wqueue_s g_lpwork;
#endif /* CONFIG_SCHED_LPWORK */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: work_thread
 *
 * Description:
 *   These are the worker threads that performs the actions placed on the
 *   high priority work queue.
 *
 *   These, along with the lower priority worker thread(s) are the kernel
 *   mode work queues (also build in the flat build).
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

static int work_thread(int argc, char *argv[])
{
  FAR struct kwork_wqueue_s *queue;
  FAR struct kworker_s      *worker;

  queue  = (FAR struct kwork_wqueue_s *)
           ((uintptr_t)strtoul(argv[1], NULL, 0));
  worker = (FAR struct kworker_s *)((uintptr_t)strtoul(argv[2], NULL, 0));

  /* Loop forever */

  for (; ; )
    {
      /* Then process queued work.  work_process will not return until: (1)
       * there is no further work in the work queue, and (2) signal is
       * triggered, or delayed work expires.
       */

      work_process(queue, worker);
    }

  return OK; /* To keep some compilers happy */
}

/****************************************************************************
 * Name: work_thread_create
 *
 * Description:
 *   This function creates and activates a work thread task with kernel-
 *   mode privileges.
 *
 * Input Parameters:
 *   name       - Name of the new task
 *   priority   - Priority of the new task
 *   stack_size - size (in bytes) of the stack needed
 *   nthread    - Number of work thread should be created
 *   wqueue     - Work queue instance
 *
 * Returned Value:
 *   A negated errno value is returned on failure.
 *
 ****************************************************************************/

static int work_thread_create(FAR const char *name, int priority,
                              int stack_size, int nthread,
                              FAR struct kwork_wqueue_s *wqueue)
{
  FAR char *argv[3];
  char args[2][16];
  int wndx;
  int pid;

  snprintf(args[0], 16, "0x%" PRIxPTR, (uintptr_t)wqueue);
  argv[0] = args[0];
  argv[2] = NULL;

  for (wndx = 0; wndx < nthread; wndx++)
    {
      nxsem_init(&wqueue->worker[wndx].sem, 0, 0);
      nxsem_set_protocol(&wqueue->worker[wndx].sem, SEM_PRIO_NONE);

      wqueue->worker[wndx].busy = true;

      snprintf(args[1], 16, "0x%" PRIxPTR, (uintptr_t)&wqueue->worker[wndx]);
      argv[1] = args[1];

      pid = kthread_create(name, priority, stack_size,
                           (main_t)work_thread, argv);

      DEBUGASSERT(pid > 0);
      if (pid < 0)
        {
          serr("ERROR: work_thread_create %d failed: %d\n", wndx, pid);
          sched_unlock();
          return pid;
        }

#ifdef CONFIG_PRIORITY_INHERITANCE
      wqueue->worker[wndx].pid  = pid;
#endif
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: work_start_highpri
 *
 * Description:
 *   Start the high-priority, kernel-mode worker thread(s)
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   A negated errno value is returned on failure.
 *
 ****************************************************************************/

#if defined(CONFIG_SCHED_HPWORK)
int work_start_highpri(void)
{
  /* Start the high-priority, kernel mode worker thread(s) */

  sinfo("Starting high-priority kernel worker thread(s)\n");

  return work_thread_create(HPWORKNAME, CONFIG_SCHED_HPWORKPRIORITY,
                            CONFIG_SCHED_HPWORKSTACKSIZE,
                            CONFIG_SCHED_HPNTHREADS,
                            (FAR struct kwork_wqueue_s *)&g_hpwork);
}
#endif /* CONFIG_SCHED_HPWORK */

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
 *   A negated errno value is returned on failure.
 *
 ****************************************************************************/

#if defined(CONFIG_SCHED_LPWORK)
int work_start_lowpri(void)
{
  /* Start the low-priority, kernel mode worker thread(s) */

  sinfo("Starting low-priority kernel worker thread(s)\n");

  return work_thread_create(LPWORKNAME, CONFIG_SCHED_LPWORKPRIORITY,
                            CONFIG_SCHED_LPWORKSTACKSIZE,
                            CONFIG_SCHED_LPNTHREADS,
                            (FAR struct kwork_wqueue_s *)&g_lpwork);
}
#endif /* CONFIG_SCHED_LPWORK */

#endif /* CONFIG_SCHED_WORKQUEUE */
