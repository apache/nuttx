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
#include <assert.h>
#include <debug.h>

#include <nuttx/queue.h>
#include <nuttx/wqueue.h>
#include <nuttx/kthread.h>
#include <nuttx/semaphore.h>

#include "wqueue/wqueue.h"

#if defined(CONFIG_SCHED_WORKQUEUE)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_SCHED_CRITMONITOR_MAXTIME_WQUEUE
#  define CONFIG_SCHED_CRITMONITOR_MAXTIME_WQUEUE 0
#endif

#if CONFIG_SCHED_CRITMONITOR_MAXTIME_WQUEUE > 0
#  define CALL_WORKER(worker, arg) \
     do \
       { \
         uint32_t start; \
         uint32_t elapsed; \
         start = up_perf_gettime(); \
         worker(arg); \
         elapsed = up_perf_gettime() - start; \
         if (elapsed > CONFIG_SCHED_CRITMONITOR_MAXTIME_WQUEUE) \
           { \
             serr("WORKER %p execute too long %"PRIu32"\n", \
                   worker, elapsed); \
           } \
       } \
     while (0)
#else
#  define CALL_WORKER(worker, arg) worker(arg)
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

#if defined(CONFIG_SCHED_HPWORK)
/* The state of the kernel mode, high priority work queue(s). */

struct hp_wqueue_s g_hpwork =
{
  {NULL, NULL},
  SEM_INITIALIZER(0),
};

#endif /* CONFIG_SCHED_HPWORK */

#if defined(CONFIG_SCHED_LPWORK)
/* The state of the kernel mode, low priority work queue(s). */

struct lp_wqueue_s g_lpwork =
{
  {NULL, NULL},
  SEM_INITIALIZER(0),
};

#endif /* CONFIG_SCHED_LPWORK */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: work_thread
 *
 * Description:
 *   These are the worker threads that perform the actions placed on the
 *   high priority work queue.
 *
 *   These, along with the lower priority worker thread(s) are the kernel
 *   mode work queues (also built in the flat build).
 *
 *   All kernel mode worker threads are started by the OS during normal
 *   bring up.  This entry point is referenced by OS internally and should
 *   not be accessed by application logic.
 *
 * Input Parameters:
 *   argc, argv
 *
 * Returned Value:
 *   Does not return
 *
 ****************************************************************************/

static int work_thread(int argc, FAR char *argv[])
{
  FAR struct kwork_wqueue_s *wqueue;
  FAR struct work_s *work;
  worker_t  worker;
  irqstate_t flags;
  FAR void *arg;

  wqueue = (FAR struct kwork_wqueue_s *)
           ((uintptr_t)strtoul(argv[1], NULL, 0));

  flags = enter_critical_section();

  /* Loop forever */

  for (; ; )
    {
      /* Then process queued work.  work_process will not return until: (1)
       * there is no further work in the work queue, and (2) semaphore is
       * posted.
       */

      nxsem_wait_uninterruptible(&wqueue->sem);

      /* And check each entry in the work queue.  Since we have disabled
       * interrupts we know:  (1) we will not be suspended unless we do
       * so ourselves, and (2) there will be no changes to the work queue
       */

      /* Remove the ready-to-execute work from the list */

      work = (FAR struct work_s *)dq_remfirst(&wqueue->q);
      if (work && work->worker)
        {
          /* Extract the work description from the entry (in case the work
           * instance will be re-used after it has been de-queued).
           */

          worker = work->worker;

          /* Extract the work argument (before re-enabling interrupts) */

          arg = work->arg;

          /* Mark the work as no longer being queued */

          work->worker = NULL;

          /* Do the work.  Re-enable interrupts while the work is being
           * performed... we don't have any idea how long this will take!
           */

          leave_critical_section(flags);
          CALL_WORKER(worker, arg);
          flags = enter_critical_section();
        }
    }

  leave_critical_section(flags);

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
  FAR char *argv[2];
  char args[32];
  int wndx;
  int pid;

  snprintf(args, sizeof(args), "0x%" PRIxPTR, (uintptr_t)wqueue);
  argv[0] = args;
  argv[1] = NULL;

  /* Don't permit any of the threads to run until we have fully initialized
   * g_hpwork and g_lpwork.
   */

  sched_lock();

  for (wndx = 0; wndx < nthread; wndx++)
    {
      pid = kthread_create(name, priority, stack_size,
                           work_thread, argv);

      DEBUGASSERT(pid > 0);
      if (pid < 0)
        {
          serr("ERROR: work_thread_create %d failed: %d\n", wndx, pid);
          sched_unlock();
          return pid;
        }

      wqueue->worker[wndx].pid  = pid;
    }

  sched_unlock();
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: work_foreach
 *
 * Description:
 *   Enumerate over each work thread and provide the tid of each task to a
 *   user callback functions.
 *
 * Input Parameters:
 *   qid     - The work queue ID
 *   handler - The function to be called with the pid of each task
 *   arg     - The function callback
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void work_foreach(int qid, work_foreach_t handler, FAR void *arg)
{
  FAR struct kwork_wqueue_s *wqueue;
  int nthread;
  int wndx;

#ifdef CONFIG_SCHED_HPWORK
  if (qid == HPWORK)
    {
      wqueue  = (FAR struct kwork_wqueue_s *)&g_hpwork;
      nthread = CONFIG_SCHED_HPNTHREADS;
    }
  else
#endif
#ifdef CONFIG_SCHED_LPWORK
  if (qid == LPWORK)
    {
      wqueue  = (FAR struct kwork_wqueue_s *)&g_lpwork;
      nthread = CONFIG_SCHED_LPNTHREADS;
    }
  else
#endif
    {
      return;
    }

  for (wndx = 0; wndx < nthread; wndx++)
    {
      handler(wqueue->worker[wndx].pid, arg);
    }
}

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
