/****************************************************************************
 * sched/wqueue/kwork_thread.c
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

#include <unistd.h>
#include <sched.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/list.h>
#include <nuttx/wqueue.h>
#include <nuttx/kthread.h>
#include <nuttx/semaphore.h>
#include <nuttx/sched.h>

#include "sched/sched.h"
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
         clock_t start; \
         clock_t elapsed; \
         start = perf_gettime(); \
         worker(arg); \
         elapsed = perf_gettime() - start; \
         if (elapsed > CONFIG_SCHED_CRITMONITOR_MAXTIME_WQUEUE) \
           { \
             CRITMONITOR_PANIC("WORKER %p execute too long %ju\n", \
                               worker, (uintmax_t)elapsed); \
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
  {
    LIST_INITIAL_VALUE(g_hpwork.wq.expired),
    LIST_INITIAL_VALUE(g_hpwork.wq.pending),
    SEM_INITIALIZER(0),
    SEM_INITIALIZER(0),
    SP_UNLOCKED,
    CONFIG_SCHED_HPNTHREADS,
  }
};

#endif /* CONFIG_SCHED_HPWORK */

#if defined(CONFIG_SCHED_LPWORK)
/* The state of the kernel mode, low priority work queue(s). */

struct lp_wqueue_s g_lpwork =
{
  {
    LIST_INITIAL_VALUE(g_lpwork.wq.expired),
    LIST_INITIAL_VALUE(g_lpwork.wq.pending),
    SEM_INITIALIZER(0),
    SEM_INITIALIZER(0),
    SP_UNLOCKED,
    CONFIG_SCHED_LPNTHREADS,
  }
};

#endif /* CONFIG_SCHED_LPWORK */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline_function
void work_dispatch(FAR struct kwork_wqueue_s *wq)
{
  FAR struct work_s *work;
  FAR struct work_s *next;
  unsigned int count = 0;
  clock_t      ticks = clock_systime_ticks();

  /* Wake up the worker thread once there is expired work.
   * If some worker threads are busy, here the callback will
   * wake up another waiting work thread.
   *
   * Becareful of the special case that the pending work
   * has been canceled but the timer is expired.
   * In this case we should not wake up any worker thread.
   */

  list_for_every_entry_safe(&wq->pending, work, next, struct work_s, node)
    {
      /* Check whether the work has expired. */

      if (!clock_compare(work->qtime, ticks))
        {
          wd_start_abstick(&wq->timer, work->qtime,
                           work_timer_expired, (wdparm_t)wq);
          break;
        }

      /* Expired work will be moved to tail of the expired queue. */

      list_delete(&work->node);
      list_add_tail(&wq->expired, &work->node);

      /* Note that the thread execution this function is also
       * a worker thread, which has already been woken up by the timer.
       * So only `count - 1` semaphore will be posted.
       */

      if (count++ > 0)
        {
          nxsem_post(&wq->sem);
        }
    }
}

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
  FAR struct kworker_s      *kworker;
  FAR struct work_s         *work;
  worker_t      worker;
  irqstate_t    flags;
  FAR void     *arg;

  /* Get the handle from argv */

  wqueue  = (FAR struct kwork_wqueue_s *)
            ((uintptr_t)strtoul(argv[1], NULL, 16));
  kworker = (FAR struct kworker_s *)
            ((uintptr_t)strtoul(argv[2], NULL, 16));

  /* Loop until wqueue->exit != 0.
   * Since the only way to set wqueue->exit is to call work_queue_free(),
   * there is no need for entering the critical section.
   */

  while (!wqueue->exit)
    {
      /* And check first entry in the work queue. Since we have disabled
       * interrupts we know:  (1) we will not be suspended unless we do
       * so ourselves, and (2) there will be no changes to the work queue
       */

      flags = spin_lock_irqsave(&wqueue->lock);
      sched_lock();

      /* If the wqueue timer is expired and non-active, it indicates that
       * there might be expired work in the pending queue.
       */

      if (!WDOG_ISACTIVE(&wqueue->timer))
        {
          work_dispatch(wqueue);
        }

      if (!list_is_empty(&wqueue->expired))
        {
          work = list_first_entry(&wqueue->expired, struct work_s, node);

          list_delete(&work->node);

          /* Extract the work description from the entry (in case the
           * work instance will be reused after it has been de-queued).
           */

          worker = work->worker;

          /* Extract the work argument (before re-enabling interrupts) */

          arg = work->arg;

          /* Return the work structure ownership to the work owner. */

          work->worker = NULL;

          /* Mark the thread busy */

          kworker->work = work;

          spin_unlock_irqrestore(&wqueue->lock, flags);
          sched_unlock();

          /* Do the work.  Re-enable interrupts while the work is being
           * performed... we don't have any idea how long this will take!
           */

          CALL_WORKER(worker, arg);
          flags = spin_lock_irqsave(&wqueue->lock);
          sched_lock();

          /* Mark the thread un-busy */

          kworker->work = NULL;

          /* Check if someone is waiting, if so, wakeup it */

          while (kworker->wait_count > 0)
            {
              kworker->wait_count--;
              nxsem_post(&kworker->wait);
            }
        }

      spin_unlock_irqrestore(&wqueue->lock, flags);
      sched_unlock();

      /* Wait for the semaphore to be posted by the wqueue timer. */

      nxsem_wait_uninterruptible(&wqueue->sem);
    }

  nxsem_post(&wqueue->exsem);
  return OK;
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
 *   stack_addr - Stack buffer of the new task
 *   stack_size - size (in bytes) of the stack needed
 *   wqueue     - Work queue instance
 *
 * Returned Value:
 *   A negated errno value is returned on failure.
 *
 ****************************************************************************/

static int work_thread_create(FAR const char *name, int priority,
                              FAR void *stack_addr, int stack_size,
                              FAR struct kwork_wqueue_s *wqueue)
{
  FAR struct kworker_s *worker = wq_get_worker(wqueue);
  FAR char *argv[3];
  char arg0[32];
  char arg1[32];
  int wndx;
  int pid;

  /* Don't permit any of the threads to run until we have fully initialized
   * all of them.
   */

  sched_lock();

  for (wndx = 0; wndx < wqueue->nthreads; wndx++)
    {
      nxsem_init(&worker[wndx].wait, 0, 0);

      snprintf(arg0, sizeof(arg0), "%p", wqueue);
      snprintf(arg1, sizeof(arg1), "%p", &worker[wndx]);
      argv[0] = arg0;
      argv[1] = arg1;
      argv[2] = NULL;

      pid = kthread_create_with_stack(name, priority, stack_addr, stack_size,
                                      work_thread, argv);

      DEBUGASSERT(pid > 0);
      if (pid < 0)
        {
          serr("ERROR: work_thread_create %d failed: %d\n", wndx, pid);
          sched_unlock();
          return pid;
        }

      worker[wndx].pid = pid;
    }

  sched_unlock();
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: work_timer_expired
 *
 * Description:
 *   The wqueue timer callback.
 *
 * Input Parameters:
 *   arg  - The work queue.
 *
 ****************************************************************************/

void work_timer_expired(wdparm_t arg)
{
  /* The work time expired callback will wake up at least one worker thread
   * to dispatch the expired work.
   */

  FAR struct kwork_wqueue_s *wq = (FAR struct kwork_wqueue_s *)arg;
  nxsem_post(&wq->sem);
}

/****************************************************************************
 * Name: work_queue_create
 *
 * Description:
 *   Create a new work queue. The work queue is identified by its work
 *   queue ID, which is used to queue works to the work queue and to
 *   perform other operations on the work queue.
 *   This function will create a work thread pool with nthreads threads.
 *   The work queue ID is returned on success.
 *
 * Input Parameters:
 *   name       - Name of the new task
 *   priority   - Priority of the new task
 *   stack_addr - Stack buffer of the new task
 *   stack_size - size (in bytes) of the stack needed
 *   nthreads   - Number of work thread should be created
 *
 * Returned Value:
 *   The work queue handle returned on success.  Otherwise, NULL
 *
 ****************************************************************************/

FAR struct kwork_wqueue_s *work_queue_create(FAR const char *name,
                                             int priority,
                                             FAR void *stack_addr,
                                             int stack_size, int nthreads)
{
  FAR struct kwork_wqueue_s *wqueue;
  int ret;

  if (nthreads < 1)
    {
      return NULL;
    }

  /* Allocate a new work queue */

  wqueue = kmm_zalloc(sizeof(struct kwork_wqueue_s) +
                      nthreads * sizeof(struct kworker_s));
  if (wqueue == NULL)
    {
      return NULL;
    }

  /* Initialize the work queue structure */

  list_initialize(&wqueue->expired);
  list_initialize(&wqueue->pending);
  wqueue->timer.func = NULL;
  nxsem_init(&wqueue->sem, 0, 0);
  nxsem_init(&wqueue->exsem, 0, 0);
  wqueue->nthreads = nthreads;
  spin_lock_init(&wqueue->lock);

  /* Create the work queue thread pool */

  ret = work_thread_create(name, priority, stack_addr, stack_size, wqueue);
  if (ret < 0)
    {
      kmm_free(wqueue);
      return NULL;
    }

  return wqueue;
}

/****************************************************************************
 * Name: work_queue_free
 *
 * Description:
 *   Destroy a work queue. The work queue is identified by its work queue ID.
 *   All worker threads will be destroyed and the work queue will be freed.
 *   The work queue ID is invalid after this function returns.
 *
 * Input Parameters:
 *  qid - The work queue ID
 *
 * Returned Value:
 *   Zero on success, a negated errno value on failure.
 *
 ****************************************************************************/

int work_queue_free(FAR struct kwork_wqueue_s *wqueue)
{
  int wndx;

  if (wqueue == NULL)
    {
      return -EINVAL;
    }

  wd_cancel(&wqueue->timer);

  /* Mark the work queue as exiting */

  wqueue->exit = true;

  /* Queue a exit work for all threads */

  for (wndx = 0; wndx < wqueue->nthreads; wndx++)
    {
      nxsem_post(&wqueue->sem);
    }

  for (wndx = 0; wndx < wqueue->nthreads; wndx++)
    {
      nxsem_wait_uninterruptible(&wqueue->exsem);
    }

  nxsem_destroy(&wqueue->sem);
  nxsem_destroy(&wqueue->exsem);
  kmm_free(wqueue);

  return OK;
}

/****************************************************************************
 * Name: work_queue_priority_wq
 *
 * Description: Get priority of the wqueue. We believe that all worker
 *   threads have the same priority.
 *
 * Input Parameters:
 *  wqueue - The work queue handle
 *
 * Returned Value:
 *   SCHED_PRIORITY_MIN ~ SCHED_PRIORITY_MAX  on success,
 *   a negated errno value on failure.
 *
 ****************************************************************************/

int work_queue_priority_wq(FAR struct kwork_wqueue_s *wqueue)
{
  FAR struct kworker_s *worker;
  FAR struct tcb_s *tcb;

  if (wqueue == NULL)
    {
      return -EINVAL;
    }

  /* Find for the TCB associated with matching PID */

  worker = wq_get_worker(wqueue);

  tcb = nxsched_get_tcb(worker[0].pid);

  if (!tcb)
    {
      return -ESRCH;
    }

  return tcb->sched_priority;
}

int work_queue_priority(int qid)
{
  return work_queue_priority_wq(work_qid2wq(qid));
}

/****************************************************************************
 * Name: work_start_highpri
 *
 * Description:
 *   Start the high-priority, kernel-mode work queue.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Return zero (OK) on success.  A negated errno value is returned on
 *   errno value is returned on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_SCHED_HPWORK
int work_start_highpri(void)
{
  /* Start the high-priority, kernel mode worker thread(s) */

  sinfo("Starting high-priority kernel worker thread(s)\n");

  return work_thread_create(HPWORKNAME, CONFIG_SCHED_HPWORKPRIORITY, NULL,
                            CONFIG_SCHED_HPWORKSTACKSIZE,
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
 *   Return zero (OK) on success.  A negated errno value is returned on
 *   errno value is returned on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_SCHED_LPWORK
int work_start_lowpri(void)
{
  /* Start the low-priority, kernel mode worker thread(s) */

  sinfo("Starting low-priority kernel worker thread(s)\n");

  return work_thread_create(LPWORKNAME, CONFIG_SCHED_LPWORKPRIORITY, NULL,
                            CONFIG_SCHED_LPWORKSTACKSIZE,
                            (FAR struct kwork_wqueue_s *)&g_lpwork);
}
#endif /* CONFIG_SCHED_LPWORK */

#endif /* CONFIG_SCHED_WORKQUEUE */
