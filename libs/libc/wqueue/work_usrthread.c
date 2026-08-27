/****************************************************************************
 * libs/libc/wqueue/work_usrthread.c
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
#include <unistd.h>
#include <pthread.h>
#include <sched.h>
#include <stdlib.h>
#include <errno.h>
#include <assert.h>

#include <nuttx/clock.h>
#include <nuttx/list.h>
#include <nuttx/wqueue.h>

#include "wqueue/wqueue.h"

#if defined(CONFIG_LIBC_USRWORK) && !defined(__KERNEL__)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define WORK_DELAY_MAX UINT64_MAX

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* The state of the user mode work queue. */

static struct usr_worker_s g_usrworker =
{
  0,
  NULL,
  &g_usrwork,
  SEM_INITIALIZER(0),
  0,
};

struct usr_wqueue_s g_usrwork =
{
  LIST_INITIAL_VALUE(g_usrwork.q),
  NXMUTEX_INITIALIZER,
  SEM_INITIALIZER(0),
  &g_usrworker,
  1,
  false,
  false,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: work_pthread
 *
 * Description:
 *   This is the worker thread that performs the actions placed on the user
 *   work queue.
 *
 *   This is a user-mode work queue.  The predefined queue is started by
 *   application start-up logic through work_usrstart(); custom queues are
 *   started by work_queue_create().
 *
 * Input Parameters:
 *   arg - Describes this worker and its parent queue
 *
 * Returned Value:
 *   NULL
 *
 ****************************************************************************/

static pthread_addr_t work_pthread(pthread_addr_t arg)
{
  FAR struct usr_worker_s *worker =
    (FAR struct usr_worker_s *)arg;
  FAR struct usr_wqueue_s *wqueue = worker->wqueue;
  FAR struct work_s *work;
  worker_t callback;
  FAR void *work_arg;
  clock_t tick;
  clock_t next;
  int ret;

  for (; ; )
    {
      /* Then process queued work.  Lock the work queue while we process
       * items in the work list.
       */

      next = WORK_DELAY_MAX;
      ret = nxmutex_lock(&wqueue->lock);
      if (ret < 0)
        {
          /* Restart if we were awakened by a signal. */

          continue;
        }

      if (wqueue->exit)
        {
          nxmutex_unlock(&wqueue->lock);
          break;
        }

      /* And check each entry in the work queue.  Since we have locked the
       * work queue we know:  (1) we will not be suspended unless we do
       * so ourselves, and (2) there will be no changes to the work queue
       */

      while (!list_is_empty(&wqueue->q))
        {
          work = list_first_entry(&wqueue->q, struct work_s, node);

          /* Is this work ready? It is ready if there is no delay or if
           * the delay has elapsed.  is the time that the work was added
           * to the work queue. Therefore a delay of equal or less than
           * zero will always execute immediately.
           */

          tick = clock();

          /* Is this delay work ready? */

          if (clock_compare(work->qtime, tick))
            {
              /* Remove the ready-to-execute work from the list */

              list_delete(&work->node);

              /* Extract the work description from the entry (in case the
               * work instance may be reused after it has been de-queued).
               */

              callback = work->worker;

              /* Check for a race condition where the work may be nullified
               * before it is removed from the queue.
               */

              if (callback != NULL)
                {
                  /* Extract the work argument before unlocking the queue. */

                  work_arg = work->arg;

                  /* Mark the work as no longer being queued */

                  work->worker = NULL;
                  worker->work = work;

                  /* Let another worker process the next ready entry. */

                  if (!list_is_empty(&wqueue->q))
                    {
                      FAR struct work_s *next_work =
                        list_first_entry(&wqueue->q, struct work_s, node);

                      if (clock_compare(next_work->qtime, tick))
                        {
                          work_wake(wqueue);
                        }
                    }

                  /* Do the work.  Unlock the work queue while the work is
                   * being performed... we don't have any idea how long
                   * this will take!
                   */

                  nxmutex_unlock(&wqueue->lock);
                  callback(work_arg);

                  /* Now, unfortunately, since we unlocked the work queue
                   * we don't know the state of the work list and we will
                   * have to start back at the head of the list.
                   */

                  do
                    {
                      ret = nxmutex_lock(&wqueue->lock);
                    }
                  while (ret < 0);

                  worker->work = NULL;

                  while (worker->wait_count > 0)
                    {
                      worker->wait_count--;
                      nxsem_post(&worker->wait);
                    }

                  if (wqueue->exit)
                    {
                      nxmutex_unlock(&wqueue->lock);
                      return NULL;
                    }
                }
            }
          else
            {
              next = work->qtime - tick;
              break;
            }
        }

      /* Unlock the work queue before waiting. */

      nxmutex_unlock(&wqueue->lock);

      if (next == WORK_DELAY_MAX)
        {
          /* Wait indefinitely until work_queue has new items */

          nxsem_wait(&wqueue->wake);
        }
      else
        {
          struct timespec now;
          struct timespec delay;
          struct timespec rqtp;

          /* Wait awhile to check the work list.  We will wait here until
           * either the time elapses or until we are awakened by a semaphore.
           * Interrupts will be re-enabled while we wait.
           */

          clock_gettime(CLOCK_REALTIME, &now);
          clock_ticks2time(&delay, next);
          clock_timespec_add(&now, &delay, &rqtp);

          nxsem_timedwait(&wqueue->wake, &rqtp);
        }
    }

  return NULL;
}

#ifdef CONFIG_BUILD_PROTECTED
static int work_usrtask(int argc, char *argv[])
{
  work_pthread(&g_usrworker);
  return OK;
}
#endif

#ifndef CONFIG_DISABLE_PTHREAD
/****************************************************************************
 * Name: work_thread_create
 *
 * Description:
 *   Create the worker threads for a dynamically allocated user work queue.
 *
 ****************************************************************************/

static int work_thread_create(FAR const char *name, int priority,
                              FAR void *stack_addr, int stack_size,
                              FAR struct usr_wqueue_s *wqueue)
{
  pthread_attr_t attr;
  struct sched_param param;
  int created = 0;
  int lockret;
  int ret;
  int wndx;

  ret = pthread_attr_init(&attr);
  if (ret != 0)
    {
      return -ret;
    }

  ret = pthread_attr_setstacksize(&attr, stack_size);
  if (ret != 0)
    {
      goto errout_with_attr;
    }

  ret = pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
  if (ret != 0)
    {
      goto errout_with_attr;
    }

  ret = pthread_attr_getschedparam(&attr, &param);
  if (ret != 0)
    {
      goto errout_with_attr;
    }

  param.sched_priority = priority;
  ret = pthread_attr_setschedparam(&attr, &param);
  if (ret != 0)
    {
      goto errout_with_attr;
    }

  ret = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
  if (ret != 0)
    {
      goto errout_with_attr;
    }

  for (wndx = 0; wndx < wqueue->nthreads; wndx++)
    {
      FAR struct usr_worker_s *worker = &wqueue->worker[wndx];

      if (stack_addr != NULL)
        {
          FAR void *stack = (FAR void *)
            ((uintptr_t)stack_addr + wndx * stack_size);

          ret = pthread_attr_setstack(&attr, stack, stack_size);
          if (ret != 0)
            {
              goto errout_with_threads;
            }
        }

      ret = pthread_create(&worker->tid, &attr, work_pthread, worker);
      if (ret != 0)
        {
          goto errout_with_threads;
        }

      created++;
      pthread_setname_np(worker->tid, name);
    }

  pthread_attr_destroy(&attr);
  return OK;

errout_with_threads:
  do
    {
      lockret = nxmutex_lock(&wqueue->lock);
    }
  while (lockret < 0);

  wqueue->exit = true;
  nxmutex_unlock(&wqueue->lock);

  for (wndx = 0; wndx < created; wndx++)
    {
      nxsem_post(&wqueue->wake);
    }

  for (wndx = 0; wndx < created; wndx++)
    {
      pthread_join(wqueue->worker[wndx].tid, NULL);
    }

errout_with_attr:
  pthread_attr_destroy(&attr);
  return -ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: work_queue_create
 *
 * Description:
 *   Create a user-mode custom work queue.  This function must only be
 *   called from task context.
 *
 ****************************************************************************/

FAR struct kwork_wqueue_s *work_queue_create(FAR const char *name,
                                             int priority,
                                             FAR void *stack_addr,
                                             int stack_size, int nthreads)
{
  FAR struct usr_wqueue_s *wqueue;
  size_t allocsize;
  int ret;
  int wndx;

  if (name == NULL || stack_size <= 0 || nthreads < 1 ||
      nthreads > (SIZE_MAX - sizeof(*wqueue)) / sizeof(struct usr_worker_s))
    {
      return NULL;
    }

  allocsize = sizeof(*wqueue) +
              nthreads * sizeof(struct usr_worker_s);
  wqueue = calloc(1, allocsize);
  if (wqueue == NULL)
    {
      return NULL;
    }

  list_initialize(&wqueue->q);
  nxmutex_init(&wqueue->lock);
  nxsem_init(&wqueue->wake, 0, 0);
  wqueue->worker = (FAR struct usr_worker_s *)(wqueue + 1);
  wqueue->nthreads = nthreads;
  wqueue->dynamic = true;

  for (wndx = 0; wndx < nthreads; wndx++)
    {
      wqueue->worker[wndx].wqueue = wqueue;
      nxsem_init(&wqueue->worker[wndx].wait, 0, 0);
    }

  ret = work_thread_create(name, priority, stack_addr, stack_size, wqueue);
  if (ret < 0)
    {
      for (wndx = 0; wndx < nthreads; wndx++)
        {
          nxsem_destroy(&wqueue->worker[wndx].wait);
        }

      nxsem_destroy(&wqueue->wake);
      nxmutex_destroy(&wqueue->lock);
      free(wqueue);
      return NULL;
    }

  return (FAR struct kwork_wqueue_s *)wqueue;
}

/****************************************************************************
 * Name: work_queue_free
 *
 * Description:
 *   Destroy a user-mode custom work queue.  This function must only be
 *   called from task context and must not be called by one of the queue's
 *   own worker threads.
 *
 ****************************************************************************/

int work_queue_free(FAR struct kwork_wqueue_s *handle)
{
  FAR struct usr_wqueue_s *wqueue = (FAR struct usr_wqueue_s *)handle;
  FAR struct work_s *work;
  FAR struct work_s *next;
  pid_t self = gettid();
  int ret;
  int wndx;

  if (wqueue == NULL || !wqueue->dynamic)
    {
      return -EINVAL;
    }

  for (wndx = 0; wndx < wqueue->nthreads; wndx++)
    {
      if (self == wqueue->worker[wndx].tid)
        {
          return -EDEADLK;
        }
    }

  do
    {
      ret = nxmutex_lock(&wqueue->lock);
    }
  while (ret < 0);

  wqueue->exit = true;

  list_for_every_entry_safe(&wqueue->q, work, next, struct work_s, node)
    {
      list_delete(&work->node);
      work->worker = NULL;
    }

  nxmutex_unlock(&wqueue->lock);

  for (wndx = 0; wndx < wqueue->nthreads; wndx++)
    {
      nxsem_post(&wqueue->wake);
    }

  for (wndx = 0; wndx < wqueue->nthreads; wndx++)
    {
      pthread_join(wqueue->worker[wndx].tid, NULL);
      nxsem_destroy(&wqueue->worker[wndx].wait);
    }

  nxsem_destroy(&wqueue->wake);
  nxmutex_destroy(&wqueue->lock);
  free(wqueue);
  return OK;
}
#endif

/****************************************************************************
 * Name: work_queue_priority_wq
 ****************************************************************************/

int work_queue_priority_wq(FAR struct kwork_wqueue_s *handle)
{
  FAR struct usr_wqueue_s *wqueue = (FAR struct usr_wqueue_s *)handle;
  struct sched_param param;
  int ret;

  if (wqueue == NULL || wqueue->nthreads < 1)
    {
      return -EINVAL;
    }

  ret = sched_getparam(wqueue->worker[0].tid, &param);
  return ret == OK ? param.sched_priority : -get_errno();
}

int work_queue_priority(int qid)
{
  if (qid != USRWORK)
    {
      return -EINVAL;
    }

  return work_queue_priority_wq((FAR struct kwork_wqueue_s *)&g_usrwork);
}

/****************************************************************************
 * Name: work_usrstart
 *
 * Description:
 *   Start the predefined user work queue.
 *
 ****************************************************************************/

int work_usrstart(void)
{
  int ret;
#ifndef CONFIG_BUILD_PROTECTED
  pthread_attr_t attr;
  struct sched_param param;
#endif

#ifdef CONFIG_BUILD_PROTECTED
  ret = task_create("uwork",
                    CONFIG_LIBC_USRWORKPRIORITY,
                    CONFIG_LIBC_USRWORKSTACKSIZE,
                    work_usrtask, NULL);
  if (ret < 0)
    {
      int errcode = get_errno();

      DEBUGASSERT(errcode > 0);
      return -errcode;
    }

  g_usrworker.tid = ret;
#else
  pthread_attr_init(&attr);
  pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
  pthread_attr_setstacksize(&attr, CONFIG_LIBC_USRWORKSTACKSIZE);
  pthread_attr_getschedparam(&attr, &param);
  param.sched_priority = CONFIG_LIBC_USRWORKPRIORITY;
  pthread_attr_setschedparam(&attr, &param);

  ret = pthread_create(&g_usrworker.tid, &attr, work_pthread,
                       &g_usrworker);
  pthread_attr_destroy(&attr);
  if (ret != 0)
    {
      return -ret;
    }
#endif

  return g_usrworker.tid;
}

#endif /* CONFIG_LIBC_USRWORK && !__KERNEL__ */
