/****************************************************************************
 * libs/libc/wqueue/work_usrthread.c
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
#include <signal.h>
#include <sched.h>
#include <errno.h>
#include <assert.h>
#include <queue.h>

#include <nuttx/semaphore.h>
#include <nuttx/clock.h>
#include <nuttx/wqueue.h>

#include "wqueue/wqueue.h"

#if defined(CONFIG_LIB_USRWORK) && !defined(__KERNEL__)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Use CLOCK_MONOTONIC if it is available.  CLOCK_REALTIME can cause bad
 * delays if the time is changed.
 */

#ifdef CONFIG_CLOCK_MONOTONIC
#  define WORK_CLOCK CLOCK_MONOTONIC
#else
#  define WORK_CLOCK CLOCK_REALTIME
#endif

#ifdef CONFIG_SYSTEM_TIME64
#  define WORK_DELAY_MAX UINT64_MAX
#else
#  define WORK_DELAY_MAX UINT32_MAX
#endif

#ifndef MIN
#  define MIN(a,b) ((a) < (b) ? (a) : (b))
#endif

/****************************************************************************
 * Private Type Declarations
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* The state of the user mode work queue. */

struct usr_wqueue_s g_usrwork;

/* This semaphore supports exclusive access to the user-mode work queue */

#ifdef CONFIG_BUILD_PROTECTED
sem_t g_usrsem;
#else
pthread_mutex_t g_usrmutex;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: work_process
 *
 * Description:
 *   This is the logic that performs actions placed on any work list.  This
 *   logic is the common underlying logic to all work queues.  This logic is
 *   part of the internal implementation of each work queue; it should not
 *   be called from application level logic.
 *
 * Input Parameters:
 *   wqueue - Describes the work queue to be processed
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void work_process(FAR struct usr_wqueue_s *wqueue)
{
  volatile FAR struct work_s *work;
  sigset_t sigset;
  sigset_t oldset;
  worker_t  worker;
  FAR void *arg;
  clock_t elapsed;
  clock_t remaining;
  clock_t stick;
  clock_t ctick;
  clock_t next;
  int ret;

  /* Then process queued work.  Lock the work queue while we process items
   * in the work list.
   */

  next = WORK_DELAY_MAX;
  ret = work_lock();
  if (ret < 0)
    {
      /* Break out earlier if we were awakened by a signal */

      return;
    }

  /* Set up the signal mask */

  sigemptyset(&sigset);
  sigaddset(&sigset, SIGWORK);

  /* Get the time that we started this polling cycle in clock ticks. */

  stick = clock();

  /* And check each entry in the work queue.  Since we have locked the
   * work queue we know:  (1) we will not be suspended unless we do
   * so ourselves, and (2) there will be no changes to the work queue
   */

  work = (FAR struct work_s *)wqueue->q.head;
  while (work)
    {
      /* Is this work ready?  It is ready if there is no delay or if
       * the delay has elapsed. qtime is the time that the work was added
       * to the work queue.  It will always be greater than or equal to
       * zero.  Therefore a delay of zero will always execute immediately.
       */

      ctick   = clock();
      elapsed = ctick - work->qtime;
      if (elapsed >= work->delay)
        {
          /* Remove the ready-to-execute work from the list */

          dq_rem((struct dq_entry_s *)work, &wqueue->q);

          /* Extract the work description from the entry (in case the work
           * instance by the re-used after it has been de-queued).
           */

          worker = work->worker;

          /* Check for a race condition where the work may be nullified
           * before it is removed from the queue.
           */

          if (worker != NULL)
            {
              /* Extract the work argument before unlocking the work queue */

              arg = work->arg;

              /* Mark the work as no longer being queued */

              work->worker = NULL;

              /* Do the work.  Unlock the work queue while the work is being
               * performed... we don't have any idea how long this will take!
               */

              work_unlock();
              worker(arg);

              /* Now, unfortunately, since we unlocked the work queue we
               * don't know the state of the work list and we will have to
               * start back at the head of the list.
               */

              ret = work_lock();
              if (ret < 0)
                {
                  /* Break out earlier if we were awakened by a signal */

                  return;
                }

              work = (FAR struct work_s *)wqueue->q.head;
            }
          else
            {
              /* Canceled.. Just move to the next work in the list with
               * the work queue still locked.
               */

              work = (FAR struct work_s *)work->dq.flink;
            }
        }
      else /* elapsed < work->delay */
        {
          /* This one is not ready.
           *
           * NOTE that elapsed is relative to the current time,
           * not the time of beginning of this queue processing pass.
           * So it may need an adjustment.
           */

          elapsed += (ctick - stick);
          if (elapsed > work->delay)
            {
              /* The delay has expired while we are processing */

              elapsed = work->delay;
            }

          /* Will it be ready before the next scheduled wakeup interval? */

          remaining = work->delay - elapsed;
          if (remaining < next)
            {
              /* Yes.. Then schedule to wake up when the work is ready */

              next = remaining;
            }

          /* Then try the next in the list. */

          work = (FAR struct work_s *)work->dq.flink;
        }
    }

  /* Unlock the work queue before waiting.  In order to assure that we do
   * not lose the SIGWORK signal before waiting, we block the SIGWORK
   * signals before unlocking the work queue.  That will cause in SIGWORK
   * signals directed to the worker thread to pend.
   */

  sigprocmask(SIG_BLOCK, &sigset, &oldset);
  work_unlock();

  if (next == WORK_DELAY_MAX)
    {
      /* Wait indefinitely until signaled with SIGWORK */

      sigwaitinfo(&sigset, NULL);
    }
  else
    {
      struct timespec rqtp;
      time_t sec;

      /* Wait awhile to check the work list.  We will wait here until
       * either the time elapses or until we are awakened by a signal.
       * Interrupts will be re-enabled while we wait.
       */

      sec          = next / 1000000;
      rqtp.tv_sec  = sec;
      rqtp.tv_nsec = (next - (sec * 1000000)) * 1000;

      sigtimedwait(&sigset, NULL, &rqtp);
    }

  sigprocmask(SIG_SETMASK, &oldset, NULL);
}

/****************************************************************************
 * Name: work_usrthread
 *
 * Description:
 *   This is the worker thread that performs the actions placed on the user
 *   work queue.
 *
 *   This is a user mode work queue.  It must be used by applications for
 *   miscellaneous operations.  The user work thread must be started by
 *   application start-up logic by calling work_usrstart().
 *
 * Input Parameters:
 *   argc, argv (not used)
 *
 * Returned Value:
 *   Does not return
 *
 ****************************************************************************/

#ifdef CONFIG_BUILD_PROTECTED
static int work_usrthread(int argc, char *argv[])
#else
static pthread_addr_t work_usrthread(pthread_addr_t arg)
#endif
{
  /* Loop forever */

  for (; ; )
    {
      /* Then process queued work.  We need to keep the work queue locked
       * while we process items in the work list.
       */

      work_process(&g_usrwork);
    }

#ifdef CONFIG_BUILD_PROTECTED
  return OK; /* To keep some compilers happy */
#else
  return NULL; /* To keep some compilers happy */
#endif
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: work_usrstart
 *
 * Description:
 *   Start the user mode work queue.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   The task ID of the worker thread is returned on success.  A negated
 *   errno value is returned on failure.
 *
 ****************************************************************************/

int work_usrstart(void)
{
#ifdef CONFIG_BUILD_PROTECTED
  /* Set up the work queue lock */

  _SEM_INIT(&g_usrsem, 0, 1);

  /* Start a user-mode worker thread for use by applications. */

  g_usrwork.pid = task_create("uwork",
                              CONFIG_LIB_USRWORKPRIORITY,
                              CONFIG_LIB_USRWORKSTACKSIZE,
                              (main_t)work_usrthread,
                              (FAR char * const *)NULL);

  DEBUGASSERT(g_usrwork.pid > 0);
  if (g_usrwork.pid < 0)
    {
      int errcode = get_errno();
      DEBUGASSERT(errcode > 0);
      return -errcode;
    }

  return g_usrwork.pid;
#else
  pthread_t usrwork;
  pthread_attr_t attr;
  struct sched_param param;
  int ret;

  /* Set up the work queue lock */

  pthread_mutex_init(&g_usrmutex, NULL);

  /* Start a user-mode worker thread for use by applications. */

  pthread_attr_init(&attr);
  pthread_attr_setstacksize(&attr, CONFIG_LIB_USRWORKSTACKSIZE);

#ifdef CONFIG_SCHED_SPORADIC
  /* Get the current sporadic scheduling parameters.  Those will not be
   * modified.
   */

  ret = set_getparam(pid, &param);
  if (ret < 0)
    {
      int errcode = get_errno();
      return -errcode;
    }
#endif

  param.sched_priority = CONFIG_LIB_USRWORKPRIORITY;
  pthread_attr_setschedparam(&attr, &param);

  ret = pthread_create(&usrwork, &attr, work_usrthread, NULL);
  if (ret != 0)
    {
      return -ret;
    }

  /* Detach because the return value and completion status will not be
   * requested.
   */

  pthread_detach(usrwork);

  g_usrwork.pid = (pid_t)usrwork;
  return g_usrwork.pid;
#endif
}

#endif /* CONFIG_LIB_USRWORK && !__KERNEL__*/
