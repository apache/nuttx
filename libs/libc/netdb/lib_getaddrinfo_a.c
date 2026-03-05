/****************************************************************************
 * libs/libc/netdb/lib_getaddrinfo_a.c
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

#include <string.h>
#include <unistd.h>
#include <netdb.h>

#include "libc.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Heap-allocated batch context, owned and freed by the worker thread.
 * Contains only the data the worker needs; all synchronization lives
 * in the per-request gaicb (ar_lock / ar_cond).
 */

struct gai_batch_arg
{
  struct sigevent   sevp;   /* Batch-completion sigevent copy   */
  pid_t             pid;    /* Caller PID for SIGEV_SIGNAL      */
  int               next;   /* Next entry to work on            */
  int               nitems; /* Number of entries in list        */
  FAR struct gaicb *list[]; /* Copy of the gaicb pointer array  */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gai_notify
 *
 * Description:
 *   Fire the batch-completion sigevent.
 *
 ****************************************************************************/

static void gai_notify(FAR const struct sigevent *sevp, pid_t pid)
{
  switch (sevp->sigev_notify)
    {
      case SIGEV_SIGNAL:
        sigqueue(pid, sevp->sigev_signo, sevp->sigev_value);
        break;

#ifdef CONFIG_SIG_EVTHREAD
      case SIGEV_THREAD:
        if (sevp->sigev_notify_function != NULL)
          {
            sevp->sigev_notify_function(sevp->sigev_value);
          }
#endif
    }
}

/****************************************************************************
 * Name: gai_worker_cleanup
 *
 * Description:
 *   Cleanup handler pushed via pthread_cleanup_push() inside gai_worker().
 *   Runs on cancellation (pthread_cancel) and the normal completion path.
 *   Marks remaining in-progress requests as EAI_CANCELED and signals
 *   their condvars, fires the batch sigevent, then frees the batch arg.
 *
 ****************************************************************************/

static void gai_worker_cleanup(FAR void *arg)
{
  FAR struct gai_batch_arg *barg = arg;

  for (; barg->next < barg->nitems; barg->next++)
    {
      FAR struct gaicb *req = barg->list[barg->next];

      if (req != NULL)
        {
          pthread_mutex_lock(&req->ar_lock);
          req->ar_return = EAI_CANCELED;
          pthread_cond_broadcast(&req->ar_cond);
          pthread_mutex_unlock(&req->ar_lock);
        }
    }

  gai_notify(&barg->sevp, barg->pid);
  lib_free(barg);
}

/****************************************************************************
 * Name: gai_worker
 *
 * Description:
 *   Iterates over all gaicb entries, calls getaddrinfo() synchronously for
 *   each, writes results into the gaicb under req->ar_lock, then calls
 *   pthread_cleanup_pop(1) to fire the batch sigevent and free resources.
 *
 ****************************************************************************/

static FAR void *gai_worker(FAR void *arg)
{
  FAR struct gai_batch_arg *barg = arg;
  int ret;

  pthread_cleanup_push(gai_worker_cleanup, barg);

  for (; barg->next < barg->nitems; barg->next++)
    {
      FAR struct gaicb *req = barg->list[barg->next];

      if (req != NULL)
        {
          ret = getaddrinfo(req->ar_name, req->ar_service,
                            req->ar_request, &req->ar_result);

          pthread_mutex_lock(&req->ar_lock);
          req->ar_return = ret;
          pthread_cond_broadcast(&req->ar_cond);
          pthread_mutex_unlock(&req->ar_lock);
        }
    }

  /* Normal completion: pop with executing the cleanup handler */

  pthread_cleanup_pop(1);
  return NULL;
}

/****************************************************************************
 * Name: gai_cb_init
 *
 * Description:
 *   Initialize a gaicb's synchronization primitives (ar_lock, ar_cond)
 *   and reset its result fields (ar_result, ar_return, ar_worker) to
 *   their initial state prior to dispatch.
 *
 * Note:
 *   ar_lock/ar_cond don't need to be destroyed manually, since both
 *   functions are the trivial operation for the the simple mutex.
 *
 ****************************************************************************/

static void gai_cb_init(FAR struct gaicb *req)
{
  pthread_mutexattr_t attr;

  pthread_mutexattr_init(&attr);
  pthread_mutexattr_setprotocol(&attr, PTHREAD_PRIO_NONE);
  pthread_mutexattr_setrobust(&attr, PTHREAD_MUTEX_STALLED);

  req->ar_result = NULL;
  req->ar_return = EAI_INPROGRESS;
  req->ar_worker = 0;

  pthread_mutex_init(&req->ar_lock, &attr);
  pthread_cond_init(&req->ar_cond, NULL);

  pthread_mutexattr_destroy(&attr);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: getaddrinfo_a
 *
 * Description:
 *   Asynchronous address resolution.
 *
 *   mode == GAI_WAIT:   resolve synchronously in the caller's thread.
 *   mode == GAI_NOWAIT: spawn a per-batch worker thread and return 0.
 *                       gai_cancel() waits for it on cancellation.
 *
 * Returned Value:
 *   0 on success; EAI_AGAIN, EAI_MEMORY, or EAI_SYSTEM on error.
 *
 ****************************************************************************/

int getaddrinfo_a(int mode, FAR struct gaicb *list[], int n,
                  FAR struct sigevent *sevp)
{
  FAR struct gai_batch_arg *barg;
  FAR struct gaicb *req;
  pthread_attr_t attr;
  pthread_t tid;
  int ret;
  int i;

  if (list == NULL || n <= 0)
    {
      return EAI_SYSTEM;
    }

  if (mode != GAI_WAIT && mode != GAI_NOWAIT)
    {
      return EAI_SYSTEM;
    }

  /* Initialize gaicb and call getaddrinfo in the wait mode */

  for (i = 0; i < n; i++)
    {
      req = list[i];
      if (req != NULL)
        {
          gai_cb_init(req);
          if (mode == GAI_WAIT)
            {
              req->ar_return = getaddrinfo(req->ar_name,
                                           req->ar_service,
                                           req->ar_request,
                                           &req->ar_result);
            }
        }
    }

  if (mode == GAI_WAIT)
    {
      return 0;
    }

  /* Build batch arg and spawn detached worker thread for no wait mode */

  barg = lib_malloc(sizeof(struct gai_batch_arg) +
                    n * sizeof(FAR struct gaicb *));
  if (barg == NULL)
    {
      ret = EAI_MEMORY;
      goto err;
    }

  barg->next   = 0;
  barg->nitems = n;
  barg->pid    = getpid();

  memcpy(barg->list, list, n * sizeof(FAR struct gaicb *));

  if (sevp != NULL)
    {
      barg->sevp = *sevp;
    }
  else
    {
      memset(&barg->sevp, 0, sizeof(barg->sevp));
      barg->sevp.sigev_notify = SIGEV_NONE;
    }

  pthread_attr_init(&attr);
  pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
  pthread_attr_setstacksize(&attr, CONFIG_NETDB_GETADDRINFO_A_STACKSIZE);

  ret = pthread_create(&tid, &attr, gai_worker, barg);
  pthread_attr_destroy(&attr);

  if (ret != 0)
    {
      lib_free(barg);
      ret = EAI_MEMORY;
      goto err;
    }

  /* Record worker tid in each req */

  for (i = 0; i < n; i++)
    {
      req = list[i];
      if (req != NULL)
        {
          req->ar_worker = tid;
        }
    }

  return 0;

err:
  for (i = 0; i < n; i++)
    {
      req = list[i];
      if (req != NULL)
        {
          req->ar_return = ret;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: gai_suspend
 *
 * Description:
 *   Block until at least one request in list[] completes, or timeout
 *   expires.  Waits on the first in-progress request's condvar only;
 *   does not wait on subsequent requests.
 *
 * Returned Value:
 *   0           - at least one request completed, others may still
 *                 be in progress.
 *   EAI_ALLDONE - all requests have already completed.
 *   EAI_AGAIN   - timeout expired before any request completed.
 *   EAI_INTR    - interrupted by a signal.
 *   EAI_SYSTEM  - system error or list is NULL.
 *
 ****************************************************************************/

int gai_suspend(FAR const struct gaicb * const list[], int n,
                FAR const struct timespec *timeout)
{
  struct timespec timeout_abs;
  int ret = EAI_ALLDONE;
  int i;

  if (list == NULL || n <= 0)
    {
      return EAI_SYSTEM;
    }

  if (timeout != NULL)
    {
      clock_gettime(CLOCK_REALTIME, &timeout_abs);
      timeout_abs.tv_sec  += timeout->tv_sec;
      timeout_abs.tv_nsec += timeout->tv_nsec;
      if (timeout_abs.tv_nsec >= NSEC_PER_SEC)
        {
          timeout_abs.tv_sec++;
          timeout_abs.tv_nsec -= NSEC_PER_SEC;
        }
    }

  /* Wait on the first in-progress req's condvar. If a request
   * has already completed, skip straight to the alldone check.
   */

  for (i = 0; i < n; i++)
    {
      FAR struct gaicb *req = (FAR struct gaicb *)list[i];

      if (req != NULL)
        {
          pthread_mutex_lock(&req->ar_lock);
          if (req->ar_return == EAI_INPROGRESS)
            {
              int rc = 0;

              /* Wait in a loop to defend against spurious wakeups, as
               * required by POSIX for pthread_cond_*wait().
               */

              while (req->ar_return == EAI_INPROGRESS && rc == 0)
                {
                  rc = timeout != NULL
                      ? pthread_cond_timedwait(&req->ar_cond,
                                               &req->ar_lock,
                                               &timeout_abs)
                      : pthread_cond_wait(&req->ar_cond, &req->ar_lock);
                }

              if (rc == EINTR)
                {
                  ret = EAI_INTR;
                }
              else if (rc == ETIMEDOUT)
                {
                  ret = EAI_AGAIN;
                }
              else if (rc != 0)
                {
                  ret = EAI_SYSTEM;
                }

              pthread_mutex_unlock(&req->ar_lock);
              break;
            }

          pthread_mutex_unlock(&req->ar_lock);
        }
    }

  for (; i < n && ret == EAI_ALLDONE; i++)
    {
      FAR struct gaicb *req = (FAR struct gaicb *)list[i];

      if (req != NULL)
        {
          pthread_mutex_lock(&req->ar_lock);
          if (req->ar_return == EAI_INPROGRESS)
            {
              ret = 0;
            }

          pthread_mutex_unlock(&req->ar_lock);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: gai_error
 *
 * Description:
 *   Return the status of an async request.
 *
 ****************************************************************************/

int gai_error(FAR struct gaicb *req)
{
  int ret = EAI_SYSTEM;

  if (req != NULL)
    {
      pthread_mutex_lock(&req->ar_lock);
      ret = req->ar_return;
      pthread_mutex_unlock(&req->ar_lock);
    }

  return ret;
}

/****************************************************************************
 * Name: gai_cancel
 *
 * Description:
 *   Cancel an async lookup.  If the request is still in progress,
 *   send pthread_cancel to the worker thread.
 *
 *   This function cancels the entire worker thread, which affects
 *   ALL requests in the same batch, not just the single req passed in.
 *
 * Returned Value:
 *   EAI_CANCELED    - cancel was requested.
 *   EAI_ALLDONE     - request has already completed or been canceled.
 *   EAI_SYSTEM      - req is NULL.
 *
 ****************************************************************************/

int gai_cancel(FAR struct gaicb *req)
{
  int ret = EAI_ALLDONE;

  if (req == NULL)
    {
      return EAI_SYSTEM;
    }

  pthread_mutex_lock(&req->ar_lock);
  if (req->ar_return == EAI_INPROGRESS)
    {
      pthread_cancel(req->ar_worker);
      while (req->ar_return == EAI_INPROGRESS)
        {
          pthread_cond_wait(&req->ar_cond, &req->ar_lock);
        }

      ret = EAI_CANCELED;
    }

  pthread_mutex_unlock(&req->ar_lock);

  return ret;
}
