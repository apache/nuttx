/****************************************************************************
 * sched/signal/sig_notification.c
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

#include <inttypes.h>
#include <string.h>
#include <signal.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/signal.h>

#include "sched/sched.h"
#include "signal/signal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_SIG_EVTHREAD_HPWORK
#  define SIG_EVTHREAD_WORK HPWORK
#else
#  define SIG_EVTHREAD_WORK LPWORK
#endif

/****************************************************************************
 * Name: nxsig_notification_worker
 *
 * Description:
 *   Perform the callback from the context of the worker thread.
 *
 * Input Parameters:
 *   arg - Work argument.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_SIG_EVTHREAD
static void nxsig_notification_worker(FAR void *arg)
{
  FAR struct sigwork_s *work = (FAR struct sigwork_s *)arg;

  DEBUGASSERT(work != NULL);

  /* Perform the callback */

  work->func(work->value);
}

#endif /* CONFIG_SIG_EVTHREAD */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsig_notification
 *
 * Description:
 *   Notify a client an event via either a signal or function call
 *   base on the sigev_notify field.
 *
 * Input Parameters:
 *   pid   - The task/thread ID a the client thread to be signaled.
 *   event - The instance of struct sigevent that describes how to signal
 *           the client.
 *   code  - Source: SI_USER, SI_QUEUE, SI_TIMER, SI_ASYNCIO, or SI_MESGQ
 *   work  - The work structure to queue.  Must be non-NULL if
 *           event->sigev_notify == SIGEV_THREAD.  Ignored if
 *           CONFIG_SIG_EVTHREAD is not defined.
 *
 * Returned Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *
 ****************************************************************************/

int nxsig_notification(pid_t pid, FAR struct sigevent *event,
                       int code, FAR struct sigwork_s *work)
{
  sinfo("pid=%" PRIu16 " signo=%d code=%d sival_ptr=%p\n",
         pid, event->sigev_signo, code, event->sigev_value.sival_ptr);

  /* Notify client via a signal? */

  if (event->sigev_notify == SIGEV_SIGNAL)
    {
#ifdef CONFIG_SCHED_HAVE_PARENT
      FAR struct tcb_s *rtcb = this_task();
#endif
      siginfo_t info;

      /* Yes.. Create the siginfo structure */

      info.si_signo  = event->sigev_signo;
      info.si_code   = code;
      info.si_errno  = OK;
#ifdef CONFIG_SCHED_HAVE_PARENT
      info.si_pid    = rtcb->pid;
      info.si_status = OK;
#endif

      /* Some compilers (e.g., SDCC), do not permit assignment of aggregates.
       * Use of memcpy() is overkill;  We could just copy the larger of the
       * int and FAR void * members in the union.  memcpy(), however, does
       * not require that we know which is larger.
       */

      memcpy(&info.si_value, &event->sigev_value, sizeof(union sigval));

      /* Send the signal */

      return nxsig_dispatch(pid, &info);
    }

#ifdef CONFIG_SIG_EVTHREAD
  /* Notify the client via a function call */

  else if (event->sigev_notify == SIGEV_THREAD)
    {
      /* Initialize the work information */

      work->value = event->sigev_value;
      work->func  = event->sigev_notify_function;

      /* Then queue the work */

      return work_queue(SIG_EVTHREAD_WORK, &work->work,
                        nxsig_notification_worker, work, 0);
    }
#endif

  return event->sigev_notify == SIGEV_NONE ? OK : -ENOSYS;
}

/****************************************************************************
 * Name: nxsig_cancel_notification
 *
 * Description:
 *   Cancel the notification if it doesn't send yet.
 *
 * Input Parameters:
 *   work  - The work structure to cancel
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SIG_EVTHREAD
void nxsig_cancel_notification(FAR struct sigwork_s *work)
{
  work_cancel(SIG_EVTHREAD_WORK, &work->work);
}
#endif
