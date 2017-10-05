/****************************************************************************
 * sched/signal/sig_notification.c
 *
 *   Copyright (C) 2015, 2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <signal.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/kmalloc.h>
#include <nuttx/wqueue.h>

#ifdef CONFIG_SIG_EVTHREAD

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Use the low-prioriry work queue is it is available */

#if defined(CONFIG_SCHED_LPWORK)
#  define NTWORK LPWORK
#elif defined(CONFIG_SCHED_HPWORK)
#  define NTWORK HPWORK
#else
#  error Work queue is not enabled
#endif

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

/* This structure retains all that is necessary to perform the notification */

struct sig_notify_s
{
  struct work_s nt_work;           /* Work queue structure */
  union sigval nt_value;           /* Data passed with notification */
  sigev_notify_function_t nt_func; /* Notification function */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsig_notify_worker
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

static void nxsig_notify_worker(FAR void *arg)
{
  FAR struct sig_notify_s *notify = (FAR struct sig_notify_s *)arg;

  DEBUGASSERT(notify != NULL);

  /* Perform the callback */

#ifdef CONFIG_CAN_PASS_STRUCTS
  notify->nt_func(notify->nt_value);
#else
  notify->nt_func(notify->nt_value.sival_ptr);
#endif

  /* Free the alloated notification parameters */

  kmm_free(notify);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsig_notification
 *
 * Description:
 *   Notify a client a signal event via a function call.  This function is
 *   an internal OS interface that implements the common logic for signal
 *   event notification for the case of SIGEV_THREAD.
 *
 * Input Parameters:
 *   pid   - The task/thread ID a the client thread to be signaled.
 *   event - The instance of struct sigevent that describes how to signal
 *           the client.
 *
 * Returned Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *
 ****************************************************************************/

int nxsig_notification(pid_t pid, FAR struct sigevent *event)
{
  FAR struct sig_notify_s *notify;
  DEBUGASSERT(event != NULL && event->sigev_notify_function != NULL);
  int ret;

  /* Allocate a structure to hold the notification information */

  notify = kmm_zalloc(sizeof(struct sig_notify_s));
  if (notify == NULL)
    {
      return -ENOMEM;
    }

  /* Initialize the notification information */

#ifdef CONFIG_CAN_PASS_STRUCTS
  notify->nt_value = event->sigev_value;
#else
  notify->nt_value.sival_ptr = event->sigev_value.sival_ptr;
#endif
  notify->nt_func = event->sigev_notify_function;

  /* Then queue the work */

  ret = work_queue(NTWORK, &notify->nt_work, nxsig_notify_worker,
                   notify, 0);
  if (ret < 0)
    {
      kmm_free(notify);
    }

  return ret;
}

#endif /* CONFIG_SIG_EVTHREAD */
