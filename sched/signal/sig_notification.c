/****************************************************************************
 * sched/signal/sig_notification.c
 *
 *   Copyright (C) 2018 Pinecone Inc. All rights reserved.
 *   Author: Xiang Xiao <xiaoxiang@pinecone.net>
 *
 * Derives from code originally written by:
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
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

#include <debug.h>
#include <signal.h>

#include "sched/sched.h"
#include "signal/signal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsig_notification
 *
 * Description:
 *   Notify a client an event via either a singal or function call
 *   base on the sigev_notify field.
 *
 * Input Parameters:
 *   pid   - The task/thread ID a the client thread to be signaled.
 *   event - The instance of struct sigevent that describes how to signal
 *           the client.
 *   code  - Source: SI_USER, SI_QUEUE, SI_TIMER, SI_ASYNCIO, or SI_MESGQ
 *
 * Returned Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *
 ****************************************************************************/

int nxsig_notification(pid_t pid, FAR struct sigevent *event, int code)
{
  sinfo("pid=%p signo=%d code=%d sival_ptr=%p\n",
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
      info.si_value  = event->sigev_value;
#ifdef CONFIG_SCHED_HAVE_PARENT
      info.si_pid    = rtcb->pid;
      info.si_status = OK;
#endif

      /* Send the signal */

      return nxsig_dispatch(pid, &info);
    }

#ifdef CONFIG_SIG_EVTHREAD
  /* Notify the client via a function call */

  else if (event->sigev_notify == SIGEV_THREAD)
    {
      return nxsig_evthread(pid, event);
    }
#endif

  return event->sigev_notify == SIGEV_NONE ? OK : -ENOSYS;
}

