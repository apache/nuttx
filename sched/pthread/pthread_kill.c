/************************************************************************
 * sched/pthread/pthread_kill.c
 *
 *   Copyright (C) 2007, 2009, 2011, 2015 Gregory Nutt. All rights reserved.
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
 ************************************************************************/

/************************************************************************
 * Included Files
 ************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <signal.h>
#include <pthread.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/sched.h>

#include "signal/signal.h"

/************************************************************************
 * Public Functions
 ************************************************************************/

/************************************************************************
 * Name: pthread_kill
 *
 * Description:
 *   The pthread_kill() system call can be used to send any signal to a
 *   thread.  See kill() for further information as this is just a simple
 *   wrapper around the kill() function.
 *
 * Parameters:
 *   thread - The id of the thread to receive the signal. Only positive,
 *     non-zero values of 'thread' are supported.
 *   signo - The signal number to send.  If 'signo' is zero, no signal is
 *    sent, but all error checking is performed.
 *
 * Return Value:
 *    On success the signal was send and zero is returned. On error one
 *    of the following error numbers is returned.
 *
 *    EINVAL An invalid signal was specified.
 *    EPERM  The thread does not have permission to send the
 *           signal to the target thread.
 *    ESRCH  No thread could be found corresponding to that
 *           specified by the given thread ID
 *    ENOSYS Do not support sending signals to process groups.
 *
 * Assumptions:
 *
 ************************************************************************/

int pthread_kill(pthread_t thread, int signo)
{
#ifdef HAVE_GROUP_MEMBERS
  /* If group members are support then pthread_kill() differs from kill().
   * kill(), in this case, must following the POSIX rules for delivery of
   * signals in the group environment.  Otherwise, kill(), like
   * pthread_kill() will just deliver the signal to the thread ID it is
   * requested to use.
   */

#ifdef CONFIG_SCHED_HAVE_PARENT
  FAR struct tcb_s *rtcb = (FAR struct tcb_s *)g_readytorun.head;
#endif
  FAR struct tcb_s *stcb;
  siginfo_t info;
  int ret;

  /* Make sure that the signal is valid */

  if (!GOOD_SIGNO(signo))
    {
      ret = -EINVAL;
      goto errout;
    }

  /* Keep things stationary through the following */

  sched_lock();

  /* Create the siginfo structure */

  info.si_signo           = signo;
  info.si_code            = SI_USER;
  info.si_value.sival_ptr = NULL;
#ifdef CONFIG_SCHED_HAVE_PARENT
  info.si_pid             = rtcb->pid;
  info.si_status          = OK;
#endif

  /* Get the TCB associated with the thread */

  stcb = sched_gettcb((pid_t)thread);
  if (!stcb)
    {
      ret = -ESRCH;
      goto errout_with_lock;
    }

  /* Dispatch the signal to thread, bypassing normal task group thread
   * dispatch rules.
   */

  ret = sig_tcbdispatch(stcb, &info);
  sched_unlock();

  if (ret < 0)
    {
      goto errout;
    }

  return OK;

errout_with_lock:
  sched_unlock();
errout:
  return -ret;

#else
  /* If group members are not supported then pthread_kill is basically the
   * same as kill().
   */

  int ret;

  set_errno(EINVAL);
  ret = kill((pid_t)thread, signo);
  if (ret != OK)
    {
       ret = get_errno();
    }

  return ret;
#endif
}
