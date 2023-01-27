/****************************************************************************
 * sched/signal/sig_tgkill.c
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

#include <signal.h>
#include <errno.h>

#include <nuttx/sched.h>
#include <nuttx/signal.h>

#include "sched/sched.h"
#include "signal/signal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tgkill
 *
 * Description:
 *   The tgkill() system call can be used to send any signal to a thread.
 *   See kill() for further information as this is just a simple wrapper
 *   around the kill() function.
 *
 * Input Parameters:
 *   gid   - The id of the task to receive the signal.
 *   tid   - The id of the thread to receive the signal. Only positive,
 *           non-zero values of 'tid' are supported.
 *   signo - The signal number to send.  If 'signo' is zero, no signal is
 *           sent, but all error checking is performed.
 *
 * Returned Value:
 *    On success the signal was send and zero is returned. On error -1 is
 *    returned, and errno is set one of the following error numbers:
 *
 *    EINVAL An invalid signal was specified.
 *    EPERM  The thread does not have permission to send the
 *           signal to the target thread.
 *    ESRCH  No thread could be found corresponding to that
 *           specified by the given thread ID
 *    ENOSYS Do not support sending signals to process groups.
 *
 ****************************************************************************/

int tgkill(pid_t pid, pid_t tid, int signo)
{
#ifdef HAVE_GROUP_MEMBERS
  /* If group members are supported then tgkill() differs from kill().
   * kill(), in this case, must follow the POSIX rules for delivery of
   * signals in the group environment.  Otherwise, kill(), like tgkill()
   * will just deliver the signal to the thread ID it is requested to use.
   */

#ifdef CONFIG_SCHED_HAVE_PARENT
  FAR struct tcb_s *rtcb = this_task();
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
  info.si_errno           = EINTR;
  info.si_value.sival_ptr = NULL;
#ifdef CONFIG_SCHED_HAVE_PARENT
  info.si_pid             = rtcb->pid;
  info.si_status          = OK;
#endif

  /* Get the TCB associated with the thread */

  stcb = nxsched_get_tcb(tid);
  if (!stcb)
    {
      ret = -ESRCH;
      goto errout_with_lock;
    }

  /* Dispatch the signal to thread, bypassing normal task group thread
   * dispatch rules.
   */

  ret = nxsig_tcbdispatch(stcb, &info);
  sched_unlock();

  if (ret < 0)
    {
      goto errout;
    }

  return OK;

errout_with_lock:
  sched_unlock();
errout:
  set_errno(-ret);
  return ERROR;

#else
  /* If group members are not supported then tgkill is basically the
   * same as kill() other than the sign of the returned value.
   */

  return kill(tid, signo);
#endif
}
