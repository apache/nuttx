/****************************************************************************
 * sched/pthread/pthread_sigqueue.c
 *
 * SPDX-License-Identifer: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements. See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership. The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License. You may obtain a copy of the License at
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

#include <errno.h>
#include <nuttx/debug.h>
#include <pthread.h>
#include <signal.h>

#include <nuttx/signal.h>

#include "sched/sched.h"
#include "signal/signal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int pthread_sigqueue(pthread_t thread, int sig, const union sigval value)
{
#ifdef CONFIG_SCHED_HAVE_PARENT
  FAR struct tcb_s *rtcb = this_task();
#endif
  siginfo_t info;
  int err;

  sinfo("tid=%d sig=%d value=%d\n", thread, sig, value.sival_int);

  /* Check if signal is valid */

  if (!GOOD_SIGNO(sig))
    {
      return EINVAL;
    }

  /* Create the siginfo structure */

  info.si_signo = sig;
  info.si_code = SI_QUEUE;
  info.si_errno = OK;
  info.si_value = value;
#ifdef CONFIG_SCHED_HAVE_PARENT
  info.si_pid = rtcb->pid;
  info.si_status = OK;
#endif
  info.si_user = NULL; /* Will be set in sig_dispatch.c */

  /* Send the signal */

  err = nxsig_dispatch((pid_t)thread, &info, true);
  if (err < 0)
    {
      return -err;
    }

  return 0;
}
