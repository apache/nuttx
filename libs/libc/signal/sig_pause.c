/****************************************************************************
 * libs/libc/signal/sig_pause.c
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

#include <unistd.h>
#include <signal.h>
#include <sched.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sigpause
 *
 * Description:
 *   The sigpause() will remove sig from the calling process' signal mask
 *   and suspend the calling process until a signal is received. The
 *   sigpause() function will restore the process' signal mask to its
 *   original state before returning.
 *
 ****************************************************************************/

int sigpause(int signo)
{
  sigset_t set;
  int ret;

  /* Get the current set of blocked signals */

  sched_lock();
  ret = sigprocmask(SIG_SETMASK, NULL, &set);
  if (ret == OK)
    {
      /* Remove the 'signo' from the set of blocked signals */

      ret = sigdelset(&set, signo);
    }

  /* Let sigsuspend do the rest of the job */

  if (ret == OK)
    {
      ret = sigsuspend(&set);
    }

  sched_unlock();
  return ret;
}
