/****************************************************************************
 * libs/libc/pthread/pthread_kill.c
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

#include <signal.h>
#include <pthread.h>
#include <errno.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pthread_kill
 *
 * Description:
 *   The pthread_kill() system call can be used to send any signal to a
 *   thread.  See nxsig_kill() for further information as this is just a
 *   simple wrapper around the nxsig_kill() function.
 *
 * Input Parameters:
 *   thread - The id of the thread to receive the signal. Only positive,
 *     non-zero values of 'thread' are supported.
 *   signo - The signal number to send.  If 'signo' is zero, no signal is
 *    sent, but all error checking is performed.
 *
 * Returned Value:
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
 ****************************************************************************/

int pthread_kill(pthread_t thread, int signo)
{
  int ret = tkill((pid_t)thread, signo);
  if (ret < 0)
    {
      ret = get_errno();
    }

  return ret;
}
