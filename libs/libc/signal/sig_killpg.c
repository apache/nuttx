/****************************************************************************
 * libs/libc/signal/sig_killpg.c
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

#include <errno.h>
#include <signal.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: killpg
 *
 * Description:
 *   The killpg() system call can be used to send any signal to process
 *   group. See kill() for further information as this is just a simple
 *   wrapper around the kill() function.
 *
 * Input Parameters:
 *   pgrp  - The id of the process group to receive the signal.
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

int killpg(pid_t pgrp, int signo)
{
  if (pgrp <= CONFIG_SMP_NCPUS)
    {
      set_errno(EINVAL);
      return ERROR;
    }

  /* Nuttx do not support process group, we use kill process instead */

  return kill(pgrp, signo);
}
