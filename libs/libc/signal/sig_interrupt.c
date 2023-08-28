/****************************************************************************
 * libs/libc/signal/sig_interrupt.c
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

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: siginterrupt
 *
 * Description:
 * The siginterrupt() function allows signals to interrupt functions
 *
 * Input Parameters:
 *   signo - Signal to interrupt functions
 *   flag  - Flag to get restarting signal
 *
 * Returned Value:
 *   Upon successful completion, siginterrupt() shall return 0;
 *   otherwise, -1 shall be returned and errno set to indicate the error.
 *
 *    EINVAL - The signo argument is invalid.
 ****************************************************************************/

int siginterrupt(int signo, int flag)
{
  struct sigaction act;

  if (sigaction(signo, NULL, &act) < 0)
    {
      return ERROR;
    }

  if (flag)
    {
      act.sa_flags &= ~SA_RESTART;
    }
  else
    {
      act.sa_flags |= SA_RESTART;
    }

  return sigaction(signo, &act, NULL);
}
