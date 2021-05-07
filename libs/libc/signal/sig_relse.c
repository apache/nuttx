/****************************************************************************
 * libs/libc/signal/sig_relse.c
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

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sigrelse
 *
 * Description:
 *   The sigrelse() function will remove 'signo' from the calling process'
 *   signal mask.
 *
 ****************************************************************************/

int sigrelse(int signo)
{
  sigset_t set;
  int ret;

  /* Create a set of signals with only the signal to be unblocked */

  sigemptyset(&set);
  ret = sigaddset(&set, signo);
  if (ret == OK)
    {
      /* Unblock the signal */

      ret = sigprocmask(SIG_UNBLOCK, &set, NULL);
    }

  return ret;
}
