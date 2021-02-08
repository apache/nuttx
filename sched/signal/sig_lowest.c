/****************************************************************************
 * sched/signal/sig_lowest.c
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

#include <nuttx/signal.h>

#include "signal/signal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsig_lowest
 *
 * Description:
 *   Return the lowest signal number that is a member of a set of signals.
 *
 ****************************************************************************/

int nxsig_lowest(sigset_t *set)
{
  int signo;

  for (signo = MIN_SIGNO; signo <= MAX_SIGNO; signo++)
    {
      if (nxsig_ismember(set, signo))
        {
          return signo;
        }
    }

  return ERROR;
}
