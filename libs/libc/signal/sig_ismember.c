/****************************************************************************
 * libs/libc/signal/sig_ismember.c
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
#include <errno.h>

#include <nuttx/signal.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsig_ismember
 *
 * Description:
 *   This function tests whether the signal specified by signo is a member
 *   of the set specified by set.
 *
 * Input Parameters:
 *   set - Signal set to test
 *   signo - Signal to test for
 *
 * Returned Value:
 *   This is an internal OS interface and should not be used by applications.
 *   On success, it returns 0 if the signal is not a member, 1 if the signal
 *   is a member of the set.
 *   A negated errno value is returned on failure.
 *
 *    EINVAL - The signo argument is invalid.
 *
 * Assumptions:
 *
 ****************************************************************************/

int nxsig_ismember(FAR const sigset_t *set, int signo)
{
  /* Verify the signal */

  if (!GOOD_SIGNO(signo))
    {
      return -EINVAL;
    }
  else
    {
      /* Check if the signal is in the set */

      return ((*set & SIGNO2SET(signo)) != 0);
    }
}

/****************************************************************************
 * Name: sigismember
 *
 * Description:
 *   This function tests whether the signal specified by signo is a member
 *   of the set specified by set.
 *
 * Input Parameters:
 *   set - Signal set to test
 *   signo - Signal to test for
 *
 * Returned Value:
 *   1 (true), if the specified signal is a member of the set,
 *   0 (OK or FALSE), if it is not, or
 *  -1 (ERROR) if the signal number is invalid.
 *
 * Assumptions:
 *
 ****************************************************************************/

int sigismember(FAR const sigset_t *set, int signo)
{
  int ret;

  /* Let nxsig_ismember do all of the work */

  ret = nxsig_ismember(set, signo);
  if (ret < 0)
    {
      set_errno(EINVAL);
      ret = ERROR;
    }

  return ret;
}
