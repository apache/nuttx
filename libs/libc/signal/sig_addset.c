/****************************************************************************
 * libs/libc/signal/sig_addset.c
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

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsig_addset
 *
 * Description:
 *   This function adds the signal specified by signo to the signal set
 *   specified by set.
 *
 * Input Parameters:
 *   set - Signal set to add signal to
 *   signo - Signal to add
 *
 * Returned Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *
 *    EINVAL - The signo argument is invalid.
 *
 * Assumptions:
 *
 ****************************************************************************/

int nxsig_addset(FAR sigset_t *set, int signo)
{
  /* Verify the signal */

  if (!GOOD_SIGNO(signo))
    {
      return -EINVAL;
    }
  else
    {
      /* Add the signal to the set */

      *set |= SIGNO2SET(signo);
      return OK;
    }
}

/****************************************************************************
 * Name: sigaddset
 *
 * Description:
 *   This function adds the signal specified by signo to the signal set
 *   specified by set.
 *
 * Input Parameters:
 *   set - Signal set to add signal to
 *   signo - Signal to add
 *
 * Returned Value:
 *   0 (OK), or -1 (ERROR) if the signal number is invalid.
 *
 * Assumptions:
 *
 ****************************************************************************/

int sigaddset(FAR sigset_t *set, int signo)
{
  int ret;

  /* Let nxsig_addset do all the work. */

  ret = nxsig_addset(set, signo);
  if (ret < 0)
    {
      set_errno(EINVAL);
      ret = ERROR;
    }

  return ret;
}
