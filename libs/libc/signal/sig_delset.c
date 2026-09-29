/****************************************************************************
 * libs/libc/signal/sig_delset.c
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

#include <signal.h>
#include <errno.h>
#include <nuttx/signal.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sigdelset
 *
 * Description:
 *   This function deletes the signal specified by signo from the signal
 *   set specified by the 'set' argument.
 *
 * Input Parameters:
 *   set - Signal set to delete the signal from
 *   signo - Signal to delete
 *
 * Returned Value:
 *   0 (OK), or -1 (ERROR) if the signal number is invalid.
 *
 * Assumptions:
 *
 ****************************************************************************/

int sigdelset(FAR sigset_t *set, int signo)
{
  int ret;

  /* Let nxseg_delset do all the work. */

  ret = nxsig_delset(set, signo);
  if (ret < 0)
    {
      set_errno(EINVAL);
      ret = ERROR;
    }

  return ret;
}
