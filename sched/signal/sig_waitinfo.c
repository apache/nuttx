/****************************************************************************
 * sched/signal/sig_waitinfo.c
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

#include <errno.h>

#include <nuttx/cancelpt.h>
#include <nuttx/signal.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sigwaitinfo
 *
 * Description:
 *   This function is equivalent to sigtimedwait with a NULL timeout
 *   parameter.
 *
 * Input Parameters:
 *   set - The pending signal set
 *   info - The returned value
 *
 * Returned Value:
 *   Signal number that cause the wait to be terminated, otherwise -1 (ERROR)
 *   is returned and the errno variable is set appropriately.
 *
 ****************************************************************************/

int sigwaitinfo(FAR const sigset_t *set, FAR struct siginfo *info)
{
  int ret;

  /* sigwaitinfo() is a cancellation point */

  enter_cancellation_point();

  /* Just a wrapper around nxsig_timedwait() */

  ret = nxsig_timedwait(set, info, NULL);
  if (ret < 0)
    {
      set_errno(-ret);
      ret = ERROR;
    }

  leave_cancellation_point();
  return ret;
}
