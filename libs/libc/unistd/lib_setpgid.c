/****************************************************************************
 * libs/libc/unistd/lib_setpgid.c
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

#include <unistd.h>
#include <errno.h>

#include <nuttx/sched.h>
#include <nuttx/signal.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: setpgid
 *
 * Description:
 *   Set the process group ID of the process whose process ID is pid to
 *   pgid.  NuttX does not implement process groups, so a process group
 *   always contains a single member and its ID equals the process ID.
 *   Consequently this stub only succeeds when the requested pgid matches
 *   the target pid (or is zero, which selects the target pid itself).
 *
 * Input Parameters:
 *   pid  - The process whose process group ID is to be changed.  A value of
 *          zero refers to the calling process.
 *   pgid - The new process group ID.  A value of zero uses the target pid.
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, -1 is returned and errno is
 *   set:  EINVAL if pgid is negative, ESRCH if pid does not exist, EPERM if
 *   the requested pgid cannot be honored by this single-member-group model.
 *
 ****************************************************************************/

int setpgid(pid_t pid, pid_t pgid)
{
  if (pgid < 0)
    {
      set_errno(EINVAL);
      return -1;
    }

  if (pid == 0)
    {
      pid = _SCHED_GETPID();
    }
  else if (_SIG_KILL(pid, 0) < 0)
    {
      return -1;
    }

  /* A process group can only contain the single member 'pid', so the only
   * legal group ID is 'pid' itself (a zero pgid selects it implicitly).
   */

  if (pgid != 0 && pgid != pid)
    {
      set_errno(EPERM);
      return -1;
    }

  return 0;
}
