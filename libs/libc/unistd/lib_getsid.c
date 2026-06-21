/****************************************************************************
 * libs/libc/unistd/lib_getsid.c
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
 * Name: getsid
 *
 * Description:
 *   Get the session ID of the process whose process ID is pid.  NuttX does
 *   not implement sessions, so the session ID is the same as the process
 *   ID, mirroring getpgid().
 *
 * Input Parameters:
 *   pid - The process ID whose session ID is requested.  A value of zero
 *         refers to the calling process.
 *
 * Returned Value:
 *   The session ID is returned on success.  Otherwise, (pid_t)-1 is
 *   returned and errno is set:  EINVAL if pid is negative, ESRCH if pid
 *   does not refer to an existing process.
 *
 ****************************************************************************/

pid_t getsid(pid_t pid)
{
  if (pid < 0)
    {
      set_errno(EINVAL);
      return (pid_t)-1;
    }

  if (pid == 0)
    {
      return _SCHED_GETPID();
    }

  if (_SIG_KILL(pid, 0) < 0)
    {
      return (pid_t)-1;
    }

  return pid;
}
