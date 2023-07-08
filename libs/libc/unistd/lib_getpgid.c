/****************************************************************************
 * libs/libc/unistd/lib_getpgid.c
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

#include <nuttx/sched.h>
#include <nuttx/signal.h>

#include <unistd.h>
#include <errno.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: getpgid
 *
 * Description:
 *   Get the Process Group ID of the required process id. But since NuttX do
 *   not support process group, so the process group id are same as process
 *   id, so this method will return the pid that transferred in directly,
 *   which is same as getpgrp() method
 *
 * Input parameters:
 *   pid - the process id that required to fetch the process group id
 *
 * Returned Value:
 *   getpgid() shall return a process group ID. Otherwise, it shall return
 *   (pid_t)-1 and set errno to indicate the error. If the given process id
 *   are invalid, set errno as EINVAL, if the given process id do not exist,
 *   set errno as ESRCH
 *
 ****************************************************************************/

pid_t getpgid(pid_t pid)
{
  if (pid < 0)
    {
      set_errno(EINVAL);
      return INVALID_PROCESS_ID;
    }

  if (pid == 0)
    {
      return getpgrp();
    }
  else if (_SIG_KILL(pid, 0) < 0)
    {
      return INVALID_PROCESS_ID;
    }

  return pid;
}
