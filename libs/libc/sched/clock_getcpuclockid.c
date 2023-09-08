/****************************************************************************
 * libs/libc/sched/clock_getcpuclockid.c
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

#include <time.h>

#include <nuttx/clock.h>
#include <nuttx/sched.h>
#include <nuttx/signal.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: clock_getcpuclockid
 *
 * Description:
 *   The function shall return clock id of the CPU-time clock of the process
 *   specified by pid
 *
 * Input Parameters:
 *   pid - the specified process id
 *   clockid - the clock type that need to setup
 *
 * Returned Value:
 *   Return 0 on success, return error number on error
 *
 ****************************************************************************/

int clock_getcpuclockid(pid_t pid, FAR clockid_t *clockid)
{
  /* If the pid is 0, we need to use the pid of current process */

  if (pid == 0)
    {
      pid = _SCHED_GETPID();
    }
  else if (_SIG_KILL(pid, 0) < 0)
    {
      return ERROR;
    }

  /* For clock_getcpuclockid, the clock type are
   * CLOCK_PROCESS_CPUTIME_ID
   */

  *clockid = (pid << CLOCK_SHIFT) | CLOCK_PROCESS_CPUTIME_ID;
  return OK;
}
