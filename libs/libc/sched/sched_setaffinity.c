/****************************************************************************
 * libs/libc/sched/sched_setaffinity.c
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

#include <nuttx/sched.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sched_setaffinity
 *
 * Description:
 *   sched_setaffinity() sets the CPU affinity mask of the thread whose ID
 *   is pid to the value specified by mask.  If pid is zero, then the
 *   calling thread is used.  The argument cpusetsize is the length (i
 *   bytes) of the data pointed to by mask.  Normally this argument would
 *   be specified as sizeof(cpu_set_t).
 *
 *   If the thread specified by pid is not currently running on one of the
 *   CPUs specified in mask, then that thread is migrated to one of the
 *   CPUs specified in mask.
 *
 *   This function is a simply wrapper around nxsched_set_affinity() that
 *   sets the errno value in the event of an error.
 *
 * Input Parameters:
 *   pid        - The ID of thread whose affinity set will be modified.
 *   cpusetsize - Size of mask.  MUST be sizeofcpu_set_t().
 *   mask       - The location to return the thread's new affinity set.
 *
 * Returned Value:
 *   0 if successful.  Otherwise, ERROR (-1) is returned, and errno is
 *   set appropriately:
 *
 *     ESRCH  The task whose ID is pid could not be found.
 *
 ****************************************************************************/

int sched_setaffinity(pid_t pid, size_t cpusetsize,
                      FAR const cpu_set_t *mask)
{
  int ret = nxsched_set_affinity(pid, cpusetsize, mask);
  if (ret < 0)
    {
      set_errno(-ret);
      ret = ERROR;
    }

  return ret;
}
