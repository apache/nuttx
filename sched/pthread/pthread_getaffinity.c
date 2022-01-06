/****************************************************************************
 * sched/pthread/pthread_getaffinity.c
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

#include <sys/types.h>
#include <stdint.h>
#include <pthread.h>
#include <sched.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/sched.h>

#include "pthread/pthread.h"

#ifdef CONFIG_SMP

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pthread_getschedparam
 *
 * Description:
 *   The pthread_getaffinity_np() function returns the CPU affinity mask
 *   of the thread thread in the buffer pointed to by cpuset.
 *
 *   The argument cpusetsize is the length (in bytes) of the buffer
 *   pointed to by cpuset.  Typically, this argument would be specified as
 *   sizeof(cpu_set_t).
 *
 * Input Parameters:
 *   thread     - The ID of thread whose affinity set will be retrieved.
 *   cpusetsize - Size of cpuset.  MUST be sizeofcpu_set_t().
 *   cpuset     - The location to return the thread's new affinity set.
 *
 * Returned Value:
 *   0 if successful.  Otherwise, an errno value is returned indicating the
 *   nature of the failure.
 *
 ****************************************************************************/

int pthread_getaffinity_np(pthread_t thread, size_t cpusetsize,
                           FAR cpu_set_t *cpuset)
{
  int ret;

  DEBUGASSERT(thread > 0 && cpusetsize == sizeof(cpu_set_t) &&
              cpuset != NULL);

  sinfo("thread ID=%d cpusetsize=%zu cpuset=%ju\n",
        (int)thread, cpusetsize, (uintmax_t)*cpuset);

  /* Let nxsched_get_affinity do all of the work */

  ret = nxsched_get_affinity((pid_t)thread, cpusetsize, cpuset);
  if (ret < 0)
    {
      /* If nxsched_get_affinity() fails, return the positive errno */

      ret = -ret;
    }

  return ret;
}

#endif /* CONFIG_SMP */
