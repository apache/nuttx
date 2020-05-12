/****************************************************************************
 * sched/pthread/pthread_getaffinity.c
 *
 *   Copyright (C) 2016, 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <pthread.h>
#include <sched.h>
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

  sinfo("thread ID=%d cpusetsize=%d cpuset=%p\n",
        (int)thread, (int)cpusetsize, cpusetsize);

  DEBUGASSERT(thread > 0 && cpusetsize == sizeof(cpu_set_t) &&
              cpuset != NULL);

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
