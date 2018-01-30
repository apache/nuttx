/****************************************************************************
 * sched/sched/sched_getaffinity.c
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
#include <sched.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/sched.h>

#include "sched/sched.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsched_getaffinity
 *
 * Description:
 *   nxsched_getaffinity() writes the affinity mask of the thread whose ID
 *   is pid into the cpu_set_t pointed to by mask.  The  cpusetsize
 *   argument specifies the size (in bytes) of mask.  If pid is zero, then
 *   the mask of the calling thread is returned.
 *
 *   nxsched_getaffinity() is identical to the function sched_getaffinity(),
 *   differing only in its return value:  This function does not modify the
 *   errno variable.
 *
 *   This is a non-standard, internal OS function and is not intended for
 *   use by application logic.  Applications should use the standard
 *   sched_getparam().
 *
 * Input Parameters:
 *   pid        - The ID of thread whose affinity set will be retrieved.
 *   cpusetsize - Size of mask.  MUST be sizeofcpu_set_t().
 *   mask       - The location to return the thread's new affinity set.
 *
 * Returned Value:
 *   Zero (OK) if successful.  Otherwise, a negated errno value is returned:
 *
 *      ESRCH  The task whose ID is pid could not be found.
 *
 ****************************************************************************/

int nxsched_getaffinity(pid_t pid, size_t cpusetsize, FAR cpu_set_t *mask)
{
  FAR struct tcb_s *tcb;
  int ret;

  DEBUGASSERT(cpusetsize == sizeof(cpu_set_t) && mask != NULL);

  /* Verify that the PID corresponds to a real task */

  sched_lock();
  if (pid == 0)
    {
      tcb = this_task();
    }
  else
    {
      tcb = sched_gettcb(pid);
    }

  if (tcb == NULL)
    {
      ret = -ESRCH;
    }
  else
    {
      /* Return the affinity mask from the TCB. */

      *mask = tcb->affinity;
      ret = OK;
    }

  sched_unlock();
  return ret;
}

/****************************************************************************
 * Name: sched_getaffinity
 *
 * Description:
 *   sched_getaffinity() writes the affinity mask of the thread whose ID
 *   is pid into the cpu_set_t pointed to by mask.  The  cpusetsize
 *   argument specifies the size (in bytes) of mask.  If pid is zero, then
 *   the mask of the calling thread is returned.
 *
 *   This function is a simply wrapper around nxsched_getaffinity() that
 *   sets the errno value in the event of an error.
 *
 * Input Parameters:
 *   pid        - The ID of thread whose affinity set will be retrieved.
 *   cpusetsize - Size of mask.  MUST be sizeofcpu_set_t().
 *   mask       - The location to return the thread's new affinity set.
 *
 * Returned Value:
 *   0 if successful.  Otherwise, ERROR (-1) is returned, and errno is
 *   set appropriately:
 *
 *      ESRCH  The task whose ID is pid could not be found.
 *
 ****************************************************************************/

int sched_getaffinity(pid_t pid, size_t cpusetsize, FAR cpu_set_t *mask)
{
  int ret = nxsched_getaffinity(pid, cpusetsize, mask);
  if (ret < 0)
    {
      set_errno(-ret);
      ret = ERROR;
    }

  return ret;
}
