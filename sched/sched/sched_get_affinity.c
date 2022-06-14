/****************************************************************************
 * sched/sched/sched_get_affinity.c
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
#include <sched.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/sched.h>

#include "sched/sched.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsched_get_affinity
 *
 * Description:
 *   nxsched_get_affinity() writes the affinity mask of the thread whose ID
 *   is pid into the cpu_set_t pointed to by mask.  The  cpusetsize
 *   argument specifies the size (in bytes) of mask.  If pid is zero, then
 *   the mask of the calling thread is returned.
 *
 *   nxsched_get_affinity() is identical to the function sched_getaffinity(),
 *   differing only in its return value:  This function does not modify the
 *   errno variable.
 *
 *   This is a non-standard, internal OS function and is not intended for
 *   use by application logic.  Applications should use the standard
 *   sched_getaffinity().
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

int nxsched_get_affinity(pid_t pid, size_t cpusetsize, FAR cpu_set_t *mask)
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
      tcb = nxsched_get_tcb(pid);
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
