/****************************************************************************
 * sched/sched/sched_gettcb.c
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

#include <sys/types.h>

#include <sched.h>

#include "nuttx/irq.h"

#include "sched/sched.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsched_get_tcb
 *
 * Description:
 *   Given a task ID, this function will return the a pointer to the
 *   corresponding TCB (or NULL if there is no such task ID).
 *
 *   NOTE:  This function holds a critical section while examining TCB data
 *   data structures but releases that critical section before returning.
 *   When it is released, the TCB may become unstable.  If the caller
 *   requires absolute stability while using the TCB, then the caller
 *   should establish the critical section BEFORE calling this function and
 *   hold that critical section as long as necessary.
 *
 ****************************************************************************/

FAR struct tcb_s *nxsched_get_tcb(pid_t pid)
{
  FAR struct tcb_s *ret = NULL;
  irqstate_t flags;
  int hash_ndx;

  flags = spin_lock_irqsave_wo_note(NULL);

  /* Verify whether g_pidhash hash table has already been allocated and
   * whether the PID is within range.
   */

  if (g_pidhash != NULL && pid >= 0)
    {
      /* The test and the return setup should be atomic.  This still does
       * not provide proper protection if the recipient of the TCB does not
       * also protect against the task associated with the TCB from
       * terminating asynchronously.
       */

      /* Get the hash_ndx associated with the pid */

      hash_ndx = PIDHASH(pid);

      /* Verify that the correct TCB was found. */

      if (g_pidhash[hash_ndx] != NULL && pid == g_pidhash[hash_ndx]->pid)
        {
          /* Return the TCB associated with this pid (if any) */

          ret = g_pidhash[hash_ndx];
        }
    }

  spin_unlock_irqrestore_wo_note(NULL, flags);

  /* Return the TCB. */

  return ret;
}
