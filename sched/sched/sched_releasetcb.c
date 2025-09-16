/****************************************************************************
 * sched/sched/sched_releasetcb.c
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

#include <nuttx/config.h>

#include <sys/types.h>
#include <sched.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/sched.h>

#include "task/task.h"
#include "sched/sched.h"
#include "group/group.h"
#include "timer/timer.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  nxsched_release_pid
 *
 * Description:  When a task is destroyed, this function must
 * be called to make its process ID available for reuse.
 ****************************************************************************/

void nxsched_release_pid(pid_t pid)
{
  irqstate_t flags = enter_critical_section();
  int hash_ndx = PIDHASH(pid);
  FAR struct tcb_s *tcb = g_pidhash[hash_ndx];

  DEBUGASSERT(tcb);
#ifndef CONFIG_SCHED_CPULOAD_NONE
  /* Decrement the total CPU load count held by this thread from the
   * total for all threads.
   */

  g_cpuload_total -= g_pidhash[hash_ndx]->ticks;
#endif

  /* Make any pid associated with this hash available.  Note:
   * no special precautions need be taken here because the
   * following action is atomic
   */

  g_pidhash[hash_ndx] = NULL;
  DEBUGASSERT(atomic_read(&tcb->refs) > 0);
  atomic_fetch_sub(&tcb->refs, 1);

  leave_critical_section(flags);

  /* Wait tcb->refs to be 0 */

  while (atomic_read(&tcb->refs))
    {
      nxsem_wait(&tcb->exit_sem);
    }
}

/****************************************************************************
 * Name: nxsched_release_tcb
 *
 * Description:
 *   Free all resources contained in a TCB
 *
 * Input Parameters:
 *   tcb - The TCB to be released
 *   ttype - The type of the TCB to be released
 *
 *   This thread type is normally available in the flags field of the TCB,
 *   however, there are certain error recovery contexts where the TCB my
 *   not be fully initialized when nxsched_release_tcb is called.
 *
 * Returned Value:
 *   OK on success; ERROR on failure
 *
 * Assumptions:
 *   Interrupts are disabled.
 *
 ****************************************************************************/

int nxsched_release_tcb(FAR struct tcb_s *tcb, uint8_t ttype)
{
  int ret = OK;

  if (tcb)
    {
      /* Released tcb shouldn't on any list */

      DEBUGASSERT(tcb->flink == NULL && tcb->blink == NULL);

#ifndef CONFIG_DISABLE_POSIX_TIMERS
      /* Release any timers that the task might hold.  We do this
       * before release the PID because it may still be trying to
       * deliver signals (although interrupts are should be
       * disabled here).
       */

      timer_deleteall(tcb->pid);
#endif

      /* Delete the thread's stack if one has been allocated */

      if (tcb->stack_alloc_ptr)
        {
          up_release_stack(tcb, ttype);
        }

#ifdef CONFIG_PIC
      /* Delete the task's allocated DSpace region (external modules only) */

      if (tcb->dspace != NULL)
        {
          if (tcb->dspace->crefs <= 1)
            {
              kmm_free(tcb->dspace);
            }
          else
            {
              tcb->dspace->crefs--;
            }
        }
#endif

#if defined(CONFIG_ARCH_ADDRENV) && defined(CONFIG_ARCH_KERNEL_STACK)
      /* Release the kernel stack */

      up_addrenv_kstackfree(tcb);
#endif

#ifdef CONFIG_ARCH_ADDRENV
      /* Release this thread's reference to the address environment */

      ret = addrenv_leave(tcb);
#endif

      /* Leave the group (if we did not already leave in task_exithook.c) */

      group_leave(tcb);

#ifndef CONFIG_DISABLE_PTHREAD
      /* Destroy the pthread join mutex */

      nxtask_joindestroy(tcb);
#endif

      nxsem_destroy(&tcb->exit_sem);

      /* And, finally, release the TCB itself */

      if (tcb->flags & TCB_FLAG_FREE_TCB)
        {
          kmm_free(tcb);
        }
    }

  return ret;
}
