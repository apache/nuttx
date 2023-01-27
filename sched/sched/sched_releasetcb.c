/****************************************************************************
 * sched/sched/sched_releasetcb.c
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

#include "sched/sched.h"
#include "group/group.h"
#include "timer/timer.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  nxsched_releasepid
 *
 * Description:  When a task is destroyed, this function must
 * be called to make its process ID available for re-use.
 ****************************************************************************/

static void nxsched_releasepid(pid_t pid)
{
  irqstate_t flags = enter_critical_section();
  int hash_ndx = PIDHASH(pid);

#ifdef CONFIG_SCHED_CPULOAD
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

  leave_critical_section(flags);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

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
#ifndef CONFIG_DISABLE_POSIX_TIMERS
      /* Release any timers that the task might hold.  We do this
       * before release the PID because it may still be trying to
       * deliver signals (although interrupts are should be
       * disabled here).
       */

      timer_deleteall(tcb->pid);
#endif

      /* Release the task's process ID if one was assigned.  PID
       * zero is reserved for the IDLE task.  The TCB of the IDLE
       * task is never release so a value of zero simply means that
       * the process ID was never allocated to this TCB.
       */

      if (tcb->pid)
        {
          nxsched_releasepid(tcb->pid);
        }

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

      ret = up_addrenv_detach(tcb);
      if (ttype == TCB_FLAG_TTYPE_TASK)
        {
          addrenv_free(tcb);
        }
#endif

      /* Leave the group (if we did not already leave in task_exithook.c) */

      group_leave(tcb);

      /* And, finally, release the TCB itself */

      kmm_free(tcb);
    }

  return ret;
}
