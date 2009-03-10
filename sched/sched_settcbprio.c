/****************************************************************************
 * sched/sched_settcbprio.c
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
#include <errno.h>
#include <nuttx/arch.h>
#include "os_internal.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Type Declarations
 ****************************************************************************/

/****************************************************************************
 * Global Variables
 ****************************************************************************/

/****************************************************************************
 * Private Variables
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  sched_settcbprio
 *
 * Description:
 *   This function sets the priority of a specified task.
 *
 *   NOTE: Setting a task's priority to the same value has a similar
 *   effect to sched_yield() -- The task will be moved to  after all other
 *   tasks with the same priority.
 *
 * Inputs:
 *   tcb - the TCB of task to reprioritize.
 *   sched_priority - The new task priority
 *
 * Return Value:
 *   On success, sched_setparam() returns 0 (OK). On error, -1
 *  (ERROR) is returned, and errno is set appropriately.
 *
 *  EINVAL The parameter 'param' is invalid or does not make
 *         sense for the current scheduling policy.
 *  EPERM  The calling task does not have appropriate privileges.
 *  ESRCH  The task whose ID is pid could not be found.
 *
 * Assumptions:
 *
 ****************************************************************************/

int sched_settcbprio(FAR _TCB *tcb, int sched_priority)
{
  FAR _TCB  *rtcb = (FAR _TCB*)g_readytorun.head;
  tstate_t   task_state;
  irqstate_t saved_state;

  /* We need to assure that there there is no interrupt activity while
   * performing the following.
   */

  saved_state = irqsave();

  /* There are four cases that must be considered: */

  task_state = tcb->task_state;
  switch (task_state)
    {
       /* CASE 1. The task is running or ready-to-run and a context switch
        * may be caused by the re-prioritization 
        */

       case TSTATE_TASK_RUNNING:

         /* A context switch will occur if the new priority of the running
          * task becomes less than OR EQUAL TO the next highest priority
          * ready to run task.
          */

         if (sched_priority <= tcb->flink->sched_priority)
           {
             /* A context switch will occur. */

             up_reprioritize_rtr(tcb, (ubyte)sched_priority);
           }

         /* Otherwise, we can just change priority since it has no effect */

         else
           {
             /* Change the task priority */

             tcb->sched_priority = (ubyte)sched_priority;
#ifdef CONFIG_PRIORITY_INHERITANCE
             tcb->base_priority  = (ubyte)sched_priority;
#endif
           }
         break;

       /* CASE 2. The task is running or ready-to-run and a context switch
        * may be caused by the re-prioritization
        */

       case TSTATE_TASK_READYTORUN:

         /* A context switch will occur if the new priority of the ready-to
          * run task is (strictly) greater than the current running task 
          */

         if (sched_priority > rtcb->sched_priority)
           {
             /* A context switch will occur. */

             up_reprioritize_rtr(tcb, (ubyte)sched_priority);
           }

         /* Otherwise, we can just change priority and re-schedule (since it
          * have no other effect).
          */

         else
           {
             /* Remove the TCB from the ready-to-run task list */

             ASSERT(!sched_removereadytorun(tcb));

             /* Change the task priority */

             tcb->sched_priority = (ubyte)sched_priority;
#ifdef CONFIG_PRIORITY_INHERITANCE
             tcb->base_priority  = (ubyte)sched_priority;
#endif

             /* Put it back into the ready-to-run task list */

             ASSERT(!sched_addreadytorun(tcb));
           }
         break;

       /* CASE 3. The task is not in the ready to run list.  Changing its
        * Priority cannot effect the currently executing task.
        */

     default:
        /* CASE 3a. The task resides in a prioritized list. */

        if (g_tasklisttable[task_state].prioritized)
          {
            /* Remove the TCB from the prioritized task list */

            dq_rem((FAR dq_entry_t*)tcb, (FAR dq_queue_t*)g_tasklisttable[task_state].list);

            /* Change the task priority */

            tcb->sched_priority = (ubyte)sched_priority;
#ifdef CONFIG_PRIORITY_INHERITANCE
            tcb->base_priority  = (ubyte)sched_priority;
#endif

            /* Put it back into the prioritized list at the correct
             * position
             */

            sched_addprioritized(tcb, (FAR dq_queue_t*)g_tasklisttable[task_state].list);
          }

        /* CASE 3b. The task resides in a non-prioritized list. */

        else
          {
            /* Just change the task's priority */

            tcb->sched_priority = (ubyte)sched_priority;
#ifdef CONFIG_PRIORITY_INHERITANCE
            tcb->base_priority  = (ubyte)sched_priority;
#endif
          }
        break;
    }

  irqrestore(saved_state);
  return OK;
}

int sched_setparam(pid_t pid, const struct sched_param *param)
{
  FAR _TCB  *rtcb;
  FAR _TCB  *tcb;
  int        ret;

  /* Verify that the requested priority is in the valid range */

  if (!param ||
      param->sched_priority < SCHED_PRIORITY_MIN || 
      param->sched_priority > SCHED_PRIORITY_MAX)
    {
      errno = EINVAL;
      return ERROR;
    }

  /* Prohibit modifications to the head of the ready-to-run task
   * list while adjusting the priority
   */

  sched_lock();

  /* Check if the task to reprioritize is the calling task */

  rtcb = (FAR _TCB*)g_readytorun.head;
  if (pid == 0 || pid == rtcb->pid)
    {
      tcb = rtcb;
    }

  /* The pid is not the calling task, we will have to search for it */

  else
    {
      tcb = sched_gettcb(pid);
      if (!tcb)
        {
          /* No task with this pid was found */

          errno = ESRCH;
          sched_unlock();
          return ERROR;
        }
    }

 /* Then perform the reprioritization */

 ret = sched_settcbprio(tcb, param->sched_priority);
 sched_unlock();
 return ret;
}
