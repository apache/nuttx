/************************************************************
 * sched_setparam.c
 *
 *   Copyright (C) 2007 Gregory Nutt. All rights reserved.
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
 * 3. Neither the name Gregory Nutt nor the names of its contributors may be
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
 ************************************************************/

/************************************************************
 * Included Files
 ************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>
#include <sched.h>
#include <nuttx/arch.h>
#include <nuttx/os_external.h>
#include "os_internal.h"

/************************************************************
 * Definitions
 ************************************************************/

/************************************************************
 * Private Type Declarations
 ************************************************************/

/************************************************************
 * Global Variables
 ************************************************************/

/************************************************************
 * Private Variables
 ************************************************************/

/************************************************************
 * Private Function Prototypes
 ************************************************************/

/************************************************************
 * Private Functions
 ************************************************************/

/************************************************************
 * Public Functions
 ************************************************************/

/************************************************************
 * Name:  sched_setparam
 *
 * Description:
 *   This function sets the priority of a specified task.
 *
 *   NOTE: Setting a task's priority to the same value has the
 *   similar effect to sched_yield() -- The task will be moved to
 *   after all other tasks with the same priority.
 *
 * Inputs:
 *   pid - the task ID of the task to reprioritize.  If pid is
 *     zero, the priority of the calling task is changed.
 *   param - A structure whose member sched_priority is the integer
 *      priority.  The range of valid priority numbers is from
 *      SCHED_PRIORITY_MIN through SCHED_PRIORITY_MAX.
 *
 * Return Value:
 *    OK if successful, otherwise ERROR.  This function can
 *    fail for the following reasons:
 *
 *    (1) parm is NULL or parm->sched_priority is out of
 *        range.
 *    (2) pid does not correspond to any task.
 *
 *    (errno is not set).
 *
 * Assumptions:
 *
 ************************************************************/

int sched_setparam(pid_t pid, const struct sched_param *param)
{
  FAR _TCB  *rtcb;
  FAR _TCB  *tcb;
  tstate_t   task_state;
  irqstate_t saved_state;
  int        sched_priority = param->sched_priority;
  int        ret = 0;

  /* Verify that the requested priority is in the valid range */

  if (!param ||
      param->sched_priority < SCHED_PRIORITY_MIN || 
      param->sched_priority > SCHED_PRIORITY_MAX)
    {
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

          sched_unlock();
          return ERROR;
        }
    }

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

             irqstate_t flags = irqsave();
             up_reprioritize_rtr(tcb, (ubyte)sched_priority);
             irqrestore(flags);
           }

         /* Otherwise, we can just change priority since it has no effect */

         else
           {
             /* Change the task priority */

             tcb->sched_priority = (ubyte)sched_priority;
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

             irqstate_t flags = irqsave();
             up_reprioritize_rtr(tcb, (ubyte)sched_priority);
             irqrestore(flags);
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

            dq_rem((FAR dq_entry_t*)tcb, (dq_queue_t*)g_tasklisttable[task_state].list);

            /* Change the task priority */

            tcb->sched_priority = (ubyte)sched_priority;

            /* Put it back into the prioritized list at the correct
             * position
             */

            sched_addprioritized(tcb, (dq_queue_t*)g_tasklisttable[task_state].list);
          }

        /* CASE 3b. The task resides in a non-prioritized list. */

        else
          {
            /* Just change the task's priority */

            tcb->sched_priority = (ubyte)sched_priority;
          }
        break;
    }

  irqrestore(saved_state);
  sched_unlock();
  return ret;
}
