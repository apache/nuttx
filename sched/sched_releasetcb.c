/************************************************************
 * sched_releasetcb.c
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
 * Compilation Switches
 ************************************************************/

/************************************************************
 * Included Files
 ************************************************************/

#include <sched.h>
#include <errno.h>
#include <nuttx/arch.h>
#include "os_internal.h"

/************************************************************
 * Private Functions
 ************************************************************/

/************************************************************
 * Name:  sched_releasepid
 *
 * Description:  When a task is destroyed, this function must
 * be called to make its process ID available for re-use.
 ************************************************************/

static void sched_releasepid(pid_t pid)
{
   int hash_ndx = PIDHASH(pid);

   /* Make any pid associated with this hash available.  Note: 
    * no special precautions need be taken here because the
    * following action is atomic
    */

   g_pidhash[hash_ndx].tcb = NULL;
   g_pidhash[hash_ndx].pid = INVALID_PROCESS_ID;
}

/************************************************************
 * Public Functions
 ************************************************************/

/************************************************************
 * Function:  sched_releasetcb
 *
 * Description:
 *   Free all resources contained in a TCB
 *
 * Parameters:
 *   None
 *
 * Return Value:
 *   OK on success; ERROR on failure
 *
 * Assumptions:
 *
 ************************************************************/

int sched_releasetcb(FAR _TCB *tcb)
{
  int ret = OK;
  int i;

  if (tcb)
    {
      /* Release the task's process ID if one was assigned.  PID
       * zero is reserved for the IDLE task.  The TCB of the IDLE
       * task is never release so a value of zero simply means that
       * the process ID was never allocated to this TCB.
       */

      if (tcb->pid)
        {
          sched_releasepid(tcb->pid);
        }

      /* Delete the thread's stack if one has been allocated */

      if (tcb->stack_alloc_ptr)
        {
          up_release_stack(tcb);
        }

      /* Release command line arguments that were allocated
       * for task start/re-start.
       */

      if ((tcb->flags & TCB_FLAG_PTHREAD) == 0)
        {
          for (i = 1; i < NUM_TASK_ARGS+1 && tcb->argv[i]; i++)
            {
              sched_free(tcb->argv[i]);
            }
        }

      /* Release any allocated file structures */

      ret = sched_releasefiles(tcb);

      /* And, finally, release the TCB itself */

      sched_free(tcb);
    }
  return ret;
}

