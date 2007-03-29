/************************************************************
 * up_exit.c
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
#include <debug.h>
#include <nuttx/arch.h>
#include "os_internal.h"
#include "up_internal.h"

/************************************************************
 * Private Definitions
 ************************************************************/

/************************************************************
 * Private Data
 ************************************************************/

/************************************************************
 * Private Funtions
 ************************************************************/

/************************************************************
 * Public Funtions
 ************************************************************/

/************************************************************
 * Name: _exit
 *
 * Description:
 *   This function causes the currently executing task to cease
 *   to exist.  This is a special case of task_delete().
 *
 ************************************************************/

void _exit(int status)
{
  _TCB* tcb = (_TCB*)g_readytorun.head;
  irqstate_t flags;

  dbg("TCB=%p exitting\n", tcb);

  /* Remove the tcb task from the ready-to-run list.  We can
   * ignore the return value because we know that a context
   * switch is needed.
   */

  (void)sched_removereadytorun(tcb);

  /* We are not in a bad stack-- the head of the ready to run task list
   * does not correspond to the thread that is running.  Disabling pre-
   * emption on this TCB should be enough to keep things stable.
   */

  sched_lock();

  /* Move the TCB to the specified blocked task list and delete it */

  sched_addblocked(tcb, TSTATE_TASK_INACTIVE);
  task_delete(tcb->pid);

  /* If there are any pending tasks, then add them to the g_readytorun
   * task list now.
   */

  if (g_pendingtasks.head)
    {
      (void)sched_mergepending();
    }

  /* Now calling sched_unlock() should have no effect */

  sched_unlock();

  /* Now, perform the context switch to the new ready-to-run task at the
   * head of the list.
   */

  tcb = (_TCB*)g_readytorun.head;
  dbg("New Active Task TCB=%p\n", tcb);

  /* The way that we handle signals in the simulation is kind of
   * a kludge.  This would be unsafe in a truly multi-threaded, interrupt
   * driven environment.
   */

  if (tcb->xcp.sigdeliver)
    {
      dbg("Delivering signals TCB=%p\n", tcb);
      ((sig_deliver_t)tcb->xcp.sigdeliver)(tcb);
      tcb->xcp.sigdeliver = NULL;
    }

  /* Then switch contexts */

  up_longjmp(tcb->xcp.regs, 1);
}

