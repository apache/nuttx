/****************************************************************************
 * arch/sim/src/sim/up_vfork.c
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

#include <inttypes.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/sched.h>
#include <nuttx/arch.h>
#include <arch/irq.h>

#include "sched/sched.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_vfork
 *
 * Description:
 *   The vfork() function has the same effect as fork(), except that the
 *   behavior is undefined if the process created by vfork() either modifies
 *   any data other than a variable of type pid_t used to store the return
 *   value from vfork(), or returns from the function in which vfork() was
 *   called, or calls any other function before successfully calling _exit()
 *   or one of the exec family of functions.
 *
 *   The overall sequence is:
 *
 *   1) User code calls vfork().  vfork() collects context information and
 *      transfers control up up_vfork().
 *   2) up_vfork() and calls nxtask_setup_vfork().
 *   3) nxtask_setup_vfork() allocates and configures the child task's TCB.
 *      This consists of:
 *      - Allocation of the child task's TCB.
 *      - Initialization of file descriptors and streams
 *      - Configuration of environment variables
 *      - Allocate and initialize the stack
 *      - Setup the input parameters for the task.
 *      - Initialization of the TCB (including call to up_initial_state())
 *   4) up_vfork() provides any additional operating context. up_vfork must:
 *      - Initialize special values in any CPU registers that were not
 *        already configured by up_initial_state()
 *   5) up_vfork() then calls nxtask_start_vfork()
 *   6) nxtask_start_vfork() then executes the child thread.
 *
 * nxtask_abort_vfork() may be called if an error occurs between steps 3 and
 * 6.
 *
 * Returned Value:
 *   Upon successful completion, vfork() returns 0 to the child process and
 *   returns the process ID of the child process to the parent process.
 *   Otherwise, -1 is returned to the parent, no child process is created,
 *   and errno is set to indicate the error.
 *
 ****************************************************************************/

pid_t up_vfork(const xcpt_reg_t *context)
{
  struct tcb_s *parent = this_task();
  struct task_tcb_s *child;
  xcpt_reg_t newsp;
  xcpt_reg_t newfp;
  xcpt_reg_t newtop;
  xcpt_reg_t stacktop;
  xcpt_reg_t stackutil;

  sinfo("vfork context [%p]:\n", context);
  sinfo("  frame pointer:%lx sp:%lx pc:%lx\n",
        context[JB_FP], context[JB_SP], context[JB_PC]);

  /* Allocate and initialize a TCB for the child task. */

  child = nxtask_setup_vfork((start_t)context[JB_PC]);
  if (!child)
    {
      serr("ERROR: nxtask_setup_vfork failed\n");
      return (pid_t)ERROR;
    }

  sinfo("TCBs: Parent=%p Child=%p\n", parent, child);

  /* How much of the parent's stack was utilized?  The ARM uses
   * a push-down stack so that the current stack pointer should
   * be lower than the initial, adjusted stack pointer.  The
   * stack usage should be the difference between those two.
   */

  stacktop = (xcpt_reg_t)parent->stack_base_ptr +
                         parent->adj_stack_size;
  DEBUGASSERT(stacktop > context[JB_SP]);
  stackutil = stacktop - context[JB_SP];

  sinfo("Parent: stackutil:%lu\n", stackutil);

  /* Make some feeble effort to preserve the stack contents.  This is
   * feeble because the stack surely contains invalid pointers and other
   * content that will not work in the child context.  However, if the
   * user follows all of the caveats of vfork() usage, even this feeble
   * effort is overkill.
   */

  newtop = (xcpt_reg_t)child->cmn.stack_base_ptr +
                       child->cmn.adj_stack_size;
  newsp = newtop - stackutil;
  memcpy((void *)newsp, (const void *)context[JB_SP], stackutil);

  /* Was there a frame pointer in place before? */

  if (context[JB_FP] >= context[JB_SP] && context[JB_FP] < stacktop)
    {
      xcpt_reg_t frameutil = stacktop - context[JB_FP];
      newfp = newtop - frameutil;
    }
  else
    {
      newfp = context[JB_FP];
    }

  sinfo("Old stack top:%lx SP:%lx FP:%lx\n",
        stacktop, context[JB_SP], context[JB_FP]);
  sinfo("New stack top:%lx SP:%lx FP:%lx\n",
        newtop, newsp, newfp);

  /* Update the stack pointer, frame pointer, and volatile registers.  When
   * the child TCB was initialized, all of the values were set to zero.
   * up_initial_state() altered a few values, but the return value in R0
   * should be cleared to zero, providing the indication to the newly started
   * child thread.
   */

  memcpy(child->cmn.xcp.regs, context,
         sizeof(xcpt_reg_t) * XCPTCONTEXT_REGS);
  child->cmn.xcp.regs[JB_FP] = newfp; /* Frame pointer */
  child->cmn.xcp.regs[JB_SP] = newsp; /* Stack pointer */

  /* And, finally, start the child task.  On a failure, nxtask_start_vfork()
   * will discard the TCB by calling nxtask_abort_vfork().
   */

  return nxtask_start_vfork(child);
}
