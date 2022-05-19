/****************************************************************************
 * arch/risc-v/src/common/riscv_vfork.c
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

#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/sched.h>
#include <nuttx/arch.h>
#include <arch/irq.h>

#include "riscv_vfork.h"
#include "riscv_internal.h"

#include "sched/sched.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

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
 *     This consists of:
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
 * nxtask_abort_vfork() may be called if an error occurs between steps 3
 * and 6.
 *
 * Input Parameters:
 *   context - Caller context information saved by vfork()
 *
 * Returned Value:
 *   Upon successful completion, vfork() returns 0 to the child process and
 *   returns the process ID of the child process to the parent process.
 *   Otherwise, -1 is returned to the parent, no child process is created,
 *   and errno is set to indicate the error.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_HAVE_VFORK

pid_t up_vfork(const struct vfork_s *context)
{
  struct tcb_s *parent = this_task();
  struct task_tcb_s *child;
  uintptr_t newsp;
#ifdef CONFIG_RISCV_FRAMEPOINTER
  uintptr_t newfp;
#endif
  uintptr_t newtop;
  uintptr_t stacktop;
  uintptr_t stackutil;

  sinfo("s0:%" PRIxREG " s1:%" PRIxREG " s2:%" PRIxREG " s3:%" PRIxREG ""
        " s4:%" PRIxREG "\n",
        context->s0, context->s1, context->s2, context->s3, context->s4);
#ifdef CONFIG_RISCV_FRAMEPOINTER
  sinfo("s5:%" PRIxREG " s6:%" PRIxREG " s7:%" PRIxREG "\n",
        context->s5, context->s6, context->s7);
#ifdef RISCV_SAVE_GP
  sinfo("fp:%" PRIxREG " sp:%" PRIxREG " ra:%" PRIxREG " gp:%" PRIxREG "\n",
        context->fp, context->sp, context->ra, context->gp);
#else
  sinfo("fp:%" PRIxREG " sp:%" PRIxREG " ra:%" PRIxREG "\n",
        context->fp context->sp, context->ra);
#endif
#else
  sinfo("s5:%" PRIxREG " s6:%" PRIxREG " s7:%" PRIxREG " s8:%" PRIxREG "\n",
        context->s5, context->s6, context->s7, context->s8);
#ifdef RISCV_SAVE_GP
  sinfo("sp:%" PRIxREG " ra:%" PRIxREG " gp:%" PRIxREG "\n",
        context->sp, context->ra, context->gp);
#else
  sinfo("sp:%" PRIxREG " ra:%" PRIxREG "\n",
        context->sp, context->ra);
#endif
#endif

  /* Allocate and initialize a TCB for the child task. */

  child = nxtask_setup_vfork((start_t)context->ra);
  if (!child)
    {
      sinfo("nxtask_setup_vfork failed\n");
      return (pid_t)ERROR;
    }

  sinfo("Parent=%p Child=%p\n", parent, child);

  /* How much of the parent's stack was utilized?  The RISC-V uses
   * a push-down stack so that the current stack pointer should
   * be lower than the initial, adjusted stack pointer.  The
   * stack usage should be the difference between those two.
   */

  stacktop = (uintptr_t)parent->stack_base_ptr + parent->adj_stack_size;
  DEBUGASSERT(stacktop > context->sp);
  stackutil = stacktop - context->sp;

  sinfo("Parent: stackutil:%" PRIxREG "\n", stackutil);

  /* Make some feeble effort to preserve the stack contents.  This is
   * feeble because the stack surely contains invalid pointers and other
   * content that will not work in the child context.  However, if the
   * user follows all of the caveats of vfork() usage, even this feeble
   * effort is overkill.
   */

  newtop = (uintptr_t)child->cmn.stack_base_ptr + child->cmn.adj_stack_size;
  newsp = newtop - stackutil;

  /* Set up frame for context */

  memcpy((void *)(newsp - XCPTCONTEXT_SIZE),
         child->cmn.xcp.regs, XCPTCONTEXT_SIZE);

  child->cmn.xcp.regs = (void *)(newsp - XCPTCONTEXT_SIZE);
  memcpy((void *)newsp, (const void *)context->sp, stackutil);

  /* Was there a frame pointer in place before? */

#ifdef CONFIG_RISCV_FRAMEPOINTER
  if (context->fp >= context->sp && context->fp < stacktop)
    {
      uintptr_t frameutil = stacktop - context->fp;
      newfp = newtop - frameutil;
    }
  else
    {
      newfp = context->fp;
    }

  sinfo("Old stack top:%" PRIxREG " SP:%" PRIxREG " FP:%" PRIxREG "\n",
        stacktop, context->sp, context->fp);
  sinfo("New stack top:%" PRIxREG " SP:%" PRIxREG " FP:%" PRIxREG "\n",
        newtop, newsp, newfp);
#else
  sinfo("Old stack top:%" PRIxREG " SP:%" PRIxREG "\n",
        stacktop, context->sp);
  sinfo("New stack top:%" PRIxREG " SP:%" PRIxREG "\n",
        newtop, newsp);
#endif

  /* Update the stack pointer, frame pointer, global pointer and saved
   * registers.  When the child TCB was initialized, all of the values
   * were set to zero. up_initial_state() altered a few values, but the
   * return value in v0 should be cleared to zero, providing the
   * indication to the newly started child thread.
   */

  child->cmn.xcp.regs[REG_S1]  = context->s1;  /* Saved register s1 */
  child->cmn.xcp.regs[REG_S2]  = context->s2;  /* Saved register s2 */
  child->cmn.xcp.regs[REG_S3]  = context->s3;  /* Saved register s3 */
  child->cmn.xcp.regs[REG_S4]  = context->s4;  /* Saved register s4 */
  child->cmn.xcp.regs[REG_S5]  = context->s5;  /* Saved register s5 */
  child->cmn.xcp.regs[REG_S6]  = context->s6;  /* Saved register s6 */
  child->cmn.xcp.regs[REG_S7]  = context->s7;  /* Saved register s7 */
  child->cmn.xcp.regs[REG_S8]  = context->s8;  /* Saved register s8 */
  child->cmn.xcp.regs[REG_S9]  = context->s9;  /* Saved register s9 */
  child->cmn.xcp.regs[REG_S10] = context->s10; /* Saved register s10 */
  child->cmn.xcp.regs[REG_S11] = context->s11; /* Saved register s11 */
#ifdef CONFIG_RISCV_FRAMEPOINTER
  child->cmn.xcp.regs[REG_FP]  = newfp;        /* Frame pointer */
#else
  child->cmn.xcp.regs[REG_S0]  = context->s0;  /* Saved register s0 */
#endif
  child->cmn.xcp.regs[REG_SP]  = newsp;        /* Stack pointer */
#ifdef RISCV_SAVE_GP
  child->cmn.xcp.regs[REG_GP]  = newsp;        /* Global pointer */
#endif

#ifdef CONFIG_LIB_SYSCALL
  /* If we got here via a syscall, then we are going to have to setup some
   * syscall return information as well.
   */

  if (parent->xcp.nsyscalls > 0)
    {
      int index;
      for (index = 0; index < parent->xcp.nsyscalls; index++)
        {
          child->cmn.xcp.syscall[index].sysreturn =
            parent->xcp.syscall[index].sysreturn;

#ifndef CONFIG_BUILD_FLAT
          child->cmn.xcp.syscall[index].int_ctx =
            parent->xcp.syscall[index].int_ctx;
#endif
        }

      child->cmn.xcp.nsyscalls = parent->xcp.nsyscalls;
    }
#endif /* CONFIG_LIB_SYSCALL */

  /* And, finally, start the child task.  On a failure, nxtask_start_vfork()
   * will discard the TCB by calling nxtask_abort_vfork().
   */

  return nxtask_start_vfork(child);
}

#endif /* CONFIG_ARCH_HAVE_VFORK */
