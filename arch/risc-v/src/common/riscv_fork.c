/****************************************************************************
 * arch/risc-v/src/common/riscv_fork.c
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

#include "riscv_fork.h"
#include "riscv_internal.h"

#include "sched/sched.h"

#ifdef CONFIG_ARCH_HAVE_FORK

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
 * Name: riscv_fork
 *
 * Description:
 *   The fork() function has the same effect as posix fork(), except that the
 *   behavior is undefined if the process created by fork() either modifies
 *   any data other than a variable of type pid_t used to store the return
 *   value from fork(), or returns from the function in which fork() was
 *   called, or calls any other function before successfully calling _exit()
 *   or one of the exec family of functions.
 *
 *   The overall sequence is:
 *
 *   1) User code calls fork().  fork() collects context information and
 *      transfers control up riscv_fork().
 *   2) riscv_fork() and calls nxtask_setup_fork().
 *   3) nxtask_setup_fork() allocates and configures the child task's TCB.
 *     This consists of:
 *      - Allocation of the child task's TCB.
 *      - Initialization of file descriptors and streams
 *      - Configuration of environment variables
 *      - Allocate and initialize the stack
 *      - Setup the input parameters for the task.
 *      - Initialization of the TCB (including call to up_initial_state())
 *   4) riscv_fork() provides any additional operating context. riscv_fork
 *      must:
 *      - Initialize special values in any CPU registers that were not
 *        already configured by up_initial_state()
 *   5) riscv_fork() then calls nxtask_start_fork()
 *   6) nxtask_start_fork() then executes the child thread.
 *
 * nxtask_abort_fork() may be called if an error occurs between steps 3
 * and 6.
 *
 * Input Parameters:
 *   context - Caller context information saved by fork()
 *
 * Returned Value:
 *   Upon successful completion, fork() returns 0 to the child process and
 *   returns the process ID of the child process to the parent process.
 *   Otherwise, -1 is returned to the parent, no child process is created,
 *   and errno is set to indicate the error.
 *
 ****************************************************************************/

#ifdef CONFIG_LIB_SYSCALL

pid_t riscv_fork(const struct fork_s *context)
{
  struct tcb_s *parent = this_task();
  struct task_tcb_s *child;
  uintptr_t newsp;
  uintptr_t newtop;
  uintptr_t stacktop;
  uintptr_t stackutil;
#ifdef CONFIG_SCHED_THREAD_LOCAL
  uintptr_t tp;
#endif
  UNUSED(context);

  /* Allocate and initialize a TCB for the child task. */

  child = nxtask_setup_fork((start_t)parent->xcp.sregs[REG_RA]);
  if (!child)
    {
      sinfo("nxtask_setup_fork failed\n");
      return (pid_t)ERROR;
    }

  /* Copy parent user stack to child */

  stacktop = (uintptr_t)parent->stack_base_ptr + parent->adj_stack_size;
  DEBUGASSERT(stacktop > parent->xcp.sregs[REG_SP]);
  stackutil = stacktop - parent->xcp.sregs[REG_SP];

  /* Copy goes to child's user stack top */

  newtop = (uintptr_t)child->cmn.stack_base_ptr + child->cmn.adj_stack_size;
  newsp = newtop - stackutil;

  memcpy((void *)newsp, (const void *)parent->xcp.sregs[REG_SP], stackutil);

#ifdef CONFIG_SCHED_THREAD_LOCAL
  /* Save child's thread pointer */

  tp = child->cmn.xcp.regs[REG_TP];
#endif

  /* Determine the integer context save area */

#ifdef CONFIG_ARCH_KERNEL_STACK
  if (child->cmn.xcp.kstack)
    {
      /* Set context to kernel stack */

      stacktop = (uintptr_t)child->cmn.xcp.ktopstk;
    }
  else
#endif
    {
      /* Set context to user stack */

      stacktop = newsp;
    }

  /* Set the new register restore area to the new stack top */

  child->cmn.xcp.regs = (void *)(stacktop - XCPTCONTEXT_SIZE);

  /* Copy the parent integer context (overwrites child's SP and TP) */

  memcpy(child->cmn.xcp.regs, parent->xcp.sregs, XCPTCONTEXT_SIZE);

  /* Save FPU */

  riscv_savefpu(child->cmn.xcp.regs, riscv_fpuregs(&child->cmn));

  /* Return 0 to child */

  child->cmn.xcp.regs[REG_A0] = 0;
  child->cmn.xcp.regs[REG_SP] = newsp;
#ifdef CONFIG_SCHED_THREAD_LOCAL
  child->cmn.xcp.regs[REG_TP] = tp;
#endif

  /* And, finally, start the child task.  On a failure, nxtask_start_fork()
   * will discard the TCB by calling nxtask_abort_fork().
   */

  return nxtask_start_fork(child);
}

#else

pid_t riscv_fork(const struct fork_s *context)
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
#ifdef CONFIG_ARCH_FPU
  uintreg_t *fregs;
#endif

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

  child = nxtask_setup_fork((start_t)(uintptr_t)context->ra);
  if (!child)
    {
      sinfo("nxtask_setup_fork failed\n");
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

  sinfo("Parent: stackutil:%" PRIxPTR "\n", stackutil);

  /* Make some feeble effort to preserve the stack contents.  This is
   * feeble because the stack surely contains invalid pointers and other
   * content that will not work in the child context.  However, if the
   * user follows all of the caveats of fork() usage, even this feeble
   * effort is overkill.
   */

  newtop = (uintptr_t)child->cmn.stack_base_ptr + child->cmn.adj_stack_size;
  newsp = newtop - stackutil;

  /* Set up frame for context and copy the initial context there */

  memcpy((void *)(newsp - XCPTCONTEXT_SIZE),
         child->cmn.xcp.regs, XCPTCONTEXT_SIZE);

  /* Copy the parent stack contents (overwrites child's SP and TP) */

  memcpy((void *)newsp, (const void *)(uintptr_t)context->sp, stackutil);

  /* Set the new register restore area to the new stack top */

  child->cmn.xcp.regs = (void *)(newsp - XCPTCONTEXT_SIZE);

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

  sinfo("Old stack top:%" PRIxPTR " SP:%" PRIxREG " FP:%" PRIxREG "\n",
        stacktop, context->sp, context->fp);
  sinfo("New stack top:%" PRIxPTR " SP:%" PRIxPTR " FP:%" PRIxPTR "\n",
        newtop, newsp, newfp);
#else
  sinfo("Old stack top:%" PRIxPTR " SP:%" PRIxREG "\n",
        stacktop, context->sp);
  sinfo("New stack top:%" PRIxPTR " SP:%" PRIxPTR "\n",
        newtop, newsp);
#endif

  /* Update the stack pointer, frame pointer, global pointer and saved
   * registers.  When the child TCB was initialized, all of the values
   * were set to zero. up_initial_state() altered a few values, but the
   * return value in v0 should be cleared to zero, providing the
   * indication to the newly started child thread.
   */

  child->cmn.xcp.regs[REG_S1]   = context->s1;  /* Saved register s1 */
  child->cmn.xcp.regs[REG_S2]   = context->s2;  /* Saved register s2 */
  child->cmn.xcp.regs[REG_S3]   = context->s3;  /* Saved register s3 */
  child->cmn.xcp.regs[REG_S4]   = context->s4;  /* Saved register s4 */
  child->cmn.xcp.regs[REG_S5]   = context->s5;  /* Saved register s5 */
  child->cmn.xcp.regs[REG_S6]   = context->s6;  /* Saved register s6 */
  child->cmn.xcp.regs[REG_S7]   = context->s7;  /* Saved register s7 */
  child->cmn.xcp.regs[REG_S8]   = context->s8;  /* Saved register s8 */
  child->cmn.xcp.regs[REG_S9]   = context->s9;  /* Saved register s9 */
  child->cmn.xcp.regs[REG_S10]  = context->s10; /* Saved register s10 */
  child->cmn.xcp.regs[REG_S11]  = context->s11; /* Saved register s11 */
#ifdef CONFIG_RISCV_FRAMEPOINTER
  child->cmn.xcp.regs[REG_FP]   = newfp;        /* Frame pointer */
#else
  child->cmn.xcp.regs[REG_S0]   = context->s0;  /* Saved register s0 */
#endif
  child->cmn.xcp.regs[REG_SP]   = newsp;        /* Stack pointer */
#ifdef RISCV_SAVE_GP
  child->cmn.xcp.regs[REG_GP]   = context->gp;  /* Global pointer */
#endif
#ifdef CONFIG_ARCH_FPU
  fregs                         = riscv_fpuregs(&child->cmn);
  fregs[REG_FS0]                = context->fs0;  /* Saved register fs1 */
  fregs[REG_FS1]                = context->fs1;  /* Saved register fs1 */
  fregs[REG_FS2]                = context->fs2;  /* Saved register fs2 */
  fregs[REG_FS3]                = context->fs3;  /* Saved register fs3 */
  fregs[REG_FS4]                = context->fs4;  /* Saved register fs4 */
  fregs[REG_FS5]                = context->fs5;  /* Saved register fs5 */
  fregs[REG_FS6]                = context->fs6;  /* Saved register fs6 */
  fregs[REG_FS7]                = context->fs7;  /* Saved register fs7 */
  fregs[REG_FS8]                = context->fs8;  /* Saved register fs8 */
  fregs[REG_FS9]                = context->fs9;  /* Saved register fs9 */
  fregs[REG_FS10]               = context->fs10; /* Saved register fs10 */
  fregs[REG_FS11]               = context->fs11; /* Saved register fs11 */
#endif

  /* And, finally, start the child task.  On a failure, nxtask_start_fork()
   * will discard the TCB by calling nxtask_abort_fork().
   */

  return nxtask_start_fork(child);
}

#endif /* CONFIG_LIB_SYSCALL */
#endif /* CONFIG_ARCH_HAVE_FORK */
