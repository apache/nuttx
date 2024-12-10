/****************************************************************************
 * arch/arm64/src/common/arm64_fork.c
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

#include <inttypes.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/sched.h>
#include <nuttx/arch.h>
#include <arch/barriers.h>
#include <arch/irq.h>
#include "sched/sched.h"

#include "arm64_arch.h"
#include "arm64_fork.h"
#include "arm64_internal.h"
#include "arm64_fatal.h"

#ifdef CONFIG_ARCH_FPU
#include "arm64_fpu.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_ARCH_FPU

void arm64_fork_fpureg_save(struct fork_s *context)
{
  irqstate_t flags;

  /* Take a snapshot of the thread fpu reg context right now */

  flags = enter_critical_section();

  arm64_fpu_save(context->fpu);
  UP_DSB();

  leave_critical_section(flags);
}

#endif

/****************************************************************************
 * Name: fork
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
 *      transfers control up arm64_fork().
 *   2) arm64_fork() and calls nxtask_setup_fork().
 *   3) nxtask_setup_fork() allocates and configures the child task's TCB.
 *      This consists of:
 *      - Allocation of the child task's TCB.
 *      - Initialization of file descriptors and streams
 *      - Configuration of environment variables
 *      - Allocate and initialize the stack
 *      - Setup the input parameters for the task.
 *      - Initialization of the TCB (including call to up_initial_state())
 *   4) arm64_fork() provides any additional operating context. arm64_fork
 *      must:
 *      - Initialize special values in any CPU registers that were not
 *        already configured by up_initial_state()
 *   5) arm64_fork() then calls nxtask_start_fork()
 *   6) nxtask_start_fork() then executes the child thread.
 *
 * nxtask_abort_fork() may be called if an error occurs between steps 3 and
 * 6.
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

pid_t arm64_fork(const struct fork_s *context)
{
  struct tcb_s *parent = this_task();
  struct task_tcb_s *child;
  uint64_t newsp;
  uint64_t newfp;
  uint64_t newtop;
  uint64_t stacktop;
  uint64_t stackutil;

  /* Allocate and initialize a TCB for the child task. */

  child = nxtask_setup_fork((start_t)(context->lr & ~1));
  if (!child)
    {
      serr("ERROR: nxtask_setup_fork failed\n");
      return (pid_t)ERROR;
    }

  /* How much of the parent's stack was utilized?  The ARM uses
   * a push-down stack so that the current stack pointer should
   * be lower than the initial, adjusted stack pointer.  The
   * stack usage should be the difference between those two.
   */

  stacktop = (uint64_t)parent->stack_base_ptr +
                       parent->adj_stack_size;
  DEBUGASSERT(stacktop > context->sp);
  stackutil = stacktop - context->sp;

  /* Make some feeble effort to preserve the stack contents.  This is
   * feeble because the stack surely contains invalid pointers and other
   * content that will not work in the child context.  However, if the
   * user follows all of the caveats of fork() usage, even this feeble
   * effort is overkill.
   */

  newtop = (uint64_t)child->cmn.stack_base_ptr +
                     child->cmn.adj_stack_size;
  newsp = newtop - stackutil;
  memcpy((void *)newsp, (const void *)context->sp, stackutil);

  /* Was there a frame pointer in place before? */

  if (context->fp >= context->sp && context->fp < stacktop)
    {
      uint64_t frameutil = stacktop - context->fp;
      newfp = newtop - frameutil;
    }
  else
    {
      newfp = context->fp;
    }

  /* Update the stack pointer, frame pointer, and volatile registers.  When
   * the child TCB was initialized, all of the values were set to zero.
   * up_initial_state() altered a few values, but the return value in R0
   * should be cleared to zero, providing the indication to the newly started
   * child thread.
   */

  /* make the fork stack frame */

#ifdef CONFIG_ARCH_FPU
  child->cmn.xcp.fpu_regs = (void *)(newsp - FPU_CONTEXT_SIZE);
  memcpy(child->cmn.xcp.fpu_regs, context->fpu, FPU_CONTEXT_SIZE);
#endif

  child->cmn.xcp.regs             = (void *)(newsp - XCPTCONTEXT_SIZE);

  child->cmn.xcp.regs[REG_X0]     = 0;
  child->cmn.xcp.regs[REG_X8]     = context->regs[FORK_REG_X8];
  child->cmn.xcp.regs[REG_X9]     = context->regs[FORK_REG_X9];
  child->cmn.xcp.regs[REG_X10]    = context->regs[FORK_REG_X10];
  child->cmn.xcp.regs[REG_X11]    = context->regs[FORK_REG_X11];
  child->cmn.xcp.regs[REG_X12]    = context->regs[FORK_REG_X12];
  child->cmn.xcp.regs[REG_X13]    = context->regs[FORK_REG_X13];
  child->cmn.xcp.regs[REG_X14]    = context->regs[FORK_REG_X14];
  child->cmn.xcp.regs[REG_X15]    = context->regs[FORK_REG_X15];
  child->cmn.xcp.regs[REG_X16]    = context->regs[FORK_REG_X16];
  child->cmn.xcp.regs[REG_X17]    = context->regs[FORK_REG_X17];
  child->cmn.xcp.regs[REG_X18]    = context->regs[FORK_REG_X18];
  child->cmn.xcp.regs[REG_X19]    = context->regs[FORK_REG_X19];
  child->cmn.xcp.regs[REG_X20]    = context->regs[FORK_REG_X20];
  child->cmn.xcp.regs[REG_X21]    = context->regs[FORK_REG_X21];
  child->cmn.xcp.regs[REG_X22]    = context->regs[FORK_REG_X22];
  child->cmn.xcp.regs[REG_X23]    = context->regs[FORK_REG_X23];
  child->cmn.xcp.regs[REG_X24]    = context->regs[FORK_REG_X24];
  child->cmn.xcp.regs[REG_X25]    = context->regs[FORK_REG_X25];
  child->cmn.xcp.regs[REG_X26]    = context->regs[FORK_REG_X26];
  child->cmn.xcp.regs[REG_X27]    = context->regs[FORK_REG_X27];
  child->cmn.xcp.regs[REG_X28]    = context->regs[FORK_REG_X28];
  child->cmn.xcp.regs[REG_FP]     = newfp;

#if CONFIG_ARCH_ARM64_EXCEPTION_LEVEL == 3
  child->cmn.xcp.regs[REG_SPSR]   = SPSR_MODE_EL3H;
#else
  child->cmn.xcp.regs[REG_SPSR]   = SPSR_MODE_EL1H;
#endif

#ifdef CONFIG_SUPPRESS_INTERRUPTS
  child->cmn.xcp.regs[REG_SPSR]  |= (DAIF_IRQ_BIT | DAIF_FIQ_BIT);
#endif /* CONFIG_SUPPRESS_INTERRUPTS */

  child->cmn.xcp.regs[REG_ELR]    = (uint64_t)context->lr;

  child->cmn.xcp.regs[REG_EXE_DEPTH] = 0;
  child->cmn.xcp.regs[REG_SP_ELX]    = newsp - XCPTCONTEXT_SIZE;
#ifdef CONFIG_ARCH_KERNEL_STACK
  child->cmn.xcp.regs[REG_SP_EL0]    = (uint64_t)child->cmn.xcp.ustkptr;
#else
  child->cmn.xcp.regs[REG_SP_EL0]    = newsp - XCPTCONTEXT_SIZE;
#endif

  /* And, finally, start the child task.  On a failure, nxtask_start_fork()
   * will discard the TCB by calling nxtask_abort_fork().
   */

  return nxtask_start_fork(child);
}
