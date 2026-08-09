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
#include <nuttx/debug.h>

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
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm64_fork_stack
 *
 * Description:
 *   Give the child the part of the parent's stack that is in use, copied to
 *   the top of the child's own stack.
 *
 *   The copy is aligned with the top of each stack rather than the bottom,
 *   so a single offset carries any address in the copied region from the
 *   parent's stack to the child's; that offset is what is returned, and
 *   arm64_fork_reloc() applies it.
 *
 * Input Parameters:
 *   parent - The parent task's TCB
 *   child  - The child task's TCB
 *   sp     - The parent's stack pointer where the primitive was called
 *
 * Returned Value:
 *   The offset from an address in the parent's stack to the same place in
 *   the child's copy of it.
 *
 ****************************************************************************/

static uint64_t arm64_fork_stack(struct tcb_s *parent, struct tcb_s *child,
                                 uint64_t sp)
{
  uint64_t stacktop;
  uint64_t stackutil;
  uint64_t newtop;

  /* How much of the parent's stack was utilized?  The ARM uses a push-down
   * stack so that the current stack pointer should be lower than the
   * initial, adjusted stack pointer.  The stack usage should be the
   * difference between those two.
   */

  stacktop  = (uint64_t)parent->stack_base_ptr + parent->adj_stack_size;
  DEBUGASSERT(stacktop > sp);
  stackutil = stacktop - sp;

  /* Make some feeble effort to preserve the stack contents.  This is
   * feeble because the stack surely contains invalid pointers and other
   * content that will not work in the child context.  However, if the
   * user follows all of the caveats of vfork() usage, even this feeble
   * effort is overkill.
   */

  newtop = (uint64_t)child->stack_base_ptr + child->adj_stack_size;
  memcpy((void *)(newtop - stackutil), (const void *)sp, stackutil);

  return newtop - stacktop;
}

/****************************************************************************
 * Name: arm64_fork_reloc
 *
 * Description:
 *   Carry one address from the parent's stack over to the child's copy of
 *   it.  Addresses outside the region that arm64_fork_stack() copied are
 *   returned unchanged:  they point somewhere the child shares with the
 *   parent, or somewhere that has no counterpart at all.
 *
 * Input Parameters:
 *   parent - The parent task's TCB
 *   addr   - The address to relocate
 *   sp     - The parent's stack pointer where the primitive was called,
 *            which is the low end of the region that was copied
 *   offset - The offset returned by arm64_fork_stack()
 *
 * Returned Value:
 *   The relocated address.
 *
 ****************************************************************************/

static uint64_t arm64_fork_reloc(struct tcb_s *parent, uint64_t addr,
                                 uint64_t sp, uint64_t offset)
{
  uint64_t stacktop = (uint64_t)parent->stack_base_ptr +
                                parent->adj_stack_size;

  /* The top of the stack is included:  a stack pointer resting there is one
   * past the last byte copied, and still has to move with it.
   */

  if (addr >= sp && addr <= stacktop)
    {
      return addr + offset;
    }

  return addr;
}

/****************************************************************************
 * Name: arm64_fork_direct
 *
 * Description:
 *   Clone a caller that reached up_fork() by an ordinary function call, so
 *   that the register snapshot taken by arm64_fork_func.S describes the
 *   caller itself.  That is the case in a flat build, and for a kernel
 *   thread in any build.
 *
 *   The child has no exception frame to inherit, so one is synthesised:  it
 *   resumes at the caller's return address, at the same privilege level,
 *   with the callee-saved registers the caller had.
 *
 * Input Parameters:
 *   vfork   - true for vfork(), false for fork()
 *   parent  - The calling task's TCB
 *   context - Caller context information saved by arm64_fork_func.S
 *
 * Returned Value:
 *   The process ID of the child, or ERROR on failure.
 *
 ****************************************************************************/

static pid_t arm64_fork_direct(bool vfork, struct tcb_s *parent,
                               const struct fork_s *context)
{
  struct tcb_s *child;
  uint64_t offset;
  uint64_t newsp;
  uint64_t newfp;

  /* Allocate and initialize a TCB for the child task. */

  child = nxtask_setup_fork((start_t)context->lr, vfork);
  if (!child)
    {
      serr("ERROR: nxtask_setup_fork failed\n");
      return (pid_t)ERROR;
    }

  /* Copy the parent's stack to the child and relocate the pointers into it */

  offset = arm64_fork_stack(parent, child, context->sp);
  newsp  = context->sp + offset;
  newfp  = arm64_fork_reloc(parent, context->fp, context->sp, offset);

  /* Update the stack pointer, frame pointer, and volatile registers.  When
   * the child TCB was initialized, all of the values were set to zero.
   * up_initial_state() altered a few values, but the return value in R0
   * should be cleared to zero, providing the indication to the newly started
   * child thread.
   */

  /* make the fork stack frame */

#ifdef CONFIG_ARCH_FPU
  child->xcp.fpu_regs = (void *)(newsp - FPU_CONTEXT_SIZE);
  memcpy(child->xcp.fpu_regs, context->fpu, FPU_CONTEXT_SIZE);
#endif

  child->xcp.regs             = (void *)(newsp - XCPTCONTEXT_SIZE);

  child->xcp.regs[REG_X0]     = 0;
  child->xcp.regs[REG_X8]     = context->regs[FORK_REG_X8];
  child->xcp.regs[REG_X9]     = context->regs[FORK_REG_X9];
  child->xcp.regs[REG_X10]    = context->regs[FORK_REG_X10];
  child->xcp.regs[REG_X11]    = context->regs[FORK_REG_X11];
  child->xcp.regs[REG_X12]    = context->regs[FORK_REG_X12];
  child->xcp.regs[REG_X13]    = context->regs[FORK_REG_X13];
  child->xcp.regs[REG_X14]    = context->regs[FORK_REG_X14];
  child->xcp.regs[REG_X15]    = context->regs[FORK_REG_X15];
  child->xcp.regs[REG_X16]    = context->regs[FORK_REG_X16];
  child->xcp.regs[REG_X17]    = context->regs[FORK_REG_X17];
  child->xcp.regs[REG_X18]    = context->regs[FORK_REG_X18];
  child->xcp.regs[REG_X19]    = context->regs[FORK_REG_X19];
  child->xcp.regs[REG_X20]    = context->regs[FORK_REG_X20];
  child->xcp.regs[REG_X21]    = context->regs[FORK_REG_X21];
  child->xcp.regs[REG_X22]    = context->regs[FORK_REG_X22];
  child->xcp.regs[REG_X23]    = context->regs[FORK_REG_X23];
  child->xcp.regs[REG_X24]    = context->regs[FORK_REG_X24];
  child->xcp.regs[REG_X25]    = context->regs[FORK_REG_X25];
  child->xcp.regs[REG_X26]    = context->regs[FORK_REG_X26];
  child->xcp.regs[REG_X27]    = context->regs[FORK_REG_X27];
  child->xcp.regs[REG_X28]    = context->regs[FORK_REG_X28];
  child->xcp.regs[REG_FP]     = newfp;

#if CONFIG_ARCH_ARM64_EXCEPTION_LEVEL == 3
  child->xcp.regs[REG_SPSR]   = SPSR_MODE_EL3H;
#else
  child->xcp.regs[REG_SPSR]   = SPSR_MODE_EL1H;
#endif

#ifdef CONFIG_SUPPRESS_INTERRUPTS
  child->xcp.regs[REG_SPSR]  |= (DAIF_IRQ_BIT | DAIF_FIQ_BIT);
#endif /* CONFIG_SUPPRESS_INTERRUPTS */

  child->xcp.regs[REG_ELR]    = (uint64_t)context->lr;

#ifdef CONFIG_ARM64_MTE
  child->xcp.regs[REG_SCTLR_EL1] = read_sysreg(sctlr_el1) | SCTLR_TCF1_BIT;
#endif

  child->xcp.regs[REG_EXE_DEPTH] = 0;
  child->xcp.regs[REG_SP_ELX]    = newsp - XCPTCONTEXT_SIZE;
#ifdef CONFIG_ARCH_KERNEL_STACK
  child->xcp.regs[REG_SP_EL0]    = (uint64_t)child->xcp.ustkptr;
#else
  child->xcp.regs[REG_SP_EL0]    = newsp - XCPTCONTEXT_SIZE;
#endif

  /* And, finally, start the child task.  On a failure, nxtask_start_fork()
   * will discard the TCB by calling nxtask_abort_fork().
   */

  return nxtask_start_fork(child, vfork);
}

#ifdef CONFIG_LIB_SYSCALL

/****************************************************************************
 * Name: arm64_fork_syscall
 *
 * Description:
 *   Clone a caller that reached up_fork() through a system call.  The
 *   register snapshot taken by arm64_fork_func.S is useless here:  it
 *   describes the kernel-side stub, so a child built from it would resume at
 *   a kernel address on a kernel stack.  What the caller was actually doing
 *   is the exception frame the SVC handler recorded in xcp.sregs; the child
 *   is built from that.
 *
 *   The child therefore returns from the very same SVC instruction as the
 *   parent, at the same privilege level, differing only in that it sees 0
 *   as the return value and runs on its own stack.
 *
 * Input Parameters:
 *   vfork  - true for vfork(), false for fork()
 *   parent - The calling task's TCB
 *
 * Returned Value:
 *   The process ID of the child, or ERROR on failure.
 *
 ****************************************************************************/

static pid_t arm64_fork_syscall(bool vfork, struct tcb_s *parent)
{
  uint64_t *sregs = parent->xcp.sregs;
  struct tcb_s *child;
  uint64_t offset;
  uint64_t newsp;
  uint64_t newfp;
  uint64_t regtop;
  uint64_t sp;

  DEBUGASSERT(sregs != NULL);

  /* Which stack the caller was on depends on the level it trapped from:  a
   * user task uses SP_EL0, anything running in the kernel uses SP_ELx.
   */

  if ((sregs[REG_SPSR] & SPSR_MODE_MASK) == SPSR_MODE_EL0T)
    {
      sp = sregs[REG_SP_EL0];
    }
  else
    {
      sp = sregs[REG_SP_ELX];
    }

  /* Allocate and initialize a TCB for the child task.  The child resumes at
   * the instruction after the SVC, which is where the parent resumes too.
   */

  child = nxtask_setup_fork((start_t)sregs[REG_ELR], vfork);
  if (!child)
    {
      serr("ERROR: nxtask_setup_fork failed\n");
      return (pid_t)ERROR;
    }

  /* Copy the parent's stack to the child and relocate the pointers into it */

  offset = arm64_fork_stack(parent, child, sp);
  newsp  = sp + offset;
  newfp  = arm64_fork_reloc(parent, sregs[REG_FP], sp, offset);

  /* Where does the register save area the child is resumed from go?  The
   * parent's is wherever SP_ELx pointed when it trapped, so put the child's
   * at the matching place:  the top of its own kernel stack if it has one --
   * a user process in a kernel build -- or else the same offset into its
   * copy of the parent's stack.
   */

#ifdef CONFIG_ARCH_KERNEL_STACK
  if (child->xcp.kstack)
    {
      regtop = (uint64_t)child->xcp.kstack + ARCH_KERNEL_STACKSIZE;
    }
  else
#endif
    {
      regtop = arm64_fork_reloc(parent, sregs[REG_SP_ELX], sp, offset);
    }

  child->xcp.regs = (void *)(regtop - XCPTCONTEXT_SIZE);

  /* Inherit the parent's whole exception frame, integer and FPU registers
   * alike, then fix up only what has to differ.
   */

  memcpy(child->xcp.regs, sregs, XCPTCONTEXT_SIZE);

#ifdef CONFIG_ARCH_FPU
  child->xcp.fpu_regs = (void *)(regtop - FPU_CONTEXT_SIZE);
#endif

  child->xcp.regs[REG_X0]        = 0;
  child->xcp.regs[REG_FP]        = newfp;
  child->xcp.regs[REG_EXE_DEPTH] = 0;
  child->xcp.regs[REG_SP_ELX]    = regtop - XCPTCONTEXT_SIZE;

  if ((sregs[REG_SPSR] & SPSR_MODE_MASK) == SPSR_MODE_EL0T)
    {
      child->xcp.regs[REG_SP_EL0] = newsp;
#ifdef CONFIG_ARCH_KERNEL_STACK
      child->xcp.ustkptr          = (uintptr_t *)newsp;
#endif
    }

  /* And, finally, start the child task.  On a failure, nxtask_start_fork()
   * will discard the TCB by calling nxtask_abort_fork().
   */

  return nxtask_start_fork(child, vfork);
}

#endif /* CONFIG_LIB_SYSCALL */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_ARCH_FPU

void arm64_fork_fpureg_save(struct fork_s *context)
{
  /* Take a snapshot of the thread fpu reg context right now */

  arm64_fpu_save(context->fpu);
  UP_DSB();
}

#endif

/****************************************************************************
 * Name: arm64_fork
 *
 * Description:
 *   The common ARM64 worker behind up_fork().  vfork() and fork() snapshot
 *   the caller's registers identically; `vfork' says which primitive was
 *   called, and is passed straight through to nxtask_setup_fork(), which is
 *   where the memory semantics are decided.
 *
 *   The overall sequence is:
 *
 *   1) User code calls vfork() or fork().  up_fork() collects context
 *      information and transfers control to arm64_fork().
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
 *   Everything above is common to the two ways this can be reached, which
 *   differ only in where the caller's registers are to be found -- see
 *   arm64_fork_direct() and arm64_fork_syscall().
 *
 * Input Parameters:
 *   vfork   - true for vfork(), false for fork()
 *   context - Caller context information saved by up_fork()
 *
 * Returned Value:
 *   Upon successful completion, fork() returns 0 to the child process and
 *   returns the process ID of the child process to the parent process.
 *   Otherwise, -1 is returned to the parent, no child process is created,
 *   and errno is set to indicate the error.
 *
 ****************************************************************************/

pid_t arm64_fork(bool vfork, const struct fork_s *context)
{
  struct tcb_s *parent = this_task();

#ifdef CONFIG_LIB_SYSCALL
  /* If a system call is in progress then this was reached from its kernel-
   * side stub, and the caller is the user task that trapped, not the code
   * that called into arm64_fork_func.S.
   */

  if ((parent->flags & TCB_FLAG_SYSCALL) != 0)
    {
      return arm64_fork_syscall(vfork, parent);
    }
#endif

  return arm64_fork_direct(vfork, parent, context);
}
