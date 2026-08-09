/****************************************************************************
 * arch/arm/src/common/arm_fork.c
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
#include <arch/irq.h>

#include "arm_fork.h"
#include "arm_internal.h"
#include "sched/sched.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_fork_direct
 *
 * Description:
 *   Clone a caller that reached up_fork() by an ordinary function call, so
 *   that the register snapshot the entry point in fork.S took describes the
 *   caller itself.  That is the case in a flat build, in a protected build
 *   -- where a system call is dispatched on the caller's own stack, so the
 *   caller's frames are copied along with the kernel-side ones and the child
 *   unwinds back through them -- and for a kernel thread in any build.
 *
 * Input Parameters:
 *   vfork   - true for vfork(), false for fork()
 *   parent  - The calling task's TCB
 *   context - Caller context information saved by the entry point
 *
 * Returned Value:
 *   The process ID of the child, or ERROR on failure.
 *
 ****************************************************************************/

static pid_t arm_fork_direct(bool vfork, struct tcb_s *parent,
                             const struct fork_s *context)
{
  struct tcb_s *child;
  uint32_t newsp;
  uint32_t newfp;
  uint32_t newtop;
  uint32_t stacktop;
  uint32_t stackutil;
  uint32_t oldsp = context->sp;

#ifdef CONFIG_ARCH_KERNEL_STACK
  /* A caller that trapped into a system call and was switched onto its
   * kernel stack left its own stack pointer here; the snapshot in `context'
   * is the kernel-side stub's.  Where the whole exception frame is available
   * arm_fork_syscall() has already taken the call, so this is reached only
   * where the caller's frames are copied along with the kernel-side ones.
   */

  if (parent->xcp.ustkptr != NULL)
    {
      oldsp = (uint32_t)parent->xcp.ustkptr;
    }
#endif

  sinfo("fork context [%p]:\n", context);
  sinfo("  r4:%08" PRIx32 " r5:%08" PRIx32
        " r6:%08" PRIx32 " r7:%08" PRIx32 "\n",
        context->r4, context->r5, context->r6, context->r7);
  sinfo("  r8:%08" PRIx32 " r9:%08" PRIx32 " r10:%08" PRIx32 "\n",
        context->r8, context->r9, context->r10);
  sinfo("  r11:%08" PRIx32 " sp:%08" PRIx32 " lr:%08" PRIx32 "\n",
        context->r11, oldsp, context->lr);

  /* Allocate and initialize a TCB for the child task. */

  child = nxtask_setup_fork((start_t)(context->lr & ~1), vfork);
  if (!child)
    {
      serr("ERROR: nxtask_setup_fork failed\n");
      return (pid_t)ERROR;
    }

  sinfo("TCBs: Parent=%p Child=%p\n", parent, child);

  /* How much of the parent's stack was utilized?  The ARM uses
   * a push-down stack so that the current stack pointer should
   * be lower than the initial, adjusted stack pointer.  The
   * stack usage should be the difference between those two.
   */

  stacktop = (uint32_t)parent->stack_base_ptr +
                       parent->adj_stack_size;
  DEBUGASSERT(stacktop > oldsp && oldsp >= (uint32_t)parent->stack_base_ptr);
  stackutil = stacktop - oldsp;

  sinfo("Parent: stackutil:%" PRIu32 "\n", stackutil);

  if (child->stack_base_ptr == parent->stack_base_ptr)
    {
      /* The child is running at the parent's stack addresses, inside its
       * own duplicated address environment.  There is nothing to relocate:
       * every stack address the child inherits is still the address it
       * names.
       */

      newsp = oldsp;
      newfp = context->fp;
    }
  else
    {
      /* Make some feeble effort to preserve the stack contents.  This is
       * feeble because the stack surely contains invalid pointers and other
       * content that will not work in the child context.  However, if the
       * user follows all of the caveats of vfork() usage, even this feeble
       * effort is overkill.
       *
       * For a POSIX fork() child the stack contents are not merely a feeble
       * effort:  the child is entitled to use them, and it does.
       */

      newtop = (uint32_t)child->stack_base_ptr +
                         child->adj_stack_size;

      newsp = newtop - stackutil;

      /* Move the register context to newtop. */

      memcpy((void *)(newsp - XCPTCONTEXT_SIZE),
             child->xcp.regs, XCPTCONTEXT_SIZE);

      child->xcp.regs = (void *)(newsp - XCPTCONTEXT_SIZE);

      memcpy((void *)newsp, (const void *)oldsp, stackutil);

      /* Was there a frame pointer in place before? */

      if (context->fp >= oldsp && context->fp < stacktop)
        {
          uint32_t frameutil = stacktop - context->fp;
          newfp = newtop - frameutil;
        }
      else
        {
          newfp = context->fp;
        }

      sinfo("Old stack top:%08" PRIx32 " SP:%08" PRIx32
            " FP:%08" PRIx32 "\n", stacktop, oldsp, context->fp);
      sinfo("New stack top:%08" PRIx32 " SP:%08" PRIx32
            " FP:%08" PRIx32 "\n", newtop, newsp, newfp);
    }

  /* Update the stack pointer, frame pointer, and volatile registers.  When
   * the child TCB was initialized, all of the values were set to zero.
   * up_initial_state() altered a few values, but the return value in R0
   * should be cleared to zero, providing the indication to the newly started
   * child thread.
   */

  child->xcp.regs[REG_R4]  = context->r4;  /* Volatile register r4 */
  child->xcp.regs[REG_R5]  = context->r5;  /* Volatile register r5 */
  child->xcp.regs[REG_R6]  = context->r6;  /* Volatile register r6 */
  child->xcp.regs[REG_R7]  = context->r7;  /* Volatile register r7 */
  child->xcp.regs[REG_R8]  = context->r8;  /* Volatile register r8 */
  child->xcp.regs[REG_R9]  = context->r9;  /* Volatile register r9 */
  child->xcp.regs[REG_R10] = context->r10; /* Volatile register r10 */
  child->xcp.regs[REG_R11] = context->r11; /* Volatile register r11 */
  child->xcp.regs[REG_FP]  = newfp;        /* Frame pointer */
  child->xcp.regs[REG_SP]  = newsp;        /* Stack pointer */

#ifdef CONFIG_LIB_SYSCALL
  /* If we got here via a syscall, then we are going to have to setup some
   * syscall return information as well.
   */

  if (parent->xcp.nsyscalls > 0)
    {
      int index;
      for (index = 0; index < parent->xcp.nsyscalls; index++)
        {
          child->xcp.syscall[index].sysreturn =
            parent->xcp.syscall[index].sysreturn;

          /* REVISIT:  This logic is *not* common. */

#if defined(CONFIG_ARCH_ARMV7A)
#  ifdef CONFIG_BUILD_KERNEL

          child->xcp.syscall[index].cpsr =
            parent->xcp.syscall[index].cpsr;

#  endif

#elif defined(CONFIG_ARCH_ARMV7R)
#  ifdef CONFIG_BUILD_PROTECTED

          child->xcp.syscall[index].cpsr =
            parent->xcp.syscall[index].cpsr;

#  endif
#elif defined(CONFIG_ARCH_ARMV6M) || defined(CONFIG_ARCH_ARMV7M) || \
      defined(CONFIG_ARCH_ARMV8M)

          child->xcp.syscall[index].excreturn =
            parent->xcp.syscall[index].excreturn;

          /* CONTROL too:  SYS_syscall_return restores it from ctrlreturn,
           * and the child's zeroed TCB would return it to user space with
           * CONTROL == 0 -- nPRIV clear, i.e. privileged.
           */

          child->xcp.syscall[index].ctrlreturn =
            parent->xcp.syscall[index].ctrlreturn;
#else
#  error Missing logic
#endif
        }

      child->xcp.nsyscalls = parent->xcp.nsyscalls;
    }
#endif

  /* And, finally, start the child task.  A vfork() additionally suspends us
   * until the child calls _exit() or exec().
   */

  return nxtask_start_fork(child, vfork);
}

#if defined(CONFIG_ARCH_ARMV7A) && defined(CONFIG_ARCH_KERNEL_STACK)

/****************************************************************************
 * Name: arm_fork_syscall
 *
 * Description:
 *   Clone a caller that reached up_fork() through a system call that was
 *   switched onto a kernel stack.  The register snapshot fork.S took is
 *   useless here:  it describes the kernel-side stub, and the frames below
 *   it are on a stack the child does not get a copy of, so a child built
 *   from it would resume at a kernel address with a stack pointer into its
 *   own user stack.
 *
 *   What the caller was actually doing is the exception frame arm_vectorsvc
 *   built on the caller's stack and arm_syscall() recorded in xcp.sregs; the
 *   child is built from that.  It therefore returns from the very same SVC
 *   instruction as the parent, in the same mode, differing only in that it
 *   sees 0 as the return value and runs on its own stack.  The child is not
 *   in a system call at all, so it inherits none of the parent's nesting
 *   state.
 *
 * Input Parameters:
 *   vfork  - true for vfork(), false for fork()
 *   parent - The calling task's TCB
 *
 * Returned Value:
 *   The process ID of the child, or ERROR on failure.
 *
 ****************************************************************************/

static pid_t arm_fork_syscall(bool vfork, struct tcb_s *parent)
{
  uint32_t *sregs = parent->xcp.sregs;
  struct tcb_s *child;
  uint32_t newsp;
  uint32_t newfp;
  uint32_t newtop;
  uint32_t stacktop;
  uint32_t stackutil;
  uint32_t oldsp = (uint32_t)parent->xcp.ustkptr;
  uint32_t sysreturn;
  uint32_t cpsr;

  DEBUGASSERT(sregs != NULL && parent->xcp.nsyscalls > 0);

  /* Where the caller resumes, and in which mode.  arm_syscall() re-pointed
   * the frame at dispatch_syscall() before this was reached, so these two
   * come from where it put the originals rather than from the frame.  In a
   * protected build there is no mode change, so the frame still holds the
   * caller's CPSR.
   */

  sysreturn = parent->xcp.syscall[0].sysreturn;
#ifdef CONFIG_BUILD_KERNEL
  cpsr      = parent->xcp.syscall[0].cpsr;
#else
  cpsr      = sregs[REG_CPSR];
#endif

  /* Allocate and initialize a TCB for the child task. */

  child = nxtask_setup_fork((start_t)(sysreturn & ~1), vfork);
  if (!child)
    {
      serr("ERROR: nxtask_setup_fork failed\n");
      return (pid_t)ERROR;
    }

  stacktop = (uint32_t)parent->stack_base_ptr +
                       parent->adj_stack_size;
  DEBUGASSERT(stacktop > oldsp && oldsp >= (uint32_t)parent->stack_base_ptr);
  stackutil = stacktop - oldsp;

  if (child->stack_base_ptr == parent->stack_base_ptr)
    {
      /* The child is running at the parent's stack addresses, inside its
       * own duplicated address environment.  There is nothing to relocate;
       * see the same case in arm_fork_direct().
       */

      newsp = oldsp;
      newfp = sregs[REG_FP];
    }
  else
    {
      newtop = (uint32_t)child->stack_base_ptr +
                         child->adj_stack_size;
      newsp  = newtop - stackutil;

      /* Put the child's register save area where the parent's is:  just
       * below the stack the caller was using.  It cannot be left at the top
       * of the child's stack, which is where up_initial_state() put it,
       * because the copy of the parent's stack below is about to land there.
       */

      child->xcp.regs = (uint32_t *)(newsp - XCPTCONTEXT_SIZE);

      memcpy((void *)newsp, (const void *)oldsp, stackutil);

      /* Was there a frame pointer in place before? */

      if (sregs[REG_FP] >= oldsp && sregs[REG_FP] < stacktop)
        {
          uint32_t frameutil = stacktop - sregs[REG_FP];
          newfp = newtop - frameutil;
        }
      else
        {
          newfp = sregs[REG_FP];
        }
    }

  /* Inherit the caller's whole exception frame, integer and floating point
   * registers alike, then fix up only what has to differ:  the child sees 0
   * as the return value and runs on its own stack.
   */

  memcpy(child->xcp.regs, sregs, XCPTCONTEXT_SIZE);

  child->xcp.regs[REG_R0]   = 0;
  child->xcp.regs[REG_FP]   = newfp;
  child->xcp.regs[REG_SP]   = newsp;
  child->xcp.regs[REG_PC]   = sysreturn;
  child->xcp.regs[REG_CPSR] = cpsr;

  /* And, finally, start the child task.  A vfork() additionally suspends us
   * until the child calls _exit() or exec().
   */

  return nxtask_start_fork(child, vfork);
}

#endif /* CONFIG_ARCH_ARMV7A && CONFIG_ARCH_KERNEL_STACK */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_fork
 *
 * Description:
 *   The common ARM worker behind up_fork().  vfork() and fork() snapshot
 *   the caller's registers identically; `vfork' says which primitive was
 *   called, and is passed straight through to nxtask_setup_fork(), which is
 *   where the memory semantics are decided.
 *
 *   What differs here is only the stack.  Normally the child has a stack of
 *   its own, and this function fills it with a relocated copy of the
 *   parent's, rebasing the stack and frame pointers to match.  When the
 *   child shares the parent's stack addresses -- a fork() child, inside its
 *   duplicated address environment -- there is nothing to relocate and the
 *   pointers are carried over unchanged.
 *
 *   The overall sequence is:
 *
 *   1) User code calls vfork() or fork().  The libc wrapper enters
 *      up_fork(), which collects context information and transfers control
 *      to arm_fork().
 *   2) arm_fork() calls nxtask_setup_fork().
 *   3) nxtask_setup_fork() allocates and configures the child task's TCB.
 *      This consists of:
 *      - Allocation of the child task's TCB.
 *      - Initialization of file descriptors and streams
 *      - Configuration of environment variables
 *      - Establishing the child's address environment:  joined to the
 *        parent's for vfork(), duplicated from it for fork()
 *      - Allocating the stack, or inheriting the parent's for fork()
 *      - Setup the input parameters for the task.
 *      - Initialization of the TCB (including call to up_initial_state())
 *   4) arm_fork() provides any additional operating context. arm_fork must:
 *      - Initialize special values in any CPU registers that were not
 *        already configured by up_initial_state()
 *   5) arm_fork() then calls nxtask_start_fork(), which for vfork()
 *      additionally suspends the caller.
 *   6) which executes the child thread.
 *
 * nxtask_abort_fork() may be called if an error occurs between steps 3 and
 * 6.
 *
 *   Everything above is common to the two ways this can be reached, which
 *   differ only in where the caller's registers are to be found -- see
 *   arm_fork_direct() and arm_fork_syscall().
 *
 * Input Parameters:
 *   vfork   - true for vfork(), false for fork()
 *   context - Caller context information saved by the entry point
 *
 * Returned Value:
 *   Upon successful completion, 0 is returned to the child and the process
 *   ID of the child is returned to the parent.  Otherwise, -1 is returned to
 *   the parent, no child is created, and errno is set to indicate the error.
 *
 ****************************************************************************/

pid_t arm_fork(bool vfork, const struct fork_s *context)
{
  struct tcb_s *parent = this_task();

#if defined(CONFIG_ARCH_ARMV7A) && defined(CONFIG_ARCH_KERNEL_STACK)
  /* A saved user stack pointer means this was reached through a system call
   * that switched to the task's kernel stack, so the caller is the user task
   * that trapped and not the code that called into fork.S.  A kernel thread
   * has no kernel stack to switch to and never gets here with one saved.
   */

  if (parent->xcp.ustkptr != NULL)
    {
      DEBUGASSERT((parent->flags & TCB_FLAG_SYSCALL) != 0);
      return arm_fork_syscall(vfork, parent);
    }
#endif

  return arm_fork_direct(vfork, parent, context);
}
