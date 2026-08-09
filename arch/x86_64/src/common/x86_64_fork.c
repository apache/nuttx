/****************************************************************************
 * arch/x86_64/src/common/x86_64_fork.c
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
#include <arch/arch.h>
#include <arch/irq.h>

#include "x86_64_fork.h"
#include "x86_64_internal.h"
#include "sched/sched.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_LIB_SYSCALL

/* Requested privilege level 3 in a segment selector.  A caller that reached
 * here through the `syscall' instruction was in user mode, and SYSRETQ is
 * going to put it back there with the selectors IA32_STAR describes -- see
 * x86_64_fork_syscall().
 */

#  define X86_GDT_RPL_USER 3

#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: x86_64_fork_stacktop
 *
 * Description:
 *   The high end of the part of a task's stack that a fork copies:  the
 *   bottom of the register save area that up_initial_state() reserved at the
 *   very top of the stack.  The save area itself is not stack and must not
 *   be copied over -- it is where the child's own resume frame is built.
 *
 * Input Parameters:
 *   tcb - The task whose stack is in question
 *
 * Returned Value:
 *   The address one past the last byte of stack that is copied.
 *
 ****************************************************************************/

static uint64_t x86_64_fork_stacktop(struct tcb_s *tcb)
{
  return (uint64_t)XCP_ALIGN_DOWN((uintptr_t)tcb->stack_base_ptr +
                                  tcb->adj_stack_size - XCPTCONTEXT_SIZE);
}

/****************************************************************************
 * Name: x86_64_fork_reloc
 *
 * Description:
 *   Carry one address from the parent's stack over to the child's copy of
 *   it.  Addresses outside the copied region are returned unchanged:  they
 *   point somewhere the child shares with the parent, or somewhere that has
 *   no counterpart at all.
 *
 * Input Parameters:
 *   addr     - The address to relocate
 *   rsp      - The parent's stack pointer where the primitive was called,
 *              which is the low end of the region that was copied
 *   stacktop - The high end of the region that was copied
 *   offset   - The distance from the parent's stack to the child's copy
 *
 * Returned Value:
 *   The relocated address.
 *
 ****************************************************************************/

static uint64_t x86_64_fork_reloc(uint64_t addr, uint64_t rsp,
                                  uint64_t stacktop, uint64_t offset)
{
  if (addr >= rsp && addr < stacktop)
    {
      return addr + offset;
    }

  return addr;
}

/****************************************************************************
 * Name: x86_64_fork_relocfp
 *
 * Description:
 *   Relocate the saved frame-pointer chain inside the child's copy of the
 *   parent's stack.
 *
 *   This is x86_64-specific and it is not optional.  A function returns here
 *   with `leave', which is `mov %rbp,%rsp' followed by `pop %rbp':  the
 *   frame pointer feeds the *stack* pointer.  Relocating only the RBP the
 *   child resumes with therefore gets it exactly one frame; the moment it
 *   returns through the next one it loads a saved RBP that still points
 *   into the parent's stack, and from then on the child runs on the
 *   parent's stack.  It looks like the child is working -- it is even at the
 *   right offset -- until something returns through a slot the parent has
 *   since reused.
 *
 *   The other architectures with this fork path do not need it:  they return
 *   through a link register, so a stale frame pointer spoils a backtrace and
 *   nothing else.
 *
 *   The walk stops at the first link that leaves the copied region -- the
 *   outermost frame's saved RBP does -- and refuses to move backwards, so a
 *   corrupt chain terminates it rather than looping.
 *
 * Input Parameters:
 *   rbp      - The parent's frame pointer where the primitive was called
 *   rsp      - The parent's stack pointer, the low end of the copied region
 *   stacktop - The high end of the copied region
 *   offset   - The distance from the parent's stack to the child's copy
 *
 ****************************************************************************/

static void x86_64_fork_relocfp(uint64_t rbp, uint64_t rsp,
                                uint64_t stacktop, uint64_t offset)
{
  while (rbp >= rsp && rbp < stacktop)
    {
      uint64_t *slot = (uint64_t *)(rbp + offset);
      uint64_t  next = *slot;

      if (next <= rbp || next >= stacktop)
        {
          break;
        }

      *slot = next + offset;
      rbp   = next;
    }
}

/****************************************************************************
 * Name: x86_64_fork_direct
 *
 * Description:
 *   Clone a caller that reached up_fork() by an ordinary function call, so
 *   that the register snapshot fork.S took describes the caller itself.
 *   That is the case in a flat build, and for a kernel thread in any build.
 *
 *   The child has no exception frame to inherit, so one is synthesised:  it
 *   resumes at the caller's return address, in the caller's own segments,
 *   with the callee-saved registers the caller had.
 *
 * Input Parameters:
 *   vfork   - true for vfork(), false for fork()
 *   parent  - The calling task's TCB
 *   context - Caller context information saved by fork.S
 *
 * Returned Value:
 *   The process ID of the child, or ERROR on failure.
 *
 ****************************************************************************/

static pid_t x86_64_fork_direct(bool vfork, struct tcb_s *parent,
                                const struct fork_s *context)
{
  struct tcb_s *child;
  uint64_t newsp;
  uint64_t newfp;
  uint64_t newtop;
  uint64_t stacktop;
  uint64_t stackutil;

  sinfo("fork context [%p]:\n", context);
  sinfo(" rbx:%08" PRIx64 " rbp:%08" PRIx64 "\n"
        " r12:%08" PRIx64 " r13:%08" PRIx64 "\n",
        context->rbx, context->rbp, context->r12, context->r13);
  sinfo(" r14:%08" PRIx64 " r15:%08" PRIx64 "\n",
        context->r14, context->r15);
  sinfo(" sp:%08" PRIx64 " ret ip:%08" PRIx64 "\n",
        context->rsp, context->rip);

  /* Allocate and initialize a TCB for the child task. */

  child = nxtask_setup_fork((start_t)context->rip, vfork);
  if (!child)
    {
      serr("ERROR: nxtask_setup_fork failed\n");
      return (pid_t)ERROR;
    }

  sinfo("TCBs: Parent=%p Child=%p\n", parent, child);

  /* How much of the parent's stack was utilized?  x86_64 uses a push-down
   * stack so that the current stack pointer should be lower than the
   * initial, adjusted stack pointer.  The stack usage should be the
   * difference between those two.
   */

  stacktop = x86_64_fork_stacktop(parent);
  DEBUGASSERT(stacktop > context->rsp);
  stackutil = stacktop - context->rsp;

  sinfo("Parent: stackutil:%" PRIu64 "\n", stackutil);

  /* Move the register context (from parent) to the child. */

  memcpy(child->xcp.regs, parent->xcp.regs, XCPTCONTEXT_SIZE);

  if (child->stack_base_ptr == parent->stack_base_ptr)
    {
      /* The child is running at the parent's stack addresses, inside its
       * own duplicated address environment.  There is nothing to relocate:
       * every stack address the child inherits is still the address it
       * names.
       */

      newsp = context->rsp;
      newfp = context->rbp;
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

      newtop = x86_64_fork_stacktop(child);
      newsp  = newtop - stackutil;

      memcpy((void *)newsp, (const void *)context->rsp, stackutil);

      /* Was there a frame pointer in place before? */

      newfp = x86_64_fork_reloc(context->rbp, context->rsp, stacktop,
                                newtop - stacktop);
      x86_64_fork_relocfp(context->rbp, context->rsp, stacktop,
                          newtop - stacktop);

      sinfo("Old stack top:%08" PRIx64 " RSP:%08" PRIx64
            " RBP:%08" PRIx64 "\n", stacktop, context->rsp, context->rbp);
      sinfo("New stack top:%08" PRIx64 " RSP:%08" PRIx64 "\n",
            newtop, newsp);
    }

  /* Update the stack pointer, frame pointer, and volatile registers.  When
   * the child TCB was initialized, all of the values were set to zero.
   * up_initial_state() altered a few values, but the return value in RAX
   * should be cleared to zero, providing the indication to the newly started
   * child thread.
   */

  child->xcp.regs[REG_RAX]    = 0;            /* Parent proc return 0 */
  child->xcp.regs[REG_R12]    = context->r12; /* Non-volatile register r12 */
  child->xcp.regs[REG_R13]    = context->r13; /* Non-volatile register r13 */
  child->xcp.regs[REG_R14]    = context->r14; /* Non-volatile register r14 */
  child->xcp.regs[REG_R15]    = context->r15; /* Non-volatile register r15 */
  child->xcp.regs[REG_RBX]    = context->rbx; /* Non-volatile register rbx */
  child->xcp.regs[REG_SS]     = context->ss;  /* SS */
  child->xcp.regs[REG_CS]     = context->cs;  /* CS */
  child->xcp.regs[REG_RFLAGS] = context->rflags;
  child->xcp.regs[REG_RIP]    = context->rip;
  child->xcp.regs[REG_RSP]    = newsp; /* Stack pointer */
  child->xcp.regs[REG_RBP]    = newfp; /* Like registers */

  /* And, finally, start the child task.  On a failure, nxtask_start_fork()
   * will discard the TCB by calling nxtask_abort_fork().
   */

  return nxtask_start_fork(child, vfork);
}

#ifdef CONFIG_LIB_SYSCALL

/****************************************************************************
 * Name: x86_64_fork_syscall
 *
 * Description:
 *   Clone a caller that reached up_fork() through a system call.  The
 *   register snapshot fork.S took is useless here:  it describes the
 *   kernel-side stub, so a child built from it would resume at a kernel
 *   address on a kernel stack.  What the caller was actually doing is the
 *   frame x86_64_syscall_entry() saved and x86_64_syscall() recorded in
 *   xcp.sregs; the child is built from that.
 *
 *   The child therefore returns from the very same `syscall' instruction as
 *   the parent, in user mode, differing only in that it sees 0 as the return
 *   value and runs on its own stack.
 *
 *   Two details of the SYSCALL/SYSRET pair shape this:
 *
 *   1. `syscall' does not save the caller's RIP and RFLAGS on a stack; it
 *      leaves them in RCX and R11, which is where the saved frame has them.
 *      The child is resumed by IRETQ (x86_64_fullcontextrestore()), so they
 *      have to be moved into the RIP and RFLAGS slots of its frame.
 *   2. The hardware never tells the kernel which CS and SS the caller had --
 *      SYSRETQ reconstructs them from IA32_STAR -- so those slots of the
 *      saved frame hold nothing, and the child's have to be filled with the
 *      selectors SYSRETQ would have produced, which is where the parent is
 *      about to return to.
 *
 *   Everything the frame does hold -- the general registers and the extended
 *   (FPU/SSE) state -- is inherited.  Everything it does not is taken from
 *   the frame up_initial_state() built for the child, so that the child
 *   keeps its own segment registers and, importantly, its own thread
 *   pointer: the child's stack is a fresh allocation at a different virtual
 *   address, so the parent's FS base does not describe it.
 *
 * Input Parameters:
 *   vfork  - true for vfork(), false for fork()
 *   parent - The calling task's TCB
 *
 * Returned Value:
 *   The process ID of the child, or ERROR on failure.
 *
 ****************************************************************************/

static pid_t x86_64_fork_syscall(bool vfork, struct tcb_s *parent)
{
  uint64_t     *sregs = parent->xcp.sregs;
  struct tcb_s *child;
  uint64_t      newsp;
  uint64_t      newtop;
  uint64_t      offset;
  uint64_t      stacktop;
  uint64_t      stackutil;
  uint64_t      rsp;
  uint64_t      rip;

  DEBUGASSERT(sregs != NULL);

  /* Where the caller was and what it was doing */

  rsp = sregs[REG_RSP];
  rip = sregs[REG_RCX];

  sinfo("syscall frame [%p]: RSP:%08" PRIx64 " RIP:%08" PRIx64 "\n",
        sregs, rsp, rip);

  /* Allocate and initialize a TCB for the child task.  The child resumes at
   * the instruction after the `syscall', which is where the parent resumes
   * too.
   */

  child = nxtask_setup_fork((start_t)rip, vfork);
  if (!child)
    {
      serr("ERROR: nxtask_setup_fork failed\n");
      return (pid_t)ERROR;
    }

  sinfo("TCBs: Parent=%p Child=%p\n", parent, child);

  /* Give the child the part of the parent's stack that is in use, copied to
   * the same place in its own stack.  The copy is aligned with the top of
   * each stack rather than the bottom, so a single offset carries any
   * address in the copied region from one to the other.
   */

  stacktop  = x86_64_fork_stacktop(parent);
  DEBUGASSERT(stacktop > rsp);
  stackutil = stacktop - rsp;

  newtop = x86_64_fork_stacktop(child);
  newsp  = newtop - stackutil;
  offset = newtop - stacktop;

  memcpy((void *)newsp, (const void *)rsp, stackutil);

  sinfo("Old stack top:%08" PRIx64 " RSP:%08" PRIx64 "\n", stacktop, rsp);
  sinfo("New stack top:%08" PRIx64 " RSP:%08" PRIx64 "\n", newtop, newsp);

  /* Inherit the parent's extended (FPU/SSE) state, which
   * x86_64_syscall_entry saved at the front of the frame, exactly where
   * the child's belongs.
   */

  memcpy(child->xcp.regs, sregs, XCPTCONTEXT_XMM_AREA_SIZE);

  /* Inherit the general registers.  RCX and R11 are included deliberately:
   * SYSRETQ leaves the return address in RCX and RFLAGS in R11, so the
   * parent resumes with those values and the child must too.
   */

  child->xcp.regs[REG_RBX] = sregs[REG_RBX];
  child->xcp.regs[REG_R8]  = sregs[REG_R8];
  child->xcp.regs[REG_R9]  = sregs[REG_R9];
  child->xcp.regs[REG_R10] = sregs[REG_R10];
  child->xcp.regs[REG_R11] = sregs[REG_R11];
  child->xcp.regs[REG_R12] = sregs[REG_R12];
  child->xcp.regs[REG_R13] = sregs[REG_R13];
  child->xcp.regs[REG_R14] = sregs[REG_R14];
  child->xcp.regs[REG_R15] = sregs[REG_R15];
  child->xcp.regs[REG_RCX] = sregs[REG_RCX];
  child->xcp.regs[REG_RDX] = sregs[REG_RDX];
  child->xcp.regs[REG_RSI] = sregs[REG_RSI];
  child->xcp.regs[REG_RDI] = sregs[REG_RDI];

  /* The frame pointer moves with the stack it points into */

  child->xcp.regs[REG_RBP] = x86_64_fork_reloc(sregs[REG_RBP], rsp,
                                               stacktop, offset);
  x86_64_fork_relocfp(sregs[REG_RBP], rsp, stacktop, offset);

  /* Build the interrupt frame the child is resumed from.  RIP and RFLAGS
   * come out of RCX and R11, and the selectors are the ones SYSRETQ derives
   * from IA32_STAR:  CS is the user code segment and SS the user data
   * segment, both at RPL 3.  See x86_64_cpu_priv_set(), which programs
   * IA32_STAR.
   */

  child->xcp.regs[REG_RAX]    = 0;
  child->xcp.regs[REG_RIP]    = rip;
  child->xcp.regs[REG_RFLAGS] = sregs[REG_R11];
  child->xcp.regs[REG_RSP]    = newsp;
  child->xcp.regs[REG_CS]     = X86_GDT_USERCODE_SEL | X86_GDT_RPL_USER;
  child->xcp.regs[REG_SS]     = X86_GDT_USERDATA_SEL | X86_GDT_RPL_USER;

#ifdef CONFIG_ARCH_KERNEL_STACK
  /* The child's own user stack pointer, for the signal dispatch path */

  child->xcp.ustkptr = (uintptr_t *)newsp;
#endif

  /* And, finally, start the child task.  On a failure, nxtask_start_fork()
   * will discard the TCB by calling nxtask_abort_fork().
   */

  return nxtask_start_fork(child, vfork);
}

#endif /* CONFIG_LIB_SYSCALL */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: x86_64_fork
 *
 * Description:
 *   The common x86_64 worker behind up_fork().  vfork() and fork() snapshot
 *   the caller's registers identically; `vfork' says which primitive was
 *   called, and is passed straight through to nxtask_setup_fork(), which is
 *   where the memory semantics are decided.
 *
 *   The overall sequence is:
 *
 *   1) User code calls vfork() or fork().  The libc wrapper enters
 *      up_fork(), which collects context information and transfers control
 *      to x86_64_fork().
 *   2) x86_64_fork() calls nxtask_setup_fork().
 *   3) nxtask_setup_fork() allocates and configures the child task's TCB.
 *      This consists of:
 *      - Allocation of the child task's TCB.
 *      - Initialization of file descriptors and streams
 *      - Configuration of environment variables
 *      - Allocate and initialize the stack
 *      - Setup the input parameters for the task.
 *      - Initialization of the TCB (including call to up_initial_state())
 *   4) x86_64_fork() provides any additional operating context. It must:
 *      - Initialize special values in any CPU registers that were not
 *        already configured by up_initial_state()
 *   5) x86_64_fork() then calls nxtask_start_fork()
 *   6) nxtask_start_fork() then executes the child thread.
 *
 * nxtask_abort_fork() may be called if an error occurs between steps 3 and
 * 6.
 *
 *   Everything above is common to the two ways this can be reached, which
 *   differ only in where the caller's registers are to be found -- see
 *   x86_64_fork_direct() and x86_64_fork_syscall().
 *
 * Input Parameters:
 *   vfork   - true for vfork(), false for fork()
 *   context - Caller context information saved by fork.S
 *
 * Returned Value:
 *   Upon successful completion, 0 is returned to the child and the process
 *   ID of the child is returned to the parent.  Otherwise, -1 is returned to
 *   the parent, no child is created, and errno is set to indicate the error.
 *
 ****************************************************************************/

pid_t x86_64_fork(bool vfork, const struct fork_s *context)
{
  struct tcb_s *parent = this_task();

#ifdef CONFIG_LIB_SYSCALL
  /* A non-NULL xcp.sregs means a system call is in progress:
   * x86_64_syscall() publishes the caller's frame there for the duration of
   * the call and nowhere else.  So this was reached from a kernel-side stub,
   * and the caller to clone is the user task that trapped, not the code that
   * called into fork.S.
   *
   * arm64 and RISC-V discriminate on TCB_FLAG_SYSCALL instead.  x86_64
   * cannot:  that flag also defers signal actions, which x86_64 has never
   * done and which its kernel-build signal path does not currently survive
   * -- see the note in x86_64_syscall().  xcp.sregs says exactly what is
   * needed here and means nothing to anyone else.
   */

  if (parent->xcp.sregs != NULL)
    {
      return x86_64_fork_syscall(vfork, parent);
    }
#endif

  return x86_64_fork_direct(vfork, parent, context);
}
