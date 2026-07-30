/****************************************************************************
 * arch/x86/src/common/x86_fork.c
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

#include "x86_fork.h"
#include "x86_internal.h"
#include "sched/sched.h"

/* This architecture gives the child a relocated copy of the parent's stack;
 * a borrowed stack would need that relocation to be skipped.
 */

#ifdef CONFIG_ARCH_VFORK_STACK_BORROW
#  error "x86 relocates the vfork() child stack; borrowing is not supported"
#endif

/* And it has no address environment, so there is no POSIX fork() here.  The
 * assembly half provides no up_fork() entry point either.
 */

#ifdef CONFIG_ARCH_HAVE_FORK
#  error "x86 has no address environment; fork() cannot be provided"
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: x86_fork_stacktop
 *
 * Description:
 *   The high end of a task's stack, and the point the copy made by a fork is
 *   aligned to.
 *
 *   Unlike x86_64, this architecture keeps the register save area inside the
 *   TCB -- xcp.regs is an array, not a pointer into the stack -- so all the
 *   stack down from the top belongs to the task and nothing has to be held
 *   back.  It is also the value up_initial_state() puts in REG_SP, and
 *   up_stack_frame() only ever takes from the bottom, so the top does not
 *   move for the rest of the task's life.
 *
 * Input Parameters:
 *   tcb - The task whose stack is in question
 *
 * Returned Value:
 *   The address one past the last byte of the task's stack.
 *
 ****************************************************************************/

static uint32_t x86_fork_stacktop(struct tcb_s *tcb)
{
  return (uint32_t)tcb->stack_base_ptr + tcb->adj_stack_size;
}

/****************************************************************************
 * Name: x86_fork_reloc
 *
 * Description:
 *   Carry one address from the parent's stack over to the child's copy of
 *   it.  Addresses outside the copied region are returned unchanged:  they
 *   point somewhere the child shares with the parent, or somewhere that has
 *   no counterpart at all.
 *
 * Input Parameters:
 *   addr     - The address to relocate
 *   esp      - The parent's stack pointer where the primitive was called,
 *              which is the low end of the region that was copied
 *   stacktop - The high end of the region that was copied
 *   offset   - The distance from the parent's stack to the child's copy
 *
 * Returned Value:
 *   The relocated address.
 *
 ****************************************************************************/

static uint32_t x86_fork_reloc(uint32_t addr, uint32_t esp,
                               uint32_t stacktop, uint32_t offset)
{
  if (addr >= esp && addr < stacktop)
    {
      return addr + offset;
    }

  return addr;
}

/****************************************************************************
 * Name: x86_fork_relocfp
 *
 * Description:
 *   Relocate the saved frame-pointer chain inside the child's copy of the
 *   parent's stack.
 *
 *   This is not optional on either x86.  A function returns here with
 *   `leave', which is `mov %ebp,%esp' followed by `pop %ebp':  the frame
 *   pointer feeds the *stack* pointer.  Relocating only the EBP the child
 *   resumes with therefore gets it exactly one frame; the moment it returns
 *   through the next one it loads a saved EBP that still points into the
 *   parent's stack, and from then on the child runs on the parent's stack.
 *   It looks like the child is working -- it is even at the right offset --
 *   until something returns through a slot the parent has since reused.
 *
 *   The architectures that return through a link register do not need this:
 *   there a stale frame pointer spoils a backtrace and nothing else.
 *
 *   The walk stops at the first link that leaves the copied region -- the
 *   outermost frame's saved EBP does -- and refuses to move backwards, so a
 *   corrupt chain terminates it rather than looping.
 *
 * Input Parameters:
 *   ebp      - The parent's frame pointer where the primitive was called
 *   esp      - The parent's stack pointer, the low end of the copied region
 *   stacktop - The high end of the copied region
 *   offset   - The distance from the parent's stack to the child's copy
 *
 ****************************************************************************/

static void x86_fork_relocfp(uint32_t ebp, uint32_t esp,
                             uint32_t stacktop, uint32_t offset)
{
  while (ebp >= esp && ebp < stacktop)
    {
      uint32_t *slot = (uint32_t *)(ebp + offset);
      uint32_t  next = *slot;

      if (next <= ebp || next >= stacktop)
        {
          break;
        }

      *slot = next + offset;
      ebp   = next;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: x86_fork
 *
 * Description:
 *   The architecture-specific half of NuttX's cloning primitives.  Which one
 *   is being performed is given by `vfork'.  This architecture provides only
 *   vfork() -- POSIX fork() needs an address environment to duplicate and
 *   there is none here -- but the flag is carried all the same, because the
 *   generic half takes it.
 *
 *   The child is given a stack of its own holding a relocated copy of the
 *   part of the parent's that is in use.  The caller is always cloned as it
 *   stood at the call, because there is only one way to get here:  an
 *   ordinary function call into fork.S.  A flat build has no system-call
 *   boundary for the caller to be on the far side of, so there is no second
 *   path of the kind x86_64 needs.
 *
 *   The overall sequence is:
 *
 *   1) User code calls vfork().  up_fork() in fork.S collects context
 *      information and transfers control to x86_fork().
 *   2) x86_fork() calls nxtask_setup_fork().
 *   3) nxtask_setup_fork() allocates and configures the child task's TCB.
 *      This consists of:
 *      - Allocation of the child task's TCB.
 *      - Initialization of file descriptors and streams
 *      - Configuration of environment variables
 *      - Allocate and initialize the stack
 *      - Setup the input parameters for the task.
 *      - Initialization of the TCB (including call to up_initial_state())
 *   4) x86_fork() provides any additional operating context.  It must:
 *      - Initialize special values in any CPU registers that were not
 *        already configured by up_initial_state()
 *   5) x86_fork() then calls nxtask_start_fork()
 *   6) nxtask_start_fork() then executes the child thread.
 *
 *   nxtask_abort_fork() may be called if an error occurs between steps 3
 *   and 6.
 *
 * Input Parameters:
 *   context - Caller context information saved by fork.S
 *   vfork   - true for vfork(), false for fork()
 *
 * Returned Value:
 *   Upon successful completion, 0 is returned to the child and the process
 *   ID of the child is returned to the caller.  Otherwise, -1 is returned to
 *   the caller, no child is created, and errno is set to indicate the error.
 *
 ****************************************************************************/

pid_t x86_fork(const struct fork_s *context, bool vfork)
{
  struct tcb_s *parent = this_task();
  struct tcb_s *child;
  uint32_t newsp;
  uint32_t newtop;
  uint32_t offset;
  uint32_t stacktop;
  uint32_t stackutil;

  sinfo("fork context [%p]:\n", context);
  sinfo("  ebx:%08" PRIx32 " esi:%08" PRIx32 " edi:%08" PRIx32 "\n",
        context->ebx, context->esi, context->edi);
  sinfo("  esp:%08" PRIx32 " ebp:%08" PRIx32 " eip:%08" PRIx32 "\n",
        context->esp, context->ebp, context->eip);

  /* Allocate and initialize a TCB for the child task.  The child resumes at
   * the instruction the caller would have returned to.
   */

  child = nxtask_setup_fork((start_t)context->eip, vfork);
  if (!child)
    {
      serr("ERROR: nxtask_setup_fork failed\n");
      return (pid_t)ERROR;
    }

  sinfo("TCBs: Parent=%p Child=%p\n", parent, child);

  /* How much of the parent's stack was utilized?  x86 uses a push-down
   * stack, so the caller's stack pointer is below the top of its stack and
   * the difference between the two is what is in use.
   */

  stacktop = x86_fork_stacktop(parent);
  DEBUGASSERT(stacktop > context->esp);
  stackutil = stacktop - context->esp;

  /* Give the child that part of the parent's stack, copied to the same place
   * in its own.  The copy is aligned with the top of each stack rather than
   * the bottom, so a single offset carries any address in the copied region
   * from one to the other.
   *
   * This is a feeble effort in the sense arm_fork.c uses the word:  the
   * copied bytes surely contain pointers to things that do not exist in the
   * child.  What is corrected below is the links -- the frame chain -- and
   * not the data.  A caller that observes the caveats of these primitives
   * never reads any of it.
   */

  newtop = x86_fork_stacktop(child);
  newsp  = newtop - stackutil;
  offset = newtop - stacktop;

  memcpy((void *)newsp, (const void *)context->esp, stackutil);

  sinfo("Parent: stackutil:%" PRIu32 " stack top:%08" PRIx32
        " esp:%08" PRIx32 "\n", stackutil, stacktop, context->esp);
  sinfo("Child:  stack top:%08" PRIx32 " esp:%08" PRIx32 "\n",
        newtop, newsp);

  /* Build the register context the child is resumed from.  It was zeroed by
   * up_initial_state(), which also filled in the entry point, the segments
   * and the flags; what is left is what the child inherits from the caller.
   *
   * x86_fullcontextrestore() resumes it with an IRET off REG_SP, so REG_SP,
   * REG_EIP, REG_CS and REG_EFLAGS are the frame it builds, and REG_ESP --
   * the copy of the stack pointer that PUSHA leaves -- is not read.
   */

  child->xcp.regs[REG_EAX]    = 0;              /* Child returns 0 */
  child->xcp.regs[REG_EBX]    = context->ebx;   /* Callee-saved */
  child->xcp.regs[REG_ESI]    = context->esi;   /* Callee-saved */
  child->xcp.regs[REG_EDI]    = context->edi;   /* Callee-saved */
  child->xcp.regs[REG_DS]     = context->ds;
  child->xcp.regs[REG_CS]     = context->cs;
  child->xcp.regs[REG_SS]     = context->ss;
  child->xcp.regs[REG_EFLAGS] = context->eflags;
  child->xcp.regs[REG_EIP]    = context->eip;
  child->xcp.regs[REG_SP]     = newsp;

  /* The frame pointer, and the chain of saved frame pointers it heads, move
   * with the stack they point into.
   */

  child->xcp.regs[REG_EBP] = x86_fork_reloc(context->ebp, context->esp,
                                            stacktop, offset);
  x86_fork_relocfp(context->ebp, context->esp, stacktop, offset);

  /* And, finally, start the child task.  On a failure, nxtask_start_fork()
   * will discard the TCB by calling nxtask_abort_fork().
   */

  return nxtask_start_fork(child, vfork);
}
