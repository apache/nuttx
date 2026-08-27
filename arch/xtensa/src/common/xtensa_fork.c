/****************************************************************************
 * arch/xtensa/src/common/xtensa_fork.c
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

#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <arch/syscall.h>

#include <nuttx/arch.h>
#include <nuttx/compiler.h>
#include <nuttx/sched.h>

#include "sched/sched.h"
#include "xtensa.h"
#include "chip_macros.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The base save area:  the 16 bytes below a frame's stack pointer, holding
 * the spilled a0-a3 of that frame's caller.  Window overflow writes them and
 * underflow reads them back, so it is the link that makes the frame chain
 * walkable, and the reason a copy starting at the stack pointer is missing
 * its first link.
 */

#define BASE_SAVE_AREA   16
#define BASE_SAVE_A1     1   /* a0, a1, a2, a3 -- a1 is the caller's SP */

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Everything a child is built from:  the register context of the thread that
 * called, and where that thread was.  There are two ways to come by it, and
 * they differ in more than provenance -- see xtensa_fork_direct() below.
 */

struct fork_snapshot_s
{
  FAR const uint32_t *regs;  /* The caller's full register context */
  uintptr_t           usp;   /* The caller's user stack pointer */
  uintptr_t           pc;    /* Where the child resumes */
  uint32_t            a2;    /* What the child sees returned in A2 */
#ifndef CONFIG_BUILD_FLAT
  uintptr_t           ctx;   /* The caller's privilege, from its syscall */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: xtensa_fork_rebase
 *
 * Description:
 *   Rebase the frame chain of a relocated stack copy.
 *
 *   A windowed ABI stores each frame's caller stack pointer absolutely, in
 *   the base save area at [sp - 16), so a copy taken at a different address
 *   still names the parent and the child's first retw would underflow onto
 *   the parent's stack.  Each link gets the relocation offset added.
 *
 *   Spilled a4-a15 and any other stack addresses in the copy are left alone.
 *   Those are data;  the chain is what the child needs to return at all.
 *
 * Input Parameters:
 *   newsp    - The child's stack pointer
 *   usp      - The parent's stack pointer, which newsp is a relocation of
 *   stacktop - The top of the parent's stack; the walk ends there
 *   offset   - newsp - usp, the amount every link moves by
 *
 ****************************************************************************/

static void xtensa_fork_rebase(uintptr_t newsp, uintptr_t usp,
                               uintptr_t stacktop, intptr_t offset)
{
  uintptr_t csp = newsp;   /* The frame being fixed, in the child's copy */
  uintptr_t psp = usp;     /* The same frame, as the parent addresses it */

  while (psp < stacktop)
    {
      FAR uint32_t *save = (FAR uint32_t *)(csp - BASE_SAVE_AREA);
      uintptr_t caller = save[BASE_SAVE_A1];

      /* The chain grows towards the top of the stack and ends there.  Stop
       * on anything else rather than following it:  the outermost frame's
       * save area was never written by an overflow, so what is in it is
       * whatever the stack was coloured with.
       */

      if (caller <= psp || caller > stacktop)
        {
          break;
        }

      save[BASE_SAVE_A1] = (uint32_t)(uintptr_t)((intptr_t)caller + offset);

      psp = caller;
      csp = (uintptr_t)((intptr_t)caller + offset);
    }
}

/****************************************************************************
 * Name: xtensa_fork_stack
 *
 * Description:
 *   Give the child its stack pointer, copying the parent's frames if needed.
 *
 *   A fork() child keeps the parent's stack addresses inside its own address
 *   environment, so it has nothing to copy.  A vfork() child gets a stack of
 *   its own, which needs the copy and xtensa_fork_rebase() with it.
 *
 *   The copy starts one base save area below the stack pointer:  the frame
 *   the child resumes into keeps its caller's spilled a0-a3 there.
 *
 * Input Parameters:
 *   parent - The parent task's TCB
 *   child  - The child task's TCB
 *   usp    - The parent's stack pointer
 *
 * Returned Value:
 *   The child's stack pointer.
 *
 ****************************************************************************/

static uintptr_t xtensa_fork_stack(FAR struct tcb_s *parent,
                                   FAR struct tcb_s *child,
                                   uintptr_t usp)
{
  uintptr_t stacktop;
  uintptr_t stackutil;
  uintptr_t newtop;
  uintptr_t newsp;

  stacktop = (uintptr_t)parent->stack_base_ptr + parent->adj_stack_size;
  DEBUGASSERT(stacktop > usp);

  if (child->stack_base_ptr == parent->stack_base_ptr)
    {
      /* The child is running at the parent's stack addresses, inside its
       * own duplicated address environment.  There is nothing to relocate:
       * every stack address the child inherits is still the address it
       * names.
       */

      return usp;
    }

  DEBUGASSERT(usp - BASE_SAVE_AREA >= (uintptr_t)parent->stack_base_ptr);

  stackutil = stacktop - (usp - BASE_SAVE_AREA);
  newtop    = (uintptr_t)child->stack_base_ptr + child->adj_stack_size;

  /* The copy has to fit, and the register save area goes below it when the
   * child has no kernel stack to put it on -- see xtensa_fork().
   */

  DEBUGASSERT(newtop - stackutil >
              (uintptr_t)child->stack_base_ptr + XCPTCONTEXT_SIZE);

  newsp = newtop - stackutil + BASE_SAVE_AREA;

  memcpy((FAR void *)(newsp - BASE_SAVE_AREA),
         (FAR const void *)(usp - BASE_SAVE_AREA), stackutil);

  xtensa_fork_rebase(newsp, usp, stacktop, (intptr_t)(newsp - usp));

  return newsp;
}

/****************************************************************************
 * Name: xtensa_fork
 *
 * Description:
 *   The common core of the two primitives.  They differ in the flag handed
 *   to nxtask_setup_fork(), which is where the memory semantics are decided,
 *   and in where their snapshot of the caller comes from.
 *
 * Input Parameters:
 *   vfork - true for vfork(), false for fork()
 *   snap  - The caller's context; see struct fork_snapshot_s
 *
 * Returned Value:
 *   The pid of the child, or ERROR on failure.
 *
 ****************************************************************************/

static pid_t xtensa_fork(bool vfork, FAR const struct fork_snapshot_s *snap)
{
  FAR struct tcb_s *parent = this_task();
  FAR struct tcb_s *child;
  uintptr_t newsp;
  uintptr_t regstop;

  DEBUGASSERT(snap->regs != NULL && snap->pc != 0);

  /* Allocate and initialise a TCB for the child.  The start address is only
   * bookkeeping here: what the child actually resumes with is the register
   * context assembled below.
   */

  child = nxtask_setup_fork((start_t)snap->pc, vfork);
  if (child == NULL)
    {
      serr("ERROR: nxtask_setup_fork failed\n");
      return (pid_t)ERROR;
    }

  newsp = xtensa_fork_stack(parent, child, snap->usp);

  /* Where the child's register context is restored from. */

#ifdef CONFIG_ARCH_KERNEL_STACK
  if (child->xcp.kstack != NULL)
    {
      /* On the kernel stack:  the child is resumed by the same path a system
       * call returns through, which runs in kernel context.
       */

      regstop = (uintptr_t)child->xcp.ktopstk;
    }
  else
#endif
    {
      /* There is no kernel stack, so it goes on the child's own stack, below
       * the base save area rather than at the stack pointer.  Writing
       * XCPTCONTEXT_SIZE bytes down from newsp would destroy the very words
       * the child's first `retw' reads.  It is dead memory once the context
       * has been restored, so the child may then grow over it.
       *
       * This is only sound because a child without a kernel stack always has
       * a stack of its own:  writing here on a shared stack would land in
       * the parent's frames.  See xtensa_fork_stack().
       */

      DEBUGASSERT(child->stack_base_ptr != parent->stack_base_ptr);
      regstop = newsp - BASE_SAVE_AREA;
    }

  child->xcp.regs = (FAR uint32_t *)(regstop - XCPTCONTEXT_SIZE);

  /* Start from the parent's context, then correct what must differ */

  memcpy(child->xcp.regs, snap->regs, XCPTCONTEXT_SIZE);

  /* The child is not returning the way the parent will:  it is being
   * started.  Give it directly what its resume path would have produced --
   * the address to resume at, its privilege, its own stack, and the value
   * the call returns to it.
   *
   * The privilege matters most.  The frame copied above carries the world
   * the *exception* left in it, not the caller's; taking it would resume an
   * unprivileged process privileged.
   */

  child->xcp.regs[REG_PC] = snap->pc;
  child->xcp.regs[REG_A1] = newsp;
  child->xcp.regs[REG_A2] = snap->a2;

#ifndef CONFIG_BUILD_FLAT
  xtensa_restoreprivilege(child->xcp.regs, snap->ctx);
#endif

#ifdef CONFIG_ARCH_KERNEL_STACK
  /* The parent is inside a system call, so its saved user stack pointer is
   * held aside in ustkptr and A1 names the kernel stack.  The child is not
   * inside that call and must not inherit it.
   */

  child->xcp.ustkptr = NULL;
#endif

  /* And start the child.  On failure nxtask_start_fork() discards the TCB
   * through nxtask_abort_fork().
   */

  return nxtask_start_fork(child, vfork);
}

#ifdef CONFIG_LIB_SYSCALL
/****************************************************************************
 * Name: xtensa_fork_syscall
 *
 * Description:
 *   Fork from the system call that brought the caller into the kernel.
 *
 *   There is no assembly counterpart to this, and it does not need one.
 *   Other architectures enter through a stub that spills the caller's
 *   registers into a struct fork_s, because a C function cannot see its
 *   caller's callee-saved registers.  Here the system call has already done
 *   better than that:  _xtensa_context_save() runs SPILL_ALL_WINDOWS on
 *   every exception entry, so every live register window of the calling
 *   thread has been written to its stack and the whole context is in the
 *   exception frame.  Copying the stack therefore copies a complete and
 *   self-consistent frame chain.  xcp.sregs is that frame (see
 *   xtensa_swint()).
 *
 *   The child resumes where the system call would have returned, and A2 is
 *   the value the call yields -- 0, as fork() and vfork() return to a child.
 *
 ****************************************************************************/

static pid_t xtensa_fork_syscall(bool vfork)
{
  FAR struct tcb_s *parent = this_task();
  struct fork_snapshot_s snap;
  int index;

  /* This runs as the body of a system call, so the caller's own state is not
   * simply what is in the exception frame.
   */

  DEBUGASSERT(parent->xcp.sregs != NULL);
  index = parent->xcp.nsyscalls - 1;
  DEBUGASSERT(index >= 0);

  snap.regs = parent->xcp.sregs;
  snap.pc   = parent->xcp.syscall[index].sysreturn;
  snap.a2   = 0;
#ifndef CONFIG_BUILD_FLAT
  snap.ctx  = parent->xcp.syscall[index].int_ctx;
#endif

  /* The stack pointer to work from is the *user* one.  A kernel build moves
   * the outermost system call onto the thread's kernel stack and holds the
   * user stack pointer aside in ustkptr, so the A1 in the exception frame
   * names the kernel stack from here on -- measuring the parent's user stack
   * against it would produce a nonsense length.
   */

#ifdef CONFIG_ARCH_KERNEL_STACK
  snap.usp = parent->xcp.ustkptr != NULL ?
             (uintptr_t)parent->xcp.ustkptr : snap.regs[REG_A1];
#else
  snap.usp = snap.regs[REG_A1];
#endif

  return xtensa_fork(vfork, &snap);
}

#else /* CONFIG_LIB_SYSCALL */

/****************************************************************************
 * Name: xtensa_fork_direct
 *
 * Description:
 *   Fork a caller that did not arrive through a system call, which in a flat
 *   build is every caller.
 *
 *   SYS_save_context spills every window and copies out the exception frame,
 *   giving the same snapshot a system call would have left behind.
 *
 *   It is issued inline rather than through up_saveusercontext() because the
 *   snapshot records the stack pointer of the frame that issues it, and the
 *   child resumes on a copy of the stack from that point up.  Everything
 *   called from here runs below this frame, so the copied region stays
 *   valid;  a helper's frame would be dead and reused by the time the child
 *   ran on it.
 *
 *   The child resumes after the syscall rather than at a syscall return, so
 *   the PC comes from the frame, and A2 is 1 so the branch below can tell
 *   parent from child.
 *
 * Returned Value:
 *   The pid of the child to the parent, 0 to the child, ERROR on failure.
 *
 ****************************************************************************/

static pid_t xtensa_fork_direct(bool vfork)
{
  uint32_t regs[XCPTCONTEXT_REGS] aligned_data(16);
  struct fork_snapshot_s snap;

  if (sys_call1(SYS_save_context, (uintptr_t)regs) != 0)
    {
      /* The child, resumed from the context captured just above with A2 set
       * to 1.  It has nothing to do but leave.
       */

      return 0;
    }

  snap.regs = regs;
  snap.usp  = regs[REG_A1];
  snap.pc   = regs[REG_PC];
  snap.a2   = 1;

  return xtensa_fork(vfork, &snap);
}
#endif /* CONFIG_LIB_SYSCALL */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_fork
 *
 * Description:
 *   The architecture half of fork() and vfork().  With vfork true the child
 *   shares the parent's memory and runs on a private copy of its stack, and
 *   the parent is suspended until the child leaves.  With vfork false the
 *   child receives its own copy of the parent's memory at the same virtual
 *   addresses.
 *
 * Input Parameters:
 *   vfork - true for vfork(), false for fork()
 *
 * Returned Value:
 *   Upon successful completion, 0 is returned to the child and the pid of
 *   the child to the parent.  Otherwise, -1 is returned to the parent, no
 *   child is created, and errno is set to indicate the error.
 *
 ****************************************************************************/

pid_t up_fork(bool vfork)
{
#ifdef CONFIG_LIB_SYSCALL
  return xtensa_fork_syscall(vfork);
#else
  return xtensa_fork_direct(vfork);
#endif
}
