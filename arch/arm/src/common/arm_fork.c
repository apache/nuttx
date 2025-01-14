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
#include <debug.h>

#include <nuttx/sched.h>
#include <nuttx/arch.h>
#include <arch/irq.h>

#include "arm_fork.h"
#include "arm_internal.h"
#include "sched/sched.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_fork
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
 *      transfers control up arm_fork().
 *   2) arm_fork() and calls nxtask_setup_fork().
 *   3) nxtask_setup_fork() allocates and configures the child task's TCB.
 *      This consists of:
 *      - Allocation of the child task's TCB.
 *      - Initialization of file descriptors and streams
 *      - Configuration of environment variables
 *      - Allocate and initialize the stack
 *      - Setup the input parameters for the task.
 *      - Initialization of the TCB (including call to up_initial_state())
 *   4) arm_fork() provides any additional operating context. arm_fork must:
 *      - Initialize special values in any CPU registers that were not
 *        already configured by up_initial_state()
 *   5) arm_fork() then calls nxtask_start_fork()
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

pid_t arm_fork(const struct fork_s *context)
{
  struct tcb_s *parent = this_task();
  struct task_tcb_s *child;
  uint32_t newsp;
  uint32_t newfp;
  uint32_t newtop;
  uint32_t stacktop;
  uint32_t stackutil;
#ifdef CONFIG_ARCH_KERNEL_STACK
  uint32_t oldsp = (uint32_t)parent->xcp.ustkptr;
#else
  uint32_t oldsp = context->sp;
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

  child = nxtask_setup_fork((start_t)(context->lr & ~1));
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

  /* Make some feeble effort to preserve the stack contents.  This is
   * feeble because the stack surely contains invalid pointers and other
   * content that will not work in the child context.  However, if the
   * user follows all of the caveats of fork() usage, even this feeble
   * effort is overkill.
   */

  newtop = (uint32_t)child->cmn.stack_base_ptr +
                     child->cmn.adj_stack_size;

  newsp = newtop - stackutil;

  /* Move the register context to newtop. */

  memcpy((void *)(newsp - XCPTCONTEXT_SIZE),
         child->cmn.xcp.regs, XCPTCONTEXT_SIZE);

  child->cmn.xcp.regs = (void *)(newsp - XCPTCONTEXT_SIZE);

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

  sinfo("Old stack top:%08" PRIx32 " SP:%08" PRIx32 " FP:%08" PRIx32 "\n",
        stacktop, oldsp, context->fp);
  sinfo("New stack top:%08" PRIx32 " SP:%08" PRIx32 " FP:%08" PRIx32 "\n",
        newtop, newsp, newfp);

  /* Update the stack pointer, frame pointer, and volatile registers.  When
   * the child TCB was initialized, all of the values were set to zero.
   * up_initial_state() altered a few values, but the return value in R0
   * should be cleared to zero, providing the indication to the newly started
   * child thread.
   */

  child->cmn.xcp.regs[REG_R4]  = context->r4;  /* Volatile register r4 */
  child->cmn.xcp.regs[REG_R5]  = context->r5;  /* Volatile register r5 */
  child->cmn.xcp.regs[REG_R6]  = context->r6;  /* Volatile register r6 */
  child->cmn.xcp.regs[REG_R7]  = context->r7;  /* Volatile register r7 */
  child->cmn.xcp.regs[REG_R8]  = context->r8;  /* Volatile register r8 */
  child->cmn.xcp.regs[REG_R9]  = context->r9;  /* Volatile register r9 */
  child->cmn.xcp.regs[REG_R10] = context->r10; /* Volatile register r10 */
  child->cmn.xcp.regs[REG_R11] = context->r11; /* Volatile register r11 */
  child->cmn.xcp.regs[REG_FP]  = newfp;        /* Frame pointer */
  child->cmn.xcp.regs[REG_SP]  = newsp;        /* Stack pointer */

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

          /* REVISIT:  This logic is *not* common. */

#if defined(CONFIG_ARCH_ARMV7A)
#  ifdef CONFIG_BUILD_KERNEL

          child->cmn.xcp.syscall[index].cpsr =
            parent->xcp.syscall[index].cpsr;

#  endif

#elif defined(CONFIG_ARCH_ARMV7R)
#  ifdef CONFIG_BUILD_PROTECTED

          child->cmn.xcp.syscall[index].cpsr =
            parent->xcp.syscall[index].cpsr;

#  endif
#elif defined(CONFIG_ARCH_ARMV6M) || defined(CONFIG_ARCH_ARMV7M) || \
      defined(CONFIG_ARCH_ARMV8M)

          child->cmn.xcp.syscall[index].excreturn =
            parent->xcp.syscall[index].excreturn;
#else
#  error Missing logic
#endif
        }

      child->cmn.xcp.nsyscalls = parent->xcp.nsyscalls;
    }
#endif

  /* And, finally, start the child task.  On a failure, nxtask_start_fork()
   * will discard the TCB by calling nxtask_abort_fork().
   */

  return nxtask_start_fork(child);
}
