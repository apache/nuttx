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
#include <debug.h>

#include <nuttx/sched.h>
#include <nuttx/arch.h>
#include <arch/irq.h>

#include "x86_64_fork.h"
#include "x86_64_internal.h"
#include "sched/sched.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: x86_64_fork
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
 *      transfers control up x86_64_fork().
 *   2) x86_64_fork() and calls nxtask_setup_fork().
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

pid_t x86_64_fork(const struct fork_s *context)
{
  struct tcb_s *parent = this_task();
  struct task_tcb_s *child;
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

  child = nxtask_setup_fork((start_t)context->rip);
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

  stacktop = (uint64_t)XCP_ALIGN_DOWN((uintptr_t)parent->stack_base_ptr +
                                      parent->adj_stack_size -
                                      XCPTCONTEXT_SIZE);
  DEBUGASSERT(stacktop > context->rsp);
  stackutil = stacktop - context->rsp;

  sinfo("Parent: stackutil:%" PRIu64 "\n", stackutil);

  /* Make some feeble effort to preserve the stack contents.  This is
   * feeble because the stack surely contains invalid pointers and other
   * content that will not work in the child context.  However, if the
   * user follows all of the caveats of fork() usage, even this feeble
   * effort is overkill.
   */

  newtop = (uint64_t)XCP_ALIGN_DOWN((uintptr_t)child->cmn.stack_base_ptr +
                                    child->cmn.adj_stack_size -
                                    XCPTCONTEXT_SIZE);

  newsp = newtop - stackutil;

  /* Move the register context (from parent) to newtop. */

  memcpy(child->cmn.xcp.regs, parent->xcp.regs, XCPTCONTEXT_SIZE);

  memcpy((void *)newsp, (const void *)context->rsp, stackutil);

  /* Was there a frame pointer in place before? */

  if (context->rbp >= context->rsp && context->rbp < stacktop)
    {
      uint32_t frameutil = stacktop - context->rbp;
      newfp = newtop - frameutil;
    }
  else
    {
      newfp = context->rbp;
    }

  /* We do not need to update the frame-pointer */

  sinfo("Old stack top:%08" PRIx64 " RSP:%08" PRIx64 " RBP:%08" PRIx64 "\n",
        stacktop, context->rsp, context->rbp);
  sinfo("New stack top:%08" PRIx64 " RSP:%08" PRIx64 "\n",
        newtop, newsp);

  /* Update the stack pointer, frame pointer, and volatile registers.  When
   * the child TCB was initialized, all of the values were set to zero.
   * up_initial_state() altered a few values, but the return value in RAX
   * should be cleared to zero, providing the indication to the newly started
   * child thread.
   */

  child->cmn.xcp.regs[REG_RAX]    = 0;            /* Parent proc return 0 */
  child->cmn.xcp.regs[REG_R12]    = context->r12; /* Non-volatile register r12 */
  child->cmn.xcp.regs[REG_R13]    = context->r13; /* Non-volatile register r13 */
  child->cmn.xcp.regs[REG_R14]    = context->r14; /* Non-volatile register r14 */
  child->cmn.xcp.regs[REG_R15]    = context->r15; /* Non-volatile register r15 */
  child->cmn.xcp.regs[REG_RBX]    = context->rbx; /* Non-volatile register rbx */
  child->cmn.xcp.regs[REG_SS]     = context->ss;  /* SS */
  child->cmn.xcp.regs[REG_CS]     = context->cs;  /* CS */
  child->cmn.xcp.regs[REG_RFLAGS] = context->rflags;
  child->cmn.xcp.regs[REG_RIP]    = context->rip;
  child->cmn.xcp.regs[REG_RSP]    = newsp; /* Stack pointer */
  child->cmn.xcp.regs[REG_RBP]    = newfp; /* Like registers */

  /* And, finally, start the child task.  On a failure, nxtask_start_fork()
   * will discard the TCB by calling nxtask_abort_fork().
   */

  return nxtask_start_fork(child);
}
