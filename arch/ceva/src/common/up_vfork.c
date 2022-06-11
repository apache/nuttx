/****************************************************************************
 * arch/ceva/src/common/up_vfork.c
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

#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>

#include "sched/sched.h"

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
 *   2) up_vfork()and calls nxtask_vforksetup().
 *   3) nxtask_vforksetup() allocates and configures the child task's TCB.
 *      This consists of:
 *      - Allocation of the child task's TCB.
 *      - Initialization of file descriptors and streams
 *      - Configuration of environment variables
 *      - Setup the input parameters for the task.
 *      - Initialization of the TCB (including call to up_initial_state()
 *   4) up_vfork() provides any additional operating context. up_vfork must:
 *      - Allocate and initialize the stack
 *      - Initialize special values in any CPU registers that were not
 *        already configured by up_initial_state()
 *   5) up_vfork() then calls nxtask_vforkstart()
 *   6) nxtask_vforkstart() then executes the child thread.
 *
 * nxtask_vforkabort() may be called if an error occurs between steps 3 & 6.
 *
 * Input Parameters:
 *   regs - Caller context information saved by vfork()
 *
 * Return:
 *   Upon successful completion, vfork() returns 0 to the child process and
 *   returns the process ID of the child process to the parent process.
 *   Otherwise, -1 is returned to the parent, no child process is created,
 *   and errno is set to indicate the error.
 *
 ****************************************************************************/

pid_t up_vfork(const uint32_t *regs)
{
#ifdef CONFIG_SCHED_WAITPID
  struct tcb_s *parent = this_task();
  struct task_tcb_s *child;
  size_t stacksize;
  const void *sp = regs + XCPTCONTEXT_REGS;
  void *newsp;
  uint32_t newfp;
  uint32_t stackutil;
  size_t argsize;
  void *argv;
  int ret;

  /* Allocate and initialize a TCB for the child task. */

  child = nxtask_vforksetup(parent->start, &argsize);
  if (!child)
    {
      serr("ERROR: nxtask_vforksetup failed\n");
      return (pid_t)ERROR;
    }

  sinfo("TCBs: Parent=%p Child=%p\n", parent, child);

  /* Get the size of the parent task's stack. */

  stacksize = parent->adj_stack_size;

  /* Allocate the stack for the TCB */

  ret = up_create_stack((struct tcb_s *)child, stacksize + argsize,
                        parent->flags & TCB_FLAG_TTYPE_MASK);
  if (ret != OK)
    {
      serr("ERROR: up_create_stack failed: %d\n", ret);
      nxtask_vforkabort(child, -ret);
      return (pid_t)ERROR;
    }

  /* Allocate the memory and copy argument from parent task */

  argv = up_stack_frame((struct tcb_s *)child, argsize);
  memcpy(argv, parent->stack_base_ptr, argsize);

  /* How much of the parent's stack was utilized?  The CEVA uses
   * a push-down stack so that the current stack pointer should
   * be lower than the initial, adjusted stack pointer.  The
   * stack usage should be the difference between those two.
   */

  DEBUGASSERT(parent->stack_base_ptr >= sp);
  stackutil = parent->stack_base_ptr - sp;

  sinfo("Parent: stacksize:%d stackutil:%d\n", stacksize, stackutil);

  /* Make some feeble effort to preserve the stack contents.  This is
   * feeble because the stack surely contains invalid pointers and other
   * content that will not work in the child context.  However, if the
   * user follows all of the caveats of vfork() usage, even this feeble
   * effort is overkill.
   */

  newsp = child->cmn.stack_base_ptr - stackutil;
  memcpy(newsp, sp, stackutil);

  /* Allocate the context and copy the parent snapshot */

  newsp -= XCPTCONTEXT_SIZE;
  memcpy(newsp, regs, XCPTCONTEXT_SIZE);
  child->cmn.xcp.regs = newsp;

  /* Was there a frame pointer in place before? */

  if (regs[REG_FP] <= (uint32_t)parent->stack_base_ptr &&
      regs[REG_FP] >= (uint32_t)parent->stack_base_ptr - stacksize)
    {
      uint32_t frameutil = (uint32_t)parent->stack_base_ptr - regs[REG_FP];
      newfp = (uint32_t)child->cmn.stack_base_ptr - frameutil;
    }
  else
    {
      newfp = regs[REG_FP];
    }

  sinfo("Parent: stack base:%08x SP:%08x FP:%08x\n",
        parent->stack_base_ptr, sp, regs[REG_FP]);
  sinfo("Child:  stack base:%08x SP:%08x FP:%08x\n",
        child->cmn.stack_base_ptr, newsp, newfp);

  /* Update the stack pointer, frame pointer, and the return value in A0
   * should be cleared to zero, providing the indication to the newly started
   * child thread.
   */

  child->cmn.xcp.regs[REG_A0] = 0;               /* Return value */
  child->cmn.xcp.regs[REG_FP] = newfp;           /* Frame pointer */
  child->cmn.xcp.regs[REG_PC] = regs[REG_LR];    /* Program counter */
  child->cmn.xcp.regs[REG_SP] = (uint32_t)newsp; /* Stack pointer */

#ifdef CONFIG_LIB_SYSCALL
  /* If we got here via a syscall, then we are going to have to setup some
   * syscall return information as well.
   */

  if (parent->xcp.nsyscalls > 0)
    {
      int index;
      for (index = 0; index < parent->xcp.nsyscalls; index++)
        {
          child->cmn.xcp.syscall[index] = parent->xcp.syscall[index];
        }

      child->cmn.xcp.nsyscalls = parent->xcp.nsyscalls;
    }
#endif

  /* And, finally, start the child task.  On a failure, nxtask_vforkstart()
   * will discard the TCB by calling nxtask_vforkabort().
   */

  return nxtask_vforkstart(child);
#else /* CONFIG_SCHED_WAITPID */
  return (pid_t)ERROR;
#endif
}
