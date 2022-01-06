/****************************************************************************
 * arch/risc-v/src/rv32im/riscv_vfork.c
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

#include "riscv_vfork.h"
#include "sched/sched.h"

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
 *   2) up_vfork() and calls nxtask_setup_vfork().
 *   3) nxtask_setup_vfork() allocates and configures the child task's TCB.
 *     This consists of:
 *      - Allocation of the child task's TCB.
 *      - Initialization of file descriptors and streams
 *      - Configuration of environment variables
 *      - Allocate and initialize the stack
 *      - Setup the input parameters for the task.
 *      - Initialization of the TCB (including call to up_initial_state())
 *   4) up_vfork() provides any additional operating context. up_vfork must:
 *      - Initialize special values in any CPU registers that were not
 *        already configured by up_initial_state()
 *   5) up_vfork() then calls nxtask_start_vfork()
 *   6) nxtask_start_vfork() then executes the child thread.
 *
 * nxtask_abort_vfork() may be called if an error occurs between steps 3
 * and 6.
 *
 * Input Parameters:
 *   context - Caller context information saved by vfork()
 *
 * Returned Value:
 *   Upon successful completion, vfork() returns 0 to the child process and
 *   returns the process ID of the child process to the parent process.
 *   Otherwise, -1 is returned to the parent, no child process is created,
 *   and errno is set to indicate the error.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_HAVE_VFORK

#error This part of the port is not done yet!!

pid_t up_vfork(const struct vfork_s *context)
{
  struct tcb_s *parent = this_task();
  struct task_tcb_s *child;
  uint32_t newsp;
#ifdef CONFIG_RISCV_FRAMEPOINTER
  uint32_t newfp;
#endif
  uint32_t newtop;
  uint32_t stacktop;
  uint32_t stackutil;

  sinfo("s0:%08x s1:%08x s2:%08x s3:%08x s4:%08x\n",
        context->s0, context->s1, context->s2, context->s3, context->s4);
#ifdef CONFIG_RISCV_FRAMEPOINTER
  sinfo("s5:%08x s6:%08x s7:%08x\n",
        context->s5, context->s6, context->s7);
#ifdef RISCV_SAVE_GP
  sinfo("fp:%08x sp:%08x ra:%08x gp:%08x\n",
        context->fp, context->sp, context->ra, context->gp);
#else
  sinfo("fp:%08x sp:%08x ra:%08x\n",
        context->fp context->sp, context->ra);
#endif
#else
  sinfo("s5:%08x s6:%08x s7:%08x s8:%08x\n",
        context->s5, context->s6, context->s7, context->s8);
#ifdef RISCV_SAVE_GP
  sinfo("sp:%08x ra:%08x gp:%08x\n",
        context->sp, context->ra, context->gp);
#else
  sinfo("sp:%08x ra:%08x\n",
        context->sp, context->ra);
#endif
#endif

  /* Allocate and initialize a TCB for the child task. */

  child = nxtask_setup_vfork((start_t)context->ra);
  if (!child)
    {
      sinfo("nxtask_setup_vfork failed\n");
      return (pid_t)ERROR;
    }

  sinfo("Parent=%p Child=%p\n", parent, child);

  /* How much of the parent's stack was utilized?  The RISC-V uses
   * a push-down stack so that the current stack pointer should
   * be lower than the initial, adjusted stack pointer.  The
   * stack usage should be the difference between those two.
   */

  stacktop = (uint32_t)parent->stack_base_ptr +
                       parent->adj_stack_size;
  DEBUGASSERT(stacktop > context->sp);
  stackutil = stacktop - context->sp;

  sinfo("Parent: stackutil:%" PRIu32 "\n", stackutil);

  /* Make some feeble effort to preserve the stack contents.  This is
   * feeble because the stack surely contains invalid pointers and other
   * content that will not work in the child context.  However, if the
   * user follows all of the caveats of vfork() usage, even this feeble
   * effort is overkill.
   */

  newtop = (uint32_t)child->cmn.stack_base_ptr +
                     child->cmn.adj_stack_size;
  newsp = newtop - stackutil;
  memcpy((void *)newsp, (const void *)context->sp, stackutil);

  /* Was there a frame pointer in place before? */

#ifdef CONFIG_RISCV_FRAMEPOINTER
  if (context->fp >= context->sp && context->fp < stacktop)
    {
      uint32_t frameutil = stacktop - context->fp;
      newfp = newtop - frameutil;
    }
  else
    {
      newfp = context->fp;
    }

  sinfo("Old stack top:%08x SP:%08x FP:%08x\n",
        stacktop, context->sp, context->fp);
  sinfo("New stack top:%08x SP:%08x FP:%08x\n",
        newtop, newsp, newfp);
#else
  sinfo("Old stack top:%08x SP:%08x\n",
        stacktop, context->sp);
  sinfo("New stack top:%08x SP:%08x\n",
        newtop, newsp);
#endif

  /* Update the stack pointer, frame pointer, global pointer and saved
   * registers.  When the child TCB was initialized, all of the values
   * were set to zero. up_initial_state() altered a few values, but the
   * return value in v0 should be cleared to zero, providing the
   * indication to the newly started child thread.
   */

  child->cmn.xcp.regs[REG_S0]  = context->s0;  /* Saved register s0 */
  child->cmn.xcp.regs[REG_S1]  = context->s1;  /* Saved register s1 */
  child->cmn.xcp.regs[REG_S2]  = context->s2;  /* Saved register s2 */
  child->cmn.xcp.regs[REG_S3]  = context->s3;  /* Volatile register s3 */
  child->cmn.xcp.regs[REG_S4]  = context->s4;  /* Volatile register s4 */
  child->cmn.xcp.regs[REG_S5]  = context->s5;  /* Volatile register s5 */
  child->cmn.xcp.regs[REG_S6]  = context->s6;  /* Volatile register s6 */
  child->cmn.xcp.regs[REG_S7]  = context->s7;  /* Volatile register s7 */
#ifdef CONFIG_RISCV_FRAMEPOINTER
  child->cmn.xcp.regs[REG_FP]  = newfp;        /* Frame pointer */
#else
  child->cmn.xcp.regs[REG_S8]  = context->s8;  /* Volatile register s8 */
#endif
  child->cmn.xcp.regs[REG_SP]  = newsp;        /* Stack pointer */
#ifdef RISCV_SAVE_GP
  child->cmn.xcp.regs[REG_GP]  = newsp;        /* Global pointer */
#endif

  /* And, finally, start the child task.  On a failure, nxtask_start_vfork()
   * will discard the TCB by calling nxtask_abort_vfork().
   */

  return nxtask_start_vfork(child);
}

#endif /* CONFIG_ARCH_HAVE_VFORK */
