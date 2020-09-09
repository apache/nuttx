/****************************************************************************
 * arch/arm/src/sam34/sam4cm_cpuidlestack.c
 *
 *   Copyright (C) 2016 Masayuki Ishikawa. All rights reserved.
 *   Author: Masayuki Ishikawa <masayuki.ishikawa@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>

#include <nuttx/arch.h>
#include <nuttx/sched.h>

#include "arm_internal.h"

#ifdef CONFIG_SMP

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_idle
 *
 * Description:
 *   up_idle() is the logic that will be executed when there is no other
 *   ready-to-run task.  This is processor idle time and will continue until
 *   some interrupt occurs to cause a context switch from the idle task.
 *
 *   Processing in this state may be processor-specific. e.g., this is where
 *   power management operations might be performed.
 *
 ****************************************************************************/

void up_idle(void)
{
#if defined(CONFIG_SUPPRESS_INTERRUPTS) || defined(CONFIG_SUPPRESS_TIMER_INTS)
  /* If the system is idle and there are no timer interrupts, then process
   * "fake" timer interrupts. Hopefully, something will wake up.
   */

  nxsched_process_timer();
#else

  /* Sleep until an interrupt occurs to save power */

  asm("WFI");

#endif
}

/****************************************************************************
 * Name: up_cpu_idlestack
 *
 * Description:
 *   Allocate a stack for the CPU[n] IDLE task (n > 0) if appropriate and
 *   setup up stack-related information in the IDLE task's TCB.  This
 *   function is always called before up_cpu_start().  This function is
 *   only called for the CPU's initial IDLE task; up_create_task is used for
 *   all normal tasks, pthreads, and kernel threads for all CPUs.
 *
 *   The initial IDLE task is a special case because the CPUs can be started
 *   in different wans in different environments:
 *
 *   1. The CPU may already have been started and waiting in a low power
 *      state for up_cpu_start().  In this case, the IDLE thread's stack
 *      has already been allocated and is already in use.  Here
 *      up_cpu_idlestack() only has to provide information about the
 *      already allocated stack.
 *
 *   2. The CPU may be disabled but started when up_cpu_start() is called.
 *      In this case, a new stack will need to be created for the IDLE
 *      thread and this function is then equivalent to:
 *
 *      return up_create_stack(tcb, stack_size, TCB_FLAG_TTYPE_KERNEL);
 *
 *   The following TCB fields must be initialized by this function:
 *
 *   - adj_stack_size: Stack size after adjustment for hardware, processor,
 *     etc.  This value is retained only for debug purposes.
 *   - stack_alloc_ptr: Pointer to allocated stack
 *   - adj_stack_ptr: Adjusted stack_alloc_ptr for HW.  The initial value of
 *     the stack pointer.
 *
 * Input Parameters:
 *   - cpu:         CPU index that indicates which CPU the IDLE task is
 *                  being created for.
 *   - tcb:         The TCB of new CPU IDLE task
 *   - stack_size:  The requested stack size for the IDLE task.  At least
 *                  this much must be allocated.  This should be
 *                  CONFIG_SMP_STACK_SIZE.
 *
 ****************************************************************************/

int up_cpu_idlestack(int cpu, FAR struct tcb_s *tcb, size_t stack_size)
{
#if CONFIG_SMP_NCPUS > 1
  up_create_stack(tcb, stack_size, TCB_FLAG_TTYPE_KERNEL);
#endif
  return OK;
}

#endif /* CONFIG_SMP */
