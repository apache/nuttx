/****************************************************************************
 * arch/risc-v/src/common/riscv_cpuidlestack.c
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

#include <sys/types.h>
#include <assert.h>

#include <nuttx/arch.h>
#include <nuttx/compiler.h>
#include <nuttx/sched.h>

#include "riscv_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SMP_STACK_MASK       15
#define SMP_STACK_SIZE       (CONFIG_IDLETHREAD_STACKSIZE & ~15)
#define STACK_ISALIGNED(a)   ((uintptr_t)(a) & ~SMP_STACK_MASK)

/****************************************************************************
 * Private Data
 ****************************************************************************/

/**
 * Note:
 *   1. QEMU-RV supports up to 8 cores currently.
 *   2. RISC-V requires a 16-byte stack alignment.
 */

#if CONFIG_SMP_NCPUS > 1
static uint8_t aligned_data(16) cpu1_idlestack[CONFIG_IDLETHREAD_STACKSIZE];
#endif

#if CONFIG_SMP_NCPUS > 2
static uint8_t aligned_data(16) cpu2_idlestack[CONFIG_IDLETHREAD_STACKSIZE];
#endif

#if CONFIG_SMP_NCPUS > 3
static uint8_t aligned_data(16) cpu3_idlestack[CONFIG_IDLETHREAD_STACKSIZE];
#endif

#if CONFIG_SMP_NCPUS > 4
static uint8_t aligned_data(16) cpu4_idlestack[CONFIG_IDLETHREAD_STACKSIZE];
#endif

#if CONFIG_SMP_NCPUS > 5
static uint8_t aligned_data(16) cpu5_idlestack[CONFIG_IDLETHREAD_STACKSIZE];
#endif

#if CONFIG_SMP_NCPUS > 6
static uint8_t aligned_data(16) cpu6_idlestack[CONFIG_IDLETHREAD_STACKSIZE];
#endif

#if CONFIG_SMP_NCPUS > 7
static uint8_t aligned_data(16) cpu7_idlestack[CONFIG_IDLETHREAD_STACKSIZE];
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

const uint8_t * const g_cpu_basestack[CONFIG_SMP_NCPUS] =
{
    (uint8_t *)_ebss,
#if CONFIG_SMP_NCPUS > 1
    cpu1_idlestack,
#endif
#if CONFIG_SMP_NCPUS > 2
    cpu2_idlestack,
#endif
#if CONFIG_SMP_NCPUS > 3
    cpu3_idlestack,
#endif
#if CONFIG_SMP_NCPUS > 4
    cpu4_idlestack,
#endif
#if CONFIG_SMP_NCPUS > 5
    cpu5_idlestack,
#endif
#if CONFIG_SMP_NCPUS > 6
    cpu6_idlestack,
#endif
#if CONFIG_SMP_NCPUS > 7
    cpu7_idlestack,
#endif
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

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
 *   - stack_base_ptr: Adjusted stack base pointer after the TLS Data and
 *     Arguments has been removed from the stack allocation.
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

int up_cpu_idlestack(int cpu, struct tcb_s *tcb, size_t stack_size)
{
  uintptr_t stack_alloc;

  DEBUGASSERT(cpu > 0 && cpu < CONFIG_SMP_NCPUS && tcb != NULL &&
              stack_size <= SMP_STACK_SIZE);

  /* Get the top of the stack */

  stack_alloc          = (uintptr_t)g_cpu_basestack[cpu];
  DEBUGASSERT(stack_alloc != 0 && STACK_ISALIGNED(stack_alloc));

  tcb->adj_stack_size  = SMP_STACK_SIZE;
  tcb->stack_alloc_ptr = (void *)stack_alloc;
  tcb->stack_base_ptr  = tcb->stack_alloc_ptr;
  return OK;
}
