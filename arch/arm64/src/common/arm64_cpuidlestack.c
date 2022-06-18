/****************************************************************************
 * arch/arm64/src/common/arm64_cpuidlestack.c
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
#include <nuttx/sched.h>

#include "arm64_smp.h"
#include "arm64_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Stack alignment macros */

#define STACK_ISALIGNED(a) ((uintptr_t)(a) & ~STACK_ALIGN_MASK)

/****************************************************************************
 * Private Data
 ****************************************************************************/

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
 *   in different ways in different environments:
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
#if CONFIG_SMP_NCPUS > 1
  uintptr_t stack_alloc;

  DEBUGASSERT(cpu > 0 && cpu < CONFIG_SMP_NCPUS && tcb != NULL &&
              stack_size <= SMP_STACK_SIZE);

  /* Get the top of the stack */

  stack_alloc          = (uintptr_t)g_cpu_idlestackalloc[cpu];
  DEBUGASSERT(stack_alloc != 0 && STACK_ISALIGNED(stack_alloc));

  tcb->adj_stack_size  = SMP_STACK_SIZE;
  tcb->stack_alloc_ptr = (void *)stack_alloc;
  tcb->stack_base_ptr  = tcb->stack_alloc_ptr;
#endif

  return OK;
}
