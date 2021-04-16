/****************************************************************************
 * arch/sim/src/sim/up_usestack.c
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
#include <string.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/tls.h>

#include "up_internal.h"

/****************************************************************************
 * Pre-processor Macros
 ****************************************************************************/

/* Use a stack alignment of 16 bytes.  If necessary frame_size must be
 * rounded up to the next boundary
 */

#define STACK_ALIGNMENT     16

/* Stack alignment macros */

#define STACK_ALIGN_MASK    (STACK_ALIGNMENT-1)
#define STACK_ALIGN_DOWN(a) ((a) & ~STACK_ALIGN_MASK)
#define STACK_ALIGN_UP(a)   (((a) + STACK_ALIGN_MASK) & ~STACK_ALIGN_MASK)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_use_stack
 *
 * Description:
 *   Setup up stack-related information in the TCB using pre-allocated stack
 *   memory.  This function is called only from nxtask_init() when a task or
 *   kernel thread is started (never for pthreads).
 *
 *   The following TCB fields must be initialized:
 *
 *   - adj_stack_size: Stack size after adjustment for hardware,
 *     processor, etc.  This value is retained only for debug
 *     purposes.
 *   - stack_alloc_ptr: Pointer to allocated stack
 *   - adj_stack_ptr: Adjusted stack_alloc_ptr for HW.  The
 *     initial value of the stack pointer.
 *
 * Input Parameters:
 *   - tcb: The TCB of new task
 *   - stack_size:  The allocated stack size.
 *
 *   NOTE:  Unlike up_stack_create() and up_stack_release, this function
 *   does not require the task type (ttype) parameter.  The TCB flags will
 *   always be set to provide the task type to up_use_stack() if it needs
 *   that information.
 *
 ****************************************************************************/

int up_use_stack(FAR struct tcb_s *tcb, FAR void *stack, size_t stack_size)
{
  uintptr_t adj_stack_addr;
  size_t adj_stack_size;

#ifdef CONFIG_TLS_ALIGNED
  /* Make certain that the user provided stack is properly aligned */

  DEBUGASSERT(((uintptr_t)stack & TLS_STACK_MASK) == 0);
#endif
  /* Move down to next even word boundary within the pre-allocated stack
   * memory, if necessary.
   */

  adj_stack_size = STACK_ALIGN_DOWN(stack_size);

  /* This is the address of the last word in the allocation.
   * NOTE that stack_alloc_ptr + adj_stack_size may lie one byte
   * outside of the stack.  This is okay for an initial state; the
   * first pushed values will be within the stack allocation.
   */

  adj_stack_addr = STACK_ALIGN_DOWN((uintptr_t)stack + adj_stack_size);

  /* Save the values in the TCB */

  tcb->adj_stack_size  = adj_stack_size;
  tcb->stack_alloc_ptr = stack;
  tcb->adj_stack_ptr   = (FAR void *)adj_stack_addr;

  /* Initialize the TLS data structure */

  memset(stack, 0, sizeof(struct tls_info_s));

#if defined(CONFIG_STACK_COLORATION)
  /* If stack debug is enabled, then fill the stack with a
   * recognizable value that we can use later to test for high
   * water marks.
   */

  up_stack_color((FAR void *)((uintptr_t)tcb->stack_alloc_ptr +
                 sizeof(struct tls_info_s)),
                 adj_stack_size - sizeof(struct tls_info_s));
#endif

  return OK;
}
