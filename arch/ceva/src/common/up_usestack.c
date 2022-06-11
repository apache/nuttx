/****************************************************************************
 * arch/ceva/src/common/up_usestack.c
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

#include <nuttx/arch.h>
#include <nuttx/tls.h>

#include "up_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_use_stack
 *
 * Description:
 *   Setup up stack-related information in the TCB using pre-allocated stack
 *   memory.  This function is called only from task_init() when a task or
 *   kernel thread is started (never for pthreads).
 *
 *   The following TCB fields must be initialized:
 *
 *   - adj_stack_size: Stack size after adjustment for hardware,
 *     processor, etc.  This value is retained only for debug
 *     purposes.
 *   - stack_alloc_ptr: Pointer to allocated stack
 *   - stack_base_ptr: Adjusted stack_alloc_ptr for HW.  The
 *     initial value of the stack pointer.
 *
 * Inputs:
 *   - tcb: The TCB of new task
 *   - stack_size:  The allocated stack size.
 *
 *   NOTE:  Unlike up_stack_create() and up_stack_release, this function
 *   does not require the task type (ttype) parameter.  The TCB flags will
 *   always be set to provide the task type to up_use_stack() if it needs
 *   that information.
 *
 ****************************************************************************/

int up_use_stack(struct tcb_s *tcb, void *stack, size_t stack_size)
{
  uintptr_t top_of_stack;
  size_t size_of_stack;

#ifdef CONFIG_TLS
  /* Make certain that the user provided stack is properly aligned */

  DEBUGASSERT((uintptr_t)stack & (TLS_STACK_ALIGN - 1) == 0);
#else
  DEBUGASSERT((uintptr_t)stack & STACK_ALIGN_MASK == 0);
#endif

  /* Is there already a stack allocated? */

  if (tcb->stack_alloc_ptr)
    {
      /* Yes... Release the old stack allocation */

      up_release_stack(tcb, tcb->flags & TCB_FLAG_TTYPE_MASK);
    }

  /* Save the new stack allocation */

  tcb->stack_alloc_ptr = stack;

  /* The CEVA uses a push-down stack:  the stack grows toward lower addresses
   * in memory.  The stack pointer register, points to the lowest, valid
   * work address (the "top" of the stack).  Items on the stack are
   * referenced as positive word offsets from sp.
   */

  /* The CEVA stack must be aligned to 4-byte alignment.
   * If necessary size_of_stack must be rounded down to the next
   * boundary
   */

  top_of_stack = (uintptr_t)tcb->stack_alloc_ptr + stack_size;
  top_of_stack = STACK_ALIGN_DOWN(top_of_stack);
  size_of_stack = top_of_stack - (uintptr_t)tcb->stack_alloc_ptr;

  /* Save the adjusted stack values in the struct tcb_s */

  tcb->stack_base_ptr = tcb->stack_alloc_ptr;
  tcb->adj_stack_size = size_of_stack;

#ifdef CONFIG_STACK_COLORATION
  /* If stack debug is enabled, then fill the stack with a recognizable
   * value that we can use later to test for high water marks.
   */

  up_stack_color(tcb->stack_alloc_ptr, tcb->adj_stack_size);
#endif

  return OK;
}
