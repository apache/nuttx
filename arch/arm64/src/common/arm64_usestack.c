/****************************************************************************
 * arch/arm64/src/common/arm64_usestack.c
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
#include <stdint.h>
#include <sched.h>
#include <debug.h>
#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/tls.h>
#include <nuttx/board.h>
#include <arch/irq.h>
#include "arm64_internal.h"
#include "arm64_fatal.h"

/****************************************************************************
 * Pre-processor Macros
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

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
 *   - stack_base_ptr: Adjusted stack base pointer after the TLS Data and
 *     Arguments has been removed from the stack allocation.
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

int up_use_stack(struct tcb_s *tcb, void *stack, size_t stack_size)
{
#ifdef CONFIG_TLS_ALIGNED
  /* Make certain that the user provided stack is properly aligned */

  DEBUGASSERT(((uintptr_t)stack & TLS_STACK_MASK) == 0);
#endif

  /* Is there already a stack allocated? */

  if (tcb->stack_alloc_ptr)
    {
      /* Yes... Release the old stack allocation */

      up_release_stack(tcb, tcb->flags & TCB_FLAG_TTYPE_MASK);
    }

  /* The ARM uses a "full descending" stack:
   * the stack grows toward lower addresses in memory.
   * The stack pointer register, points to the last pushed item
   * in the stack.
   * Items on the stack are referenced as positive word offsets from sp.
   */

  /* We align all sizes and pointer to CONFIG_STACK_ALIGNMENT.
   * Since the stack ptr is decremented before
   * the first write, we can directly save our variables to struct
   * tcb_s.
   */

  /* Save the new stack allocation */

  tcb->stack_alloc_ptr = stack;
  tcb->stack_base_ptr  = tcb->stack_alloc_ptr;
  tcb->adj_stack_size  =
         STACK_ALIGN_DOWN((uintptr_t)stack + stack_size) - (uintptr_t)stack;

#ifdef CONFIG_STACK_COLORATION
  /* If stack debug is enabled, then fill the stack with a
   * recognizable value that we can use later to test for high
   * water marks.
   */

  arm64_stack_color(tcb->stack_base_ptr, tcb->adj_stack_size);
#endif /* CONFIG_STACK_COLORATION */

  return OK;
}
