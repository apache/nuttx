/****************************************************************************
 * arch/arm64/src/common/arm64_createstack.c
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
 * Name: up_create_stack
 *
 * Description:
 *   Allocate a stack for a new thread and setup up stack-related information
 *   in the TCB.
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
 *   - tcb: The TCB of new task
 *   - stack_size:  The requested stack size.  At least this much
 *     must be allocated.
 *   - ttype:  The thread type.  This may be one of following (defined in
 *     include/nuttx/sched.h):
 *
 *       TCB_FLAG_TTYPE_TASK     Normal user task
 *       TCB_FLAG_TTYPE_PTHREAD  User pthread
 *       TCB_FLAG_TTYPE_KERNEL   Kernel thread
 *
 *     This thread type is normally available in the flags field of the TCB,
 *     however, there are certain contexts where the TCB may not be fully
 *     initialized when up_create_stack is called.
 *
 *     If either CONFIG_BUILD_PROTECTED or CONFIG_BUILD_KERNEL are defined,
 *     then this thread type may affect how the stack is allocated.  For
 *     example, kernel thread stacks should be allocated from protected
 *     kernel memory.  Stacks for user tasks and threads must come from
 *     memory that is accessible to user code.
 *
 ****************************************************************************/

int up_create_stack(struct tcb_s *tcb, size_t stack_size, uint8_t ttype)
{
  stack_size = STACK_ALIGN_UP(stack_size);

#ifdef CONFIG_TLS_ALIGNED
  /* The allocated stack size must not exceed the maximum possible for the
   * TLS feature.
   */

  DEBUGASSERT(stack_size <= TLS_MAXSTACK);
  if (stack_size > TLS_MAXSTACK)
    {
      stack_size = TLS_MAXSTACK;
    }
#endif

  /* Is there already a stack allocated of a different size? */

  if (tcb->stack_alloc_ptr && tcb->adj_stack_size != stack_size)
    {
      /* Yes.. Release the old stack */

      up_release_stack(tcb, ttype);
    }

  /* Do we need to allocate a new stack? */

  if (!tcb->stack_alloc_ptr)
    {
      /* Allocate the stack.  If DEBUG is enabled (but not stack debug),
       * then create a zeroed stack to make stack dumps easier to trace.
       * If TLS is enabled, then we must allocate aligned stacks.
       */

#ifdef CONFIG_TLS_ALIGNED
#ifdef CONFIG_MM_KERNEL_HEAP
      /* Use the kernel allocator if this is a kernel thread */

      if (ttype == TCB_FLAG_TTYPE_KERNEL)
        {
          tcb->stack_alloc_ptr = kmm_memalign(TLS_STACK_ALIGN, stack_size);
        }
      else
#endif
        {
          /* Use the user-space allocator if this is a task or pthread */

          tcb->stack_alloc_ptr = kumm_memalign(TLS_STACK_ALIGN, stack_size);
        }

#else /* CONFIG_TLS_ALIGNED */
#ifdef CONFIG_MM_KERNEL_HEAP
      /* Use the kernel allocator if this is a kernel thread */

      if (ttype == TCB_FLAG_TTYPE_KERNEL)
        {
          tcb->stack_alloc_ptr =
              kmm_memalign(STACK_ALIGNMENT, stack_size);
        }
      else
#endif
        {
          /* Use the user-space allocator if this is a task or pthread */

          tcb->stack_alloc_ptr =
              kumm_memalign(STACK_ALIGNMENT, stack_size);
        }
#endif /* CONFIG_TLS_ALIGNED */

#ifdef CONFIG_DEBUG_FEATURES
      /* Was the allocation successful? */

      if (!tcb->stack_alloc_ptr)
        {
          serr("ERROR: Failed to allocate stack, size %ld\n", stack_size);
        }
#endif
    }

  /* Did we successfully allocate a stack? */

  if (tcb->stack_alloc_ptr)
    {
      /* The ARM uses a "full descending" stack:
       * the stack grows toward lower addresses in memory.
       * The stack pointer register points to the last pushed item in
       * the stack.
       * Items on the stack are referenced as positive word offsets from sp.
       */

      /* Since both stack_alloc_ptr and stack_size are in
       * CONFIG_STACK_ALIGNMENT, and the stack ptr is decremented before
       * the first write, we can directly save our variables to struct
       * tcb_s.
       */

      tcb->adj_stack_size = stack_size;
      tcb->stack_base_ptr = tcb->stack_alloc_ptr;

#ifdef CONFIG_STACK_COLORATION
      /* If stack debug is enabled, then fill the stack with a
       * recognizable value that we can use later to test for high
       * water marks.
       */

      arm64_stack_color(tcb->stack_base_ptr, tcb->adj_stack_size);
#endif /* CONFIG_STACK_COLORATION */
      tcb->flags |= TCB_FLAG_FREE_STACK;

      return OK;
    }

  return ERROR;
}
