/****************************************************************************
 * arch/z16/common/up_createstack.c
 *
 *   Copyright (C) 2008-2009, 2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
#include <stdint.h>
#include <sched.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "chip/chip.h"
#include "up_internal.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Global Functions
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
 *   - adj_stack_ptr: Adjusted stack_alloc_ptr for HW.  The initial value of
 *     the stack pointer.
 *
 * Inputs:
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
 *     If CONFIG_NUTTX_KERNEL is defined, then this thread type may affect
 *     how the stack is allocated.  For example, kernel thread stacks should
 *     be allocated from protected kernel memory.  Stacks for user tasks and
 *     threads must come from memory that is accessible to user code.
 *
 ****************************************************************************/

int up_create_stack(FAR struct tcb_s *tcb, size_t stack_size, uint8_t ttype)
{
  /* Is there already a stack allocated of a different size?  Because of
   * alignment issues, stack_size might erroneously appear to be of a
   * different size.  Fortunately, this is not a critical operation.
   */

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
       */

#if defined(CONFIG_NUTTX_KERNEL) && defined(CONFIG_MM_KERNEL_HEAP)
      /* Use the kernel allocator if this is a kernel thread */

      if (ttype == TCB_FLAG_TTYPE_KERNEL)
        {
#if defined(CONFIG_DEBUG) && !defined(CONFIG_DEBUG_STACK)
          tcb->stack_alloc_ptr = (uint32_t *)kzalloc(stack_size);
#else
          tcb->stack_alloc_ptr = (uint32_t *)kmalloc(stack_size);
#endif
        }
      else
#endif
        {
          /* Use the user-space allocator if this is a task or pthread */

#if defined(CONFIG_DEBUG) && !defined(CONFIG_DEBUG_STACK)
          tcb->stack_alloc_ptr = (uint32_t *)kuzalloc(stack_size);
#else
          tcb->stack_alloc_ptr = (uint32_t *)kumalloc(stack_size);
#endif
        }

#ifdef CONFIG_DEBUG
      /* Was the allocation successful? */

      if (!tcb->stack_alloc_ptr)
        {
          sdbg("ERROR: Failed to allocate stack, size %d\n", stack_size);
        }
#endif
    }

  /* Did we successfully allocate a stack? */

  if (tcb->stack_alloc_ptr)
    {
      size_t top_of_stack;
      size_t size_of_stack;

      /* Yes.. If stack debug is enabled, then fill the stack with a
       * recognizable value that we can use later to test for high
       * water marks.
       */

#if defined(CONFIG_DEBUG) && defined(CONFIG_DEBUG_STACK)
      memset(tcb->stack_alloc_ptr, 0xaa, stack_size);
#endif

      /* The ZNeo uses a push-down stack:  the stack grows toward lower
       * addresses in memory.  The stack pointer register, points to the
       * lowest, valid work address (the "top" of the stack).  Items on
       * the stack are referenced as positive word offsets from sp.
       */

      top_of_stack = (uint32_t)tcb->stack_alloc_ptr + stack_size - 4;

      /* Align the stack to word (4 byte) boundaries.  This is probably
       * a greater alignement than is required.
       */

      top_of_stack &= ~3;
      size_of_stack = top_of_stack - (uint32_t)tcb->stack_alloc_ptr + 4;

      /* Save the adjusted stack values in the struct tcb_s */

      tcb->adj_stack_ptr  = (uint32_t*)top_of_stack;
      tcb->adj_stack_size = size_of_stack;

      board_led_on(LED_STACKCREATED);
      return OK;
    }

   return ERROR;
}
