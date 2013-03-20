/****************************************************************************
 * arch/arm/src/common/up_createstack.c
 *
 *   Copyright (C) 2007-2013 Gregory Nutt. All rights reserved.
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

#include "up_arch.h"
#include "up_internal.h"

/****************************************************************************
 * Pre-processor Macros
 ****************************************************************************/

/* ARM requires at least a 4-byte stack alignment.  For use with EABI and
 * floating point, the stack must be aligned to 8-byte addresses.
 */

#ifndef CONFIG_STACK_ALIGNMENT

/* The symbol  __ARM_EABI__ is defined by GCC if EABI is being used.  If you
 * are not using GCC, make sure that CONFIG_STACK_ALIGNMENT is set correctly!
 */

#  ifdef __ARM_EABI__
#    define CONFIG_STACK_ALIGNMENT 8
#  else
#    define CONFIG_STACK_ALIGNMENT 4
#  endif
#endif

/* Stack alignment macros */

#define STACK_ALIGN_MASK    (CONFIG_STACK_ALIGNMENT-1)
#define STACK_ALIGN_DOWN(a) ((a) & ~STACK_ALIGN_MASK)
#define STACK_ALIGN_UP(a)   (((a) + STACK_ALIGN_MASK) & ~STACK_ALIGN_MASK)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Name: memset32
 *
 * On most larger then 8 bit archs this will need to be word aligned so
 * so maybe some checks should be put in place?
 *
 ****************************************************************************/

#if defined(CONFIG_DEBUG) && defined(CONFIG_DEBUG_STACK)
static void *memset32(void *s, uint32_t  c, size_t n)
{
  uint32_t *p = (uint32_t *)s;
  while (n-- > 0) *p++ = c;
  return s;
}
#endif

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

      /* The ARM uses a push-down stack:  the stack grows toward lower
       * addresses in memory.  The stack pointer register, points to
       * the lowest, valid work address (the "top" of the stack).  Items
       * on the stack are referenced as positive word offsets from sp.
       */

      top_of_stack = (uint32_t)tcb->stack_alloc_ptr + stack_size - 4;

      /* The ARM stack must be aligned; 4 byte alignment for OABI and
       * 8-byte alignment for EABI. If necessary top_of_stack must be
       * rounded down to the next boundary
       */

      top_of_stack = STACK_ALIGN_DOWN(top_of_stack);

      /* The size of the stack in bytes is then the difference between
       * the top and the bottom of the stack (+4 because if the top
       * is the same as the bottom, then the size is one 32-bit element).
       * The size need not be aligned.
       */

      size_of_stack = top_of_stack - (uint32_t)tcb->stack_alloc_ptr + 4;

      /* Save the adjusted stack values in the struct tcb_s */

      tcb->adj_stack_ptr  = (uint32_t*)top_of_stack;
      tcb->adj_stack_size = size_of_stack;

      /* If stack debug is enabled, then fill the stack with a
       * recognizable value that we can use later to test for high
       * water marks.
       */

#if defined(CONFIG_DEBUG) && defined(CONFIG_DEBUG_STACK)
      memset32(tcb->stack_alloc_ptr, 0xdeadbeef, tcb->adj_stack_size/4);
#endif

      up_ledon(LED_STACKCREATED);
      return OK;
    }

   return ERROR;
}
