/****************************************************************************
 * arch/sim/src/up_createstack.c
 *
 *   Copyright (C) 2007-2009, 2013, 2016 Gregory Nutt. All rights reserved.
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
#include <string.h>

#include <nuttx/arch.h>
#include <nuttx/tls.h>
#include <nuttx/kmalloc.h>

#include "up_internal.h"

/****************************************************************************
 * Pre-processor Macros
 ****************************************************************************/

/* Use a stack alignment of 16 bytes.  If necessary frame_size must be rounded
 * up to the next boundary
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
 ****************************************************************************/

int up_create_stack(FAR struct tcb_s *tcb, size_t stack_size, uint8_t ttype)
{
  FAR uint8_t *stack_alloc_ptr;
  int ret = ERROR;

#ifdef CONFIG_TLS
   /* Add the size of the TLS information structure */

   stack_size += sizeof(struct tls_info_s);

   /* The allocated stack size must not exceed the maximum possible for the
    * TLS feature.
    */

   DEBUGASSERT(stack_size <= TLS_MAXSTACK);
   if (stack_size >= TLS_MAXSTACK)
     {
       stack_size = TLS_MAXSTACK;
     }
#endif

  /* Move up to next even word boundary if necessary */

  size_t adj_stack_size  = STACK_ALIGN_UP(stack_size);

  /* Allocate the memory for the stack */

#ifdef CONFIG_TLS
  stack_alloc_ptr = (FAR uint8_t *)kumm_memalign(TLS_STACK_ALIGN, adj_stack_size);
#else /* CONFIG_TLS */
  stack_alloc_ptr = (FAR uint8_t *)kumm_malloc(adj_stack_size);
#endif /* CONFIG_TLS */

  /* Was the allocation successful? */

  if (stack_alloc_ptr)
    {
      /* This is the address of the last aligned word in the allocation.
       * NOTE that stack_alloc_ptr + adj_stack_size may lie one byte
       * outside of the stack.  This is okay for an inital state; the
       * first pushed values will be within the stack allocation.
       */

      uintptr_t adj_stack_addr =
        STACK_ALIGN_DOWN((uintptr_t)stack_alloc_ptr + adj_stack_size);

      /* Save the values in the TCB */

      tcb->adj_stack_size  = adj_stack_size;
      tcb->stack_alloc_ptr = stack_alloc_ptr;
      tcb->adj_stack_ptr   = (FAR void *)adj_stack_addr;

#ifdef CONFIG_TLS
      /* Initialize the TLS data structure */

      memset(stack_alloc_ptr, 0, sizeof(struct tls_info_s));
#endif

      ret = OK;
    }

  return ret;
}
