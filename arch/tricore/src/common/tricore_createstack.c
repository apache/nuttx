/****************************************************************************
 * arch/tricore/src/common/tricore_createstack.c
 *
 * SPDX-License-Identifier: Apache-2.0
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
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/kmalloc.h>
#include <nuttx/sched.h>

#include <tricore_internal.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int up_create_stack(struct tcb_s *tcb, size_t stack_size, uint8_t ttype)
{
#ifdef CONFIG_TLS_ALIGNED

  DEBUGASSERT(stack_size <= TLS_MAXSTACK);
  if (stack_size >= TLS_MAXSTACK)
    {
      stack_size = TLS_MAXSTACK;
    }
#endif

  if (tcb->stack_alloc_ptr && tcb->adj_stack_size != stack_size)
    {
      /* Yes.. Release the old stack */

      up_release_stack(tcb, ttype);
    }

  /* Do we need to allocate a new stack? */

  if (!tcb->stack_alloc_ptr)
    {
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
          tcb->stack_alloc_ptr = kmm_malloc(stack_size);
        }
      else
#endif
        {
          /* Use the user-space allocator if this is a task or pthread */

          tcb->stack_alloc_ptr = kumm_malloc(stack_size);
        }
#endif /* CONFIG_TLS_ALIGNED */

#ifdef CONFIG_DEBUG_FEATURES
      /* Was the allocation successful? */

      if (!tcb->stack_alloc_ptr)
        {
          serr("ERROR: Failed to allocate stack, size %zu\n", stack_size);
        }
#endif
    }

  /* Did we successfully allocate a stack? */

  if (tcb->stack_alloc_ptr)
    {
      uintptr_t top_of_stack;
      size_t size_of_stack;

      top_of_stack = (uintptr_t)tcb->stack_alloc_ptr + stack_size;

      top_of_stack = STACK_ALIGN_DOWN(top_of_stack);
      size_of_stack = top_of_stack - (uintptr_t)tcb->stack_alloc_ptr;

      /* Save the adjusted stack values in the struct tcb_s */

      tcb->stack_base_ptr = tcb->stack_alloc_ptr;
      tcb->adj_stack_size = size_of_stack;

#ifdef CONFIG_STACK_COLORATION

      tricore_stack_color(tcb->stack_base_ptr, tcb->adj_stack_size);

#endif /* CONFIG_STACK_COLORATION */

      tcb->flags |= TCB_FLAG_FREE_STACK;

      return OK;
    }

  return ERROR;
}

#ifdef CONFIG_STACK_COLORATION
void tricore_stack_color(void *stackbase, size_t nbytes)
{
  uint32_t *stkptr;
  uintptr_t stkend;
  size_t    nwords;
  uintptr_t sp;

  /* Take extra care that we do not write outside the stack boundaries */

  stkptr = (uint32_t *)STACK_ALIGN_UP((uintptr_t)stackbase);

  if (nbytes == 0) /* 0: colorize the running stack */

    {
      stkend = up_getsp();
      if (stkend > (uintptr_t)&sp)
        {
          stkend = (uintptr_t)&sp;
        }
    }
  else
    {
      stkend = (uintptr_t)stackbase + nbytes;
    }

  stkend = STACK_ALIGN_DOWN(stkend);
  nwords = (stkend - (uintptr_t)stkptr) >> 2;

  /* Set the entire stack to the coloration value */

  while (nwords-- > 0)
    {
      *stkptr++ = STACK_COLOR;
    }
}
#endif
