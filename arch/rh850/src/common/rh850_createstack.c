/****************************************************************************
 * arch/rh850/src/common/rh850_createstack.c
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

#include <errno.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/debug.h>
#include <nuttx/kmalloc.h>
#include <nuttx/sched.h>
#include <nuttx/tls.h>

#include "rh850_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_create_stack
 ****************************************************************************/

int up_create_stack(FAR struct tcb_s *tcb, size_t stack_size, uint8_t ttype)
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
      up_release_stack(tcb, ttype);
    }

  if (!tcb->stack_alloc_ptr)
    {
#ifdef CONFIG_TLS_ALIGNED
      tcb->stack_alloc_ptr = kumm_memalign(TLS_STACK_ALIGN, stack_size);
#else
      tcb->stack_alloc_ptr = kumm_malloc(stack_size);
#endif

#ifdef CONFIG_DEBUG_FEATURES
      if (!tcb->stack_alloc_ptr)
        {
          serr("ERROR: Failed to allocate stack, size %zu\n", stack_size);
        }
#endif
    }

  if (tcb->stack_alloc_ptr)
    {
      uintptr_t top_of_stack;

      top_of_stack = (uintptr_t)tcb->stack_alloc_ptr + stack_size;
      top_of_stack = STACKFRAME_ALIGN_DOWN(top_of_stack);

      tcb->stack_base_ptr = tcb->stack_alloc_ptr;
      tcb->adj_stack_size = top_of_stack - (uintptr_t)tcb->stack_alloc_ptr;

#ifdef CONFIG_STACK_COLORATION
      rh850_stack_color(tcb->stack_base_ptr, tcb->adj_stack_size);
#endif

      tcb->flags |= TCB_FLAG_FREE_STACK;
      board_autoled_on(LED_STACKCREATED);
      return OK;
    }

  return -ENOMEM;
}

#ifdef CONFIG_STACK_COLORATION
/****************************************************************************
 * Name: rh850_stack_color
 *
 * Description:
 *   Write STACK_COLOR to a stack; nbytes = 0 colors the running stack up
 *   to the current stack pointer.
 *
 ****************************************************************************/

void rh850_stack_color(FAR void *stackbase, size_t nbytes)
{
  uint32_t *stkptr;
  uintptr_t stkend;
  size_t nwords;
  uintptr_t sp;

  stkptr = (uint32_t *)STACKFRAME_ALIGN_UP((uintptr_t)stackbase);

  if (nbytes == 0)
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

  stkend = STACKFRAME_ALIGN_DOWN(stkend);
  nwords = (stkend - (uintptr_t)stkptr) >> 2;

  while (nwords-- > 0)
    {
      *stkptr++ = STACK_COLOR;
    }
}
#endif
