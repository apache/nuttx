/****************************************************************************
 * arch/rh850/src/common/rh850_usestack.c
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
 * Name: up_use_stack
 ****************************************************************************/

int up_use_stack(FAR struct tcb_s *tcb, FAR void *stack, size_t stack_size)
{
  uintptr_t top_of_stack;

#ifdef CONFIG_TLS_ALIGNED
  DEBUGASSERT(stack_size <= TLS_MAXSTACK);
  if (stack_size >= TLS_MAXSTACK)
    {
      stack_size = TLS_MAXSTACK;
    }
#endif

  if (tcb->stack_alloc_ptr)
    {
      up_release_stack(tcb, tcb->flags & TCB_FLAG_TTYPE_MASK);
    }

  tcb->stack_alloc_ptr = stack;
  tcb->stack_base_ptr  = (FAR void *)STACKFRAME_ALIGN_UP((uintptr_t)stack);

  top_of_stack = STACKFRAME_ALIGN_DOWN((uintptr_t)stack + stack_size);
  tcb->adj_stack_size = top_of_stack - (uintptr_t)tcb->stack_base_ptr;

#ifdef CONFIG_STACK_COLORATION
  rh850_stack_color(tcb->stack_base_ptr, tcb->adj_stack_size);
#endif

  return OK;
}
