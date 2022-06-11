/****************************************************************************
 * arch/misoc/src/lm32/lm32_stackframe.c
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

#include "lm32.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_stack_frame
 *
 * Description:
 *   Allocate a stack frame in the TCB's stack to hold thread-specific data.
 *   This function may be called anytime after up_create_stack() or
 *   up_use_stack() have been called but before the task has been started.
 *
 *   Thread data may be kept in the stack (instead of in the TCB) if it is
 *   accessed by the user code directly.  This includes such things as
 *   argv[].  The stack memory is guaranteed to be in the same protection
 *   domain as the thread.
 *
 *   The following TCB fields will be re-initialized:
 *
 *   - adj_stack_size: Stack size after removal of the stack frame from
 *     the stack
 *   - stack_base_ptr: Adjusted stack base pointer after the TLS Data and
 *     Arguments has been removed from the stack allocation.
 *
 * Input Parameters:
 *   - tcb:  The TCB of new task
 *   - frame_size:  The size of the stack frame to allocate.
 *
 *  Returned Value:
 *   - A pointer to bottom of the allocated stack frame.  NULL will be
 *     returned on any failures.  The alignment of the returned value is
 *     the same as the alignment of the stack itself.
 *
 ****************************************************************************/

void *up_stack_frame(struct tcb_s *tcb, size_t frame_size)
{
  void *ret;

  /* Align the frame_size */

  frame_size = STACK_ALIGN_UP(frame_size);

  /* Is there already a stack allocated? Is it big enough? */

  if (!tcb->stack_alloc_ptr || tcb->adj_stack_size <= frame_size)
    {
      return NULL;
    }

  ret = tcb->stack_base_ptr;
  memset(ret, 0, frame_size);

  /* Save the adjusted stack values in the struct tcb_s */

  tcb->stack_base_ptr  = (uint8_t *)tcb->stack_base_ptr + frame_size;
  tcb->adj_stack_size -= frame_size;

  /* And return the pointer to the allocated region */

  return ret;
}
