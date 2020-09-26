/****************************************************************************
 * arch/arm/src/common/arm_usestack.c
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
#include <string.h>
#include <sched.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/arch.h>
#include <nuttx/tls.h>

#include "arm_internal.h"

/****************************************************************************
 * Pre-processor Macros
 ****************************************************************************/

/* For use with EABI and floating point, the stack must be aligned to 8-byte
 * addresses.
 */

#define CONFIG_STACK_ALIGNMENT 8

/* Stack alignment macros */

#define STACK_ALIGN_MASK    (CONFIG_STACK_ALIGNMENT - 1)
#define STACK_ALIGN_DOWN(a) ((a) & ~STACK_ALIGN_MASK)
#define STACK_ALIGN_UP(a)   (((a) + STACK_ALIGN_MASK) & ~STACK_ALIGN_MASK)

/* 32bit alignment macros */

#define INT32_ALIGN_MASK    (3)
#define INT32_ALIGN_DOWN(a) ((a) & ~INT32_ALIGN_MASK)
#define INT32_ALIGN_UP(a)   (((a) + INT32_ALIGN_MASK) & ~INT32_ALIGN_MASK)

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
 *   - adj_stack_ptr: Adjusted stack_alloc_ptr for HW.  The
 *     initial value of the stack pointer.
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
  size_t tls_size;

#ifdef CONFIG_TLS_ALIGNED
  /* Make certain that the user provided stack is properly aligned */

  DEBUGASSERT(((uintptr_t)stack & TLS_STACK_MASK) == 0);
#endif

  tls_size = INT32_ALIGN_UP(sizeof(struct tls_info_s));

  /* Is there already a stack allocated? */

  if (tcb->stack_alloc_ptr)
    {
      /* Yes... Release the old stack allocation */

      up_release_stack(tcb, tcb->flags & TCB_FLAG_TTYPE_MASK);
    }

  /* The ARM uses a "full descending" stack:
   * the stack grows toward lower addresses in memory.
   * The stack pointer register, points to the last pushed item in the stack.
   * Items on the stack are referenced as positive word offsets from sp.
   */

  /* We align all sizes and pointer to CONFIG_STACK_ALIGNMENT.
   * Since the stack ptr is decremented before
   * the first write, we can directly save our variables to struct
   * tcb_s.
   */

  /* Save the new stack allocation */

  tcb->stack_alloc_ptr = stack;

  /* Align stack top */

  tcb->adj_stack_ptr =
      (FAR void *)STACK_ALIGN_DOWN((uintptr_t)stack + stack_size);

  /* Offset by tls_size */

  stack = (FAR void *)((uintptr_t)stack + tls_size);

  /* Is there enough room for at least TLS ? */

  if ((uintptr_t)stack > (uintptr_t)tcb->adj_stack_ptr)
    {
      return -ENOMEM;
    }

  tcb->adj_stack_size = (uintptr_t)tcb->adj_stack_ptr - (uintptr_t)stack;

  /* Initialize the TLS data structure */

  memset(tcb->stack_alloc_ptr, 0, tls_size);

#ifdef CONFIG_STACK_COLORATION
  /* If stack debug is enabled, then fill the stack with a
   * recognizable value that we can use later to test for high
   * water marks.
   */

  if (tcb->pid != 0)
    {
      arm_stack_color((FAR void *)((uintptr_t)tcb->adj_stack_ptr -
          tcb->adj_stack_size), tcb->adj_stack_size);
    }
#endif /* CONFIG_STACK_COLORATION */

  return OK;
}
