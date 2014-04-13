/****************************************************************************
 * arch/sh/src/common/up_stackframe.c
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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

#include <nuttx/arch.h>

#include "up_internal.h"

/****************************************************************************
 * Pre-processor Macros
 ****************************************************************************/
/* The SH stack must be aligned at word (4 byte) boundaries. If necessary
 * frame_size must be rounded up to the next boundary
 */

#define STACK_ALIGNMENT     4

/* Stack alignment macros */

#define STACK_ALIGN_MASK    (STACK_ALIGNMENT-1)
#define STACK_ALIGN_DOWN(a) ((a) & ~STACK_ALIGN_MASK)
#define STACK_ALIGN_UP(a)   (((a) + STACK_ALIGN_MASK) & ~STACK_ALIGN_MASK)

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
 *   - adj_stack_ptr: Adjusted initial stack pointer after the frame has
 *     been removed from the stack.  This will still be the initial value
 *     of the stack pointer when the task is started.
 *
 * Inputs:
 *   - tcb:  The TCB of new task
 *   - frame_size:  The size of the stack frame to allocate.
 *
 *  Returned Value:
 *   - A pointer to bottom of the allocated stack frame.  NULL will be
 *     returned on any failures.  The alignment of the returned value is
 *     the same as the alignment of the stack itself.
 *
 ****************************************************************************/

FAR void *up_stack_frame(FAR struct tcb_s *tcb, size_t frame_size)
{
  uintptr_t topaddr;

  /* Align the frame_size */

  frame_size = STACK_ALIGN_UP(frame_size);

  /* Is there already a stack allocated? Is it big enough? */

  if (!tcb->stack_alloc_ptr || tcb->adj_stack_size <= frame_size)
    {
      return NULL;
    }

  /* Save the adjusted stack values in the struct tcb_s */

  topaddr              = (uintptr_t)tcb->adj_stack_ptr - frame_size;
  tcb->adj_stack_ptr   = (FAR void *)topaddr;
  tcb->adj_stack_size -= frame_size;

  /* Reset the initial state */

  up_initial_state(tcb);

  /* And return a pointer to allocated memory */

  return (FAR void *)(topaddr + sizeof(uint32_t));
}
