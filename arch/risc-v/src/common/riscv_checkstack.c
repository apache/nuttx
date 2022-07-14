/****************************************************************************
 * arch/risc-v/src/common/riscv_checkstack.c
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
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>

#include "sched/sched.h"
#include "riscv_internal.h"

#ifdef CONFIG_STACK_COLORATION

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static size_t do_stackcheck(uintptr_t alloc, size_t size);

/****************************************************************************
 * Name: do_stackcheck
 *
 * Description:
 *   Determine (approximately) how much stack has been used be searching the
 *   stack memory for a high water mark.  That is, the deepest level of the
 *   stack that clobbered some recognizable marker in the stack memory.
 *
 * Input Parameters:
 *   alloc - Allocation base address of the stack
 *   size - The size of the stack in bytes
 *
 * Returned Value:
 *   The estimated amount of stack space used.
 *
 ****************************************************************************/

static size_t do_stackcheck(uintptr_t alloc, size_t size)
{
  uintptr_t start;
  uintptr_t end;
  uint32_t *ptr;
  size_t mark;

  if (size == 0)
    {
      return 0;
    }

  /* Get aligned addresses of the top and bottom of the stack */

  start = (alloc + 3) & ~3;
  end   = (alloc + size) & ~3;

  /* Get the adjusted size based on the top and bottom of the stack */

  size  = end - start;

  /* RISC-V uses a push-down stack:  the stack grows toward lower addresses
   * in memory.  We need to start at the lowest address in the stack memory
   * allocation and search to higher addresses.  The first word we encounter
   * that does not have the magic value is the high water mark.
   */

  for (ptr = (uint32_t *)start, mark = (size >> 2);
       *ptr == STACK_COLOR && mark > 0;
       ptr++, mark--);

  /* If the stack is completely used, then this might mean that the stack
   * overflowed from above (meaning that the stack is too small), or may
   * have been overwritten from below meaning that some other stack or data
   * structure overflowed.
   *
   * If you see returned values saying that the entire stack is being used
   * then enable the following logic to see it there are unused areas in the
   * middle of the stack.
   */

#if 0
  if (mark + 16 > nwords)
    {
      int i;
      int j;

      ptr = (uint32_t *)start;
      for (i = 0; i < size; i += 4 * 64)
        {
          for (j = 0; j < 64; j++)
            {
              int ch;
              if (*ptr++ == STACK_COLOR)
                {
                  ch = '.';
                }
              else
                {
                  ch = 'X';
                }

              up_putc(ch);
            }

          up_putc('\n');
        }
    }
#endif

  /* Return our guess about how much stack space was used */

  return mark << 2;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_check_stack and friends
 *
 * Description:
 *   Determine (approximately) how much stack has been used be searching the
 *   stack memory for a high water mark.  That is, the deepest level of the
 *   stack that clobbered some recognizable marker in the stack memory.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   The estimated amount of stack space used.
 *
 ****************************************************************************/

size_t up_check_tcbstack(struct tcb_s *tcb)
{
  return do_stackcheck((uintptr_t)tcb->stack_base_ptr, tcb->adj_stack_size);
}

ssize_t up_check_tcbstack_remain(struct tcb_s *tcb)
{
  return tcb->adj_stack_size - up_check_tcbstack(tcb);
}

size_t up_check_stack(void)
{
  return up_check_tcbstack(running_task());
}

ssize_t up_check_stack_remain(void)
{
  return up_check_tcbstack_remain(running_task());
}

#if CONFIG_ARCH_INTERRUPTSTACK > 15
size_t up_check_intstack(void)
{
  return do_stackcheck((uintptr_t)&g_intstackalloc,
                       (CONFIG_ARCH_INTERRUPTSTACK & ~15));
}

size_t up_check_intstack_remain(void)
{
  return (CONFIG_ARCH_INTERRUPTSTACK & ~15) - up_check_intstack();
}
#endif

#endif /* CONFIG_STACK_COLORATION */
