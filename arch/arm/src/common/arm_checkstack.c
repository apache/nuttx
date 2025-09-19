/****************************************************************************
 * arch/arm/src/common/arm_checkstack.c
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
#include <sched.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/addrenv.h>
#include <nuttx/arch.h>
#include <nuttx/board.h>

#include "sched/sched.h"
#include "arm_internal.h"

#ifdef CONFIG_STACK_COLORATION

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_TASK_STACK_OVERFLOW_CHECK
static inline bool
arm_check_stack_overflow(uintptr_t stack_base, int threshold)
{
  uint32_t *ptr;
  int free_size;

  for (free_size = 0, ptr = (uint32_t *)stack_base;
       *ptr == STACK_COLOR && free_size < threshold;
       free_size += 4, ptr++);

  return (free_size < threshold);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_stack_check
 *
 * Description:
 *   Determine (approximately) how much stack has been used by searching the
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

size_t arm_stack_check(void *stackbase, size_t nbytes)
{
  uintptr_t start;
  uintptr_t end;
  uint32_t *ptr;
  size_t mark;

  if (nbytes == 0)
    {
      return 0;
    }

  /* Take extra care that we do not check outside the stack boundaries */

  start = STACK_ALIGN_UP((uintptr_t)stackbase);
  end   = STACK_ALIGN_DOWN((uintptr_t)stackbase + nbytes);

  /* Get the adjusted size based on the top and bottom of the stack */

  nbytes  = end - start;

  /* The ARM uses a push-down stack:  the stack grows toward lower addresses
   * in memory.  We need to start at the lowest address in the stack memory
   * allocation and search to higher addresses.  The first word we encounter
   * that does not have the magic value is the high water mark.
   */

  for (ptr = (uint32_t *)start, mark = (nbytes >> 2);
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
      for (i = 0; i < nbytes; i += 4 * 64)
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
 * Name: arm_stack_color
 *
 * Description:
 *   Write a well know value into the stack
 *
 ****************************************************************************/

void arm_stack_color(void *stackbase, size_t nbytes)
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

/****************************************************************************
 * Name: up_check_tcbstack and friends
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
  size_t size;

#ifdef CONFIG_ARCH_ADDRENV
  struct addrenv_s *oldenv;

  if (tcb->addrenv_own != NULL)
    {
      addrenv_select(tcb->addrenv_own, &oldenv);
    }
#endif

  size = arm_stack_check(tcb->stack_base_ptr, tcb->adj_stack_size);

#ifdef CONFIG_ARCH_ADDRENV
  if (tcb->addrenv_own != NULL)
    {
      addrenv_restore(oldenv);
    }
#endif

  return size;
}

#ifdef CONFIG_TASK_STACK_OVERFLOW_CHECK
/****************************************************************************
 * Name: up_check_tcbstack_overflow
 *
 * Description:
 *   Check whether a stack overflow has occurred. The check fails if fewer
 *   than CONFIG_TASK_STACK_OVERFLOW_CHECK_THRESHOLD bytes of unused stack
 *   remain after verifying the stack coloration pattern.
 *
 * Input Parameters:
 *   tcb - The TCB of the task being checked
 *
 * Returned Value:
 *   true  - Stack overflow detected (check failed)
 *   false - No overflow detected (check passed)
 *
 ****************************************************************************/

bool up_check_tcbstack_overflow(FAR struct tcb_s *tcb)
{
  bool overflow;
  int threshold = CONFIG_TASK_STACK_OVERFLOW_CHECK_THRESHOLD;

#ifdef CONFIG_ARCH_ADDRENV
  struct addrenv_s *oldenv;

  if (tcb->addrenv_own != NULL)
    {
      addrenv_select(tcb->addrenv_own, &oldenv);
    }
#endif

  /* Check for stack overflow */

  overflow = arm_check_stack_overflow((uintptr_t)tcb->stack_base_ptr,
                                      threshold);

#ifdef CONFIG_ARCH_ADDRENV
  if (tcb->addrenv_own != NULL)
    {
      addrenv_restore(oldenv);
    }
#endif

  return overflow;
}
#endif

#if CONFIG_ARCH_INTERRUPTSTACK > 3
size_t up_check_intstack(int cpu)
{
  return arm_stack_check((void *)up_get_intstackbase(cpu),
                         STACK_ALIGN_DOWN(CONFIG_ARCH_INTERRUPTSTACK));
}
#endif

#endif /* CONFIG_STACK_COLORATION */
