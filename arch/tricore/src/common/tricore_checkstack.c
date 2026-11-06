/****************************************************************************
 * arch/tricore/src/common/tricore_checkstack.c
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
#include <nuttx/debug.h>

#include <nuttx/addrenv.h>
#include <nuttx/arch.h>

#include "sched/sched.h"
#include "tricore_internal.h"

#ifdef CONFIG_STACK_COLORATION

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tricore_stack_check
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

size_t tricore_stack_check(uintptr_t alloc, size_t size)
{
  uintptr_t start;
  uintptr_t end;
  uint32_t *ptr;
  size_t checksize;

  if (size == 0)
    {
      return 0;
    }

  /* Get aligned addresses of the top and bottom of the stack */

  start = STACKFRAME_ALIGN_UP((uintptr_t)alloc);
  end   = STACKFRAME_ALIGN_DOWN((uintptr_t)alloc + size);

  /* Get the adjusted size based on the top and bottom of the stack */

  size = (end - start) >> 2;
  checksize = size;

  /* Find the first colored CSA, skipping each CSA's PCXI link word. */

  for (ptr = (uint32_t *)start; size >= TC_CONTEXT_REGS;
       size -= TC_CONTEXT_REGS, ptr += TC_CONTEXT_REGS)
    {
      if (*(ptr + 1) == STACK_COLOR)
        {
          break;
        }
    }

  /* Count the remaining unused words.  PCXI link words are always unused. */

  for (; checksize > 0 && size > 0; size--, ptr++)
    {
      if (*ptr == STACK_COLOR || TC_CONTEXT_ALIGNED(ptr))
        {
          checksize--;
        }
      else
        {
          break;
        }
    }

  return checksize << 2;
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

size_t up_check_tcbstack(struct tcb_s *tcb, size_t check_size)
{
  size_t size;

#ifdef CONFIG_ARCH_ADDRENV
  struct addrenv_s *oldenv;

  if (tcb->addrenv_own != NULL)
    {
      addrenv_select(tcb->addrenv_own, &oldenv);
    }
#endif

  size = tricore_stack_check((uintptr_t)tcb->stack_base_ptr, check_size);

#ifdef CONFIG_ARCH_ADDRENV
  if (tcb->addrenv_own != NULL)
    {
      addrenv_restore(oldenv);
    }
#endif

  return size;
}

#if CONFIG_ARCH_INTERRUPTSTACK > 15
size_t up_check_intstack(int cpu, size_t check_size)
{
  if (check_size == 0)
    {
      check_size = CONFIG_ARCH_INTERRUPTSTACK & ~15;
    }

  return tricore_stack_check((uintptr_t)g_intstackalloc, check_size);
}
#endif

#endif /* CONFIG_STACK_COLORATION */
