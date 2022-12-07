/****************************************************************************
 * arch/x86_64/src/common/x86_64_assert.c
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

#include <stdint.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/board.h>

#include <arch/board/board.h>

#include "sched/sched.h"
#include "x86_64_internal.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: x86_64_stackdump
 ****************************************************************************/

#ifdef CONFIG_ARCH_STACKDUMP
static void x86_64_stackdump(uint64_t sp, uint64_t stack_top)
{
  uint64_t stack;

  for (stack = sp & ~0x1f; stack < (stack_top & ~0x1f); stack += 32)
    {
      uint32_t *ptr = (uint32_t *)stack;
      _alert("%016" PRIx64 ": %08" PRIx32 " %08" PRIx32 " %08" PRIx32
             " %08" PRIx32 " %08" PRIx32 " %08" PRIx32 " %08" PRIx32
             " %08" PRIx32 "\n",
             stack, ptr[0], ptr[1], ptr[2], ptr[3],
             ptr[4], ptr[5], ptr[6], ptr[7]);
    }
}
#endif

/****************************************************************************
 * Name: x86_64_dumpstate
 ****************************************************************************/

#ifdef CONFIG_ARCH_STACKDUMP
static void x86_64_dumpstate(void)
{
  struct tcb_s *rtcb = running_task();
  uint64_t sp = up_getsp();
  uint64_t ustackbase;
  uint64_t ustacksize;
#if CONFIG_ARCH_INTERRUPTSTACK > 3
  uint64_t istackbase;
  uint64_t istacksize;
#endif

  /* Get the limits on the user stack memory */

  ustackbase = (uint64_t)rtcb->stack_base_ptr;
  ustacksize = (uint64_t)rtcb->adj_stack_size;

  /* Get the limits on the interrupt stack memory */

#if CONFIG_ARCH_INTERRUPTSTACK > 3
  istackbase = (uint64_t)g_intstackalloc;
  istacksize = (CONFIG_ARCH_INTERRUPTSTACK & ~3);

  /* Show interrupt stack info */

  _alert("sp:     %016" PRIx64 "\n", sp);
  _alert("IRQ stack:\n");
  _alert("  base: %016" PRIx64 "\n", istackbase);
  _alert("  size: %016" PRIx64 "\n", istacksize);

  /* Does the current stack pointer lie within the interrupt
   * stack?
   */

  if (sp >= istackbase && sp < istackbase + istacksize)
    {
      /* Yes.. dump the interrupt stack */

      x86_64_stackdump(sp, istackbase + istacksize);

      /* Extract the user stack pointer which should lie
       * at the base of the interrupt stack.
       */

      sp = g_intstacktop;
      _alert("sp:     %016" PRIx64 "\n", sp);
    }
  else if (g_current_regs)
    {
      _alert("ERROR: Stack pointer is not within the interrupt stack\n");
      x86_64_stackdump(istackbase, istackbase + istacksize);
    }

  /* Show user stack info */

  _alert("User stack:\n");
  _alert("  base: %016" PRIx64 "\n", ustackbase);
  _alert("  size: %016" PRIx64 "\n", ustacksize);
#else
  _alert("sp:         %016" PRIx64 "\n", sp);
  _alert("stack base: %016" PRIx64 "\n", ustackbase);
  _alert("stack size: %016" PRIx64 "\n", ustacksize);
#endif

  /* Dump the user stack if the stack pointer lies within the allocated user
   * stack memory.
   */

  if (sp >= ustackbase && sp < ustackbase + ustacksize)
    {
      x86_64_stackdump(sp, ustackbase + ustacksize);
    }
  else
    {
      _alert("ERROR: Stack pointer is not within allocated stack\n");
      x86_64_stackdump(ustackbase, ustackbase + ustacksize);
    }

  /* Then dump the registers (if available) */

  if (g_current_regs != NULL)
    {
      x86_64_registerdump((uint64_t *)g_current_regs);
    }
}
#else
# define x86_64_dumpstate()
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_assert
 ****************************************************************************/

void up_assert(const char *filename, int lineno)
{
  board_autoled_on(LED_ASSERTION);
  x86_64_dumpstate();
}
