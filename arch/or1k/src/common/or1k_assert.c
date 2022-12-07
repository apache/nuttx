/****************************************************************************
 * arch/or1k/src/common/or1k_assert.c
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
#include <nuttx/arch.h>
#include <nuttx/board.h>

#include <arch/board/board.h>

#include "sched/sched.h"
#include "or1k_internal.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_ARCH_STACKDUMP
static uint32_t s_last_regs[XCPTCONTEXT_REGS];
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: or1k_stackdump
 ****************************************************************************/

#ifdef CONFIG_ARCH_STACKDUMP
static void or1k_stackdump(uint32_t sp, uint32_t stack_top)
{
  uint32_t stack;

  for (stack = sp & ~0x1f; stack < (stack_top & ~0x1f); stack += 32)
    {
      uint32_t *ptr = (uint32_t *)stack;
      _alert("%08x: %08x %08x %08x %08x %08x %08x %08x %08x\n",
             stack, ptr[0], ptr[1], ptr[2], ptr[3],
             ptr[4], ptr[5], ptr[6], ptr[7]);
    }
}
#endif

/****************************************************************************
 * Name: up_taskdump
 ****************************************************************************/

#ifdef CONFIG_STACK_COLORATION
static void up_taskdump(struct tcb_s *tcb, void *arg)
{
  /* Dump interesting properties of this task */

#if CONFIG_TASK_NAME_SIZE > 0
  _alert("%s: PID=%d Stack Used=%lu of %lu\n",
         tcb->name, tcb->pid, (unsigned long)up_check_tcbstack(tcb),
         (unsigned long)tcb->adj_stack_size);
#else
  _alert("PID: %d Stack Used=%lu of %lu\n",
         tcb->pid, (unsigned long)up_check_tcbstack(tcb),
         (unsigned long)tcb->adj_stack_size);
#endif
}
#endif

/****************************************************************************
 * Name: up_showtasks
 ****************************************************************************/

#ifdef CONFIG_STACK_COLORATION
static inline void up_showtasks(void)
{
  /* Dump interesting properties of each task in the crash environment */

  nxsched_foreach(up_taskdump, NULL);
}
#else
#  define up_showtasks()
#endif

/****************************************************************************
 * Name: or1k_registerdump
 ****************************************************************************/

#ifdef CONFIG_ARCH_STACKDUMP
static inline void or1k_registerdump(void)
{
  volatile uint32_t *regs = CURRENT_REGS;

  /* Are user registers available from interrupt processing? */

  if (regs == NULL)
    {
      /* No.. capture user registers by hand */

      up_saveusercontext(s_last_regs);
      regs = s_last_regs;
    }

  /* Dump the interrupt registers */

  _alert("R0: %08x %08x %08x %08x %08x %08x %08x %08x\n",
         regs[REG_R0], regs[REG_R1], regs[REG_R2], regs[REG_R3],
         regs[REG_R4], regs[REG_R5], regs[REG_R6], regs[REG_R7]);
  _alert("R8: %08x %08x %08x %08x %08x %08x %08x %08x\n",
         regs[REG_R8],  regs[REG_R9],  regs[REG_R10], regs[REG_R11],
         regs[REG_R12], regs[REG_R13], regs[REG_R14], regs[REG_R15]);
#ifdef CONFIG_BUILD_PROTECTED
  _alert("xPSR: %08x PRIMASK: %08x EXEC_RETURN: %08x\n",
         regs[REG_XPSR], regs[REG_PRIMASK], regs[REG_EXC_RETURN]);
#else
  _alert("xPSR: %08x PRIMASK: %08x\n",
         regs[REG_XPSR], regs[REG_PRIMASK]);
#endif
}
#endif

/****************************************************************************
 * Name: up_dumpstate
 ****************************************************************************/

#ifdef CONFIG_ARCH_STACKDUMP
static void up_dumpstate(void)
{
  struct tcb_s *rtcb = running_task();
  uint32_t sp = up_getsp();
  uint32_t ustackbase;
  uint32_t ustacksize;
#if CONFIG_ARCH_INTERRUPTSTACK > 3
  uint32_t istackbase;
  uint32_t istacksize;
#endif

  /* Dump the registers (if available) */

  or1k_registerdump();

  /* Get the limits on the user stack memory */

  ustackbase = (uint32_t)rtcb->stack_base_ptr;
  ustacksize = (uint32_t)rtcb->adj_stack_size;

  /* Get the limits on the interrupt stack memory */

#if CONFIG_ARCH_INTERRUPTSTACK > 3
  istackbase = (uint32_t)g_intstackalloc;
  istacksize = (CONFIG_ARCH_INTERRUPTSTACK & ~3);

  /* Show interrupt stack info */

  _alert("sp:     %08x\n", sp);
  _alert("IRQ stack:\n");
  _alert("  base: %08x\n", istackbase);
  _alert("  size: %08x\n", istacksize);
#ifdef CONFIG_STACK_COLORATION
  _alert("  used: %08x\n", up_check_intstack());
#endif

  /* Does the current stack pointer lie within the interrupt
   * stack?
   */

  if (sp >= istackbase && sp < istackbase + istacksize)
    {
      /* Yes.. dump the interrupt stack */

      or1k_stackdump(sp, istackbase + istacksize);
    }
  else if (CURRENT_REGS)
    {
      _alert("ERROR: Stack pointer is not within the interrupt stack\n");
      or1k_stackdump(istackbase, istackbase + istacksize);
    }

  /* Extract the user stack pointer if we are in an interrupt handler.
   * If we are not in an interrupt handler.  Then sp is the user stack
   * pointer (and the above range check should have failed).
   */

  if (CURRENT_REGS)
    {
      sp = CURRENT_REGS[REG_R13];
      _alert("sp:     %08x\n", sp);
    }

  _alert("User stack:\n");
  _alert("  base: %08x\n", ustackbase);
  _alert("  size: %08x\n", ustacksize);
#ifdef CONFIG_STACK_COLORATION
  _alert("  used: %08x\n", up_check_tcbstack(rtcb));
#endif
#else
  _alert("sp:         %08x\n", sp);
  _alert("stack base: %08x\n", ustackbase);
  _alert("stack size: %08x\n", ustacksize);
#ifdef CONFIG_STACK_COLORATION
  _alert("stack used: %08x\n", up_check_tcbstack(rtcb));
#endif
#endif

  /* Dump the user stack if the stack pointer lies within the allocated user
   * stack memory.
   */

  if (sp >= ustackbase && sp < ustackbase + ustacksize)
    {
      or1k_stackdump(sp, ustackbase + ustacksize);
    }
  else
    {
      _alert("ERROR: Stack pointer is not within allocated stack\n");
      or1k_stackdump(ustackbase, ustackbase + ustacksize);
    }

  /* Dump the state of all tasks (if available) */

  up_showtasks();
}
#else
# define up_dumpstate()
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
  up_dumpstate();
}
