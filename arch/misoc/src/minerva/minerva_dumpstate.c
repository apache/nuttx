/****************************************************************************
 * arch/misoc/src/minerva/minerva_dumpstate.c
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
#include <stdlib.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "sched/sched.h"
#include "minerva.h"

#ifdef CONFIG_ARCH_STACKDUMP

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_stackdump
 ****************************************************************************/

static void up_stackdump(uint32_t sp, uint32_t stack_top)
{
  uint32_t stack;

  for (stack = sp & ~0x1f; stack < (stack_top & ~0x1f); stack += 32)
    {
      uint32_t *ptr = (uint32_t *) stack;
      _alert("%08x: %08x %08x %08x %08x %08x %08x %08x %08x\n",
             stack, ptr[0], ptr[1], ptr[2], ptr[3],
             ptr[4], ptr[5], ptr[6], ptr[7]);
    }
}

/****************************************************************************
 * Name: up_registerdump
 ****************************************************************************/

static inline void up_registerdump(void)
{
  /* Are user registers available from interrupt processing? */

  if (g_current_regs)
    {
      _alert("EPC:%08x \n", g_current_regs[REG_CSR_MEPC]);
      _alert
        (" X0:%08x  A0:%08x  A1:%08x  A2:%08x "
         " A3:%08x  A4:%08x  A5:%08x  A6:%08x\n",
         g_current_regs[REG_X0_NDX], g_current_regs[REG_X1_NDX],
         g_current_regs[REG_X2_NDX], g_current_regs[REG_X3_NDX],
         g_current_regs[REG_X4_NDX], g_current_regs[REG_X5_NDX],
         g_current_regs[REG_X6_NDX], g_current_regs[REG_X7_NDX]);
      _alert
        (" A7:%08x  X9:%08x X10:%08x X11:%08x "
         "X12:%08x X13:%08x X14:%08x X15:%08x\n",
         g_current_regs[REG_X8_NDX], g_current_regs[REG_X9_NDX],
         g_current_regs[REG_X10_NDX], g_current_regs[REG_X11_NDX],
         g_current_regs[REG_X12_NDX], g_current_regs[REG_X13_NDX],
         g_current_regs[REG_X14_NDX], g_current_regs[REG_X15_NDX]);
      _alert
        ("X16:%08x X17:%08x X18:%08x X19:%08x "
         "X20:%08x X21:%08x X22:%08x X23:%08x\n",
         g_current_regs[REG_X16_NDX], g_current_regs[REG_X17_NDX],
         g_current_regs[REG_X18_NDX], g_current_regs[REG_X19_NDX],
         g_current_regs[REG_X20_NDX], g_current_regs[REG_X21_NDX],
         g_current_regs[REG_X22_NDX], g_current_regs[REG_X23_NDX]);
      _alert
        ("X24:%08x X25:%08x  GP:%08x  FP:%08x "
         " SP:%08x  RA:%08x  EA:%08x  BA:%08x\n",
         g_current_regs[REG_X24_NDX], g_current_regs[REG_X25_NDX],
         g_current_regs[REG_X26_NDX], g_current_regs[REG_X27_NDX],
         g_current_regs[REG_X28_NDX], g_current_regs[REG_X29_NDX],
         g_current_regs[REG_X30_NDX], g_current_regs[REG_X31_NDX]);
      _alert(" IE:%08x\n", g_current_regs[REG_CSR_MSTATUS]);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: minerva_dumpstate
 ****************************************************************************/

void minerva_dumpstate(void)
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

  up_registerdump();

  /* Get the limits on the user stack memory NOTE: You cannot use the PID to
   * determine if this is an IDLE task.  In the SMP case, there may be
   * multiple IDLE tasks with different PIDs.  The only consistent way to
   * test for the IDLE task is to check it is at the end of the list (flink
   * == NULL)
   */

  ustackbase = (uint32_t) rtcb->stack_base_ptr;
  ustacksize = (uint32_t) rtcb->adj_stack_size;

  /* Get the limits on the interrupt stack memory */

#if CONFIG_ARCH_INTERRUPTSTACK > 3
  istackbase = (uint32_t) &g_intstackalloc;
  istacksize = (CONFIG_ARCH_INTERRUPTSTACK & ~3);

  /* Show interrupt stack info */

  _alert("sp:     %08x\n", sp);
  _alert("IRQ stack:\n");
  _alert("  base: %08x\n", istackbase);
  _alert("  size: %08x\n", istacksize);

  /* Does the current stack pointer lie within the interrupt stack? */

  if (sp >= istackbase && sp < istackbase + istacksize)
    {
      /* Yes.. dump the interrupt stack */

      up_stackdump(sp, istackbase + istacksize);

      /* Extract the user stack pointer which should lie at the base of the
       * interrupt stack.
       */

      sp = g_intstacktop;
      _alert("sp:     %08x\n", sp);
    }
  else if (g_current_regs)
    {
      _alert("ERROR: Stack pointer is not within the interrupt stack\n");
      up_stackdump(istackbase, istackbase + istacksize);
    }

  /* Show user stack info */

  _alert("User stack:\n");
  _alert("  base: %08x\n", ustackbase);
  _alert("  size: %08x\n", ustacksize);
#else
  _alert("sp:         %08x\n", sp);
  _alert("stack base: %08x\n", ustackbase);
  _alert("stack size: %08x\n", ustacksize);
#endif

  /* Dump the user stack if the stack pointer lies within the allocated user
   * stack memory.
   */

  if (sp >= ustackbase && sp < ustackbase + ustacksize)
    {
      up_stackdump(sp, ustackbase + ustacksize);
    }
  else
    {
      _alert("ERROR: Stack pointer is not within allocated stack\n");
      up_stackdump(ustackbase, ustackbase + ustacksize);
    }
}

#endif /* CONFIG_ARCH_STACKDUMP */
