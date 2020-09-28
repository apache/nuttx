/****************************************************************************
 * arch/misoc/src/lm32/lm32_dumpstate.c
 *
 *  Copyright (C) 2011, 2019 Gregory Nutt. All rights reserved.
 *  Author: Gregory Nutt <gnutt@nuttx.org>
 *          Ramtin Amin <keytwo@gmail.com>
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

#include <stdint.h>
#include <stdlib.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "sched/sched.h"
#include "lm32.h"

#ifdef CONFIG_ARCH_STACKDUMP

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_stackdump
 ****************************************************************************/

static void up_stackdump(uint32_t sp, uint32_t stack_base)
{
  uint32_t stack ;

  for (stack = sp & ~0x1f; stack < stack_base; stack += 32)
    {
      uint32_t *ptr = (uint32_t *)stack;
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
      _alert("EPC:%08x \n",
            g_current_regs[REG_EPC]);
      _alert(" X0:%08x  A0:%08x  A1:%08x  A2:%08x "
             " A3:%08x  A4:%08x  A5:%08x  A6:%08x\n",
            g_current_regs[REG_X0_NDX], g_current_regs[REG_X1_NDX],
            g_current_regs[REG_X2_NDX], g_current_regs[REG_X3_NDX],
            g_current_regs[REG_X4_NDX], g_current_regs[REG_X5_NDX],
            g_current_regs[REG_X6_NDX], g_current_regs[REG_X7_NDX]);
      _alert(" A7:%08x  X9:%08x X10:%08x X11:%08x "
             "X12:%08x X13:%08x X14:%08x X15:%08x\n",
            g_current_regs[REG_X8_NDX], g_current_regs[REG_X9_NDX],
            g_current_regs[REG_X10_NDX], g_current_regs[REG_X11_NDX],
            g_current_regs[REG_X12_NDX], g_current_regs[REG_X13_NDX],
            g_current_regs[REG_X14_NDX], g_current_regs[REG_X15_NDX]);
      _alert("X16:%08x X17:%08x X18:%08x X19:%08x "
             "X20:%08x X21:%08x X22:%08x X23:%08x\n",
            g_current_regs[REG_X16_NDX], g_current_regs[REG_X17_NDX],
            g_current_regs[REG_X18_NDX], g_current_regs[REG_X19_NDX],
            g_current_regs[REG_X20_NDX], g_current_regs[REG_X21_NDX],
            g_current_regs[REG_X22_NDX], g_current_regs[REG_X23_NDX]);
      _alert("X24:%08x X25:%08x  GP:%08x  FP:%08x "
             " SP:%08x  RA:%08x  EA:%08x  BA:%08x\n",
            g_current_regs[REG_X24_NDX], g_current_regs[REG_X25_NDX],
            g_current_regs[REG_X26_NDX], g_current_regs[REG_X27_NDX],
            g_current_regs[REG_X28_NDX], g_current_regs[REG_X29_NDX],
            g_current_regs[REG_X30_NDX], g_current_regs[REG_X31_NDX]);
      _alert(" IE:%08x\n",
            g_current_regs[REG_X32_NDX]);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lm32_dumpstate
 ****************************************************************************/

void lm32_dumpstate(void)
{
  struct tcb_s *rtcb = running_task();
  uint32_t sp = misoc_getsp();
  uint32_t ustackbase;
  uint32_t ustacksize;
#if CONFIG_ARCH_INTERRUPTSTACK > 3
  uint32_t istackbase;
  uint32_t istacksize;
#endif

  /* Dump the registers (if available) */

  up_registerdump();

  /* Get the limits on the user stack memory */

  ustackbase = (uint32_t)rtcb->adj_stack_ptr;
  ustacksize = (uint32_t)rtcb->adj_stack_size;

  /* Get the limits on the interrupt stack memory */

#if CONFIG_ARCH_INTERRUPTSTACK > 3
  istackbase = (uint32_t)&g_intstackbase;
  istacksize = (CONFIG_ARCH_INTERRUPTSTACK & ~3) - 4;

  /* Show interrupt stack info */

  _alert("sp:     %08x\n", sp);
  _alert("IRQ stack:\n");
  _alert("  base: %08x\n", istackbase);
  _alert("  size: %08x\n", istacksize);

  /* Does the current stack pointer lie within the interrupt
   * stack?
   */

  if (sp <= istackbase && sp > istackbase - istacksize)
    {
      /* Yes.. dump the interrupt stack */

      up_stackdump(sp, istackbase);

      /* Extract the user stack pointer which should lie
       * at the base of the interrupt stack.
       */

      sp = g_intstackbase;
      _alert("sp:     %08x\n", sp);
    }
  else if (g_current_regs)
    {
      _alert("ERROR: Stack pointer is not within the interrupt stack\n");
      up_stackdump(istackbase - istacksize, istackbase);
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

  if (sp > ustackbase || sp <= ustackbase - ustacksize)
    {
      _alert("ERROR: Stack pointer is not within allocated stack\n");
      up_stackdump(ustackbase - ustacksize, ustackbase);
    }
  else
    {
      up_stackdump(sp, ustackbase);
    }
}

#endif /* CONFIG_ARCH_STACKDUMP */
