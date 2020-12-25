/****************************************************************************
 * arch/avr/src/avr/up_dumpstate.c
 *
 *   Copyright (C) 2011, 2014, 2016 Gregory Nutt. All rights reserved.
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

#include <stdint.h>
#include <stdlib.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "up_arch.h"
#include "sched/sched.h"
#include "up_internal.h"

#ifdef CONFIG_ARCH_STACKDUMP

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_stackdump
 ****************************************************************************/

static void up_stackdump(uint16_t sp, uint16_t stack_base)
{
  uint16_t stack ;

  for (stack = sp & ~3; stack < stack_base; stack += 12)
    {
      uint8_t *ptr = (uint8_t *)stack;
      _alert("%04x: %02x %02x %02x %02x %02x %02x %02x %02x"
            " %02x %02x %02x %02x\n",
             stack,
             ptr[0], ptr[1], ptr[2], ptr[3], ptr[4], ptr[5], ptr[6], ptr[7],
             ptr[8], ptr[9], ptr[10], ptr[11]);
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
      _alert("R%02d: %02x %02x %02x %02x %02x %02x %02x %02x\n",
            0,
            g_current_regs[REG_R0],  g_current_regs[REG_R1],
            g_current_regs[REG_R2],  g_current_regs[REG_R3],
            g_current_regs[REG_R4],  g_current_regs[REG_R5],
            g_current_regs[REG_R6],  g_current_regs[REG_R7]);

      _alert("R%02d: %02x %02x %02x %02x %02x %02x %02x %02x\n",
            8,
            g_current_regs[REG_R8],  g_current_regs[REG_R9],
            g_current_regs[REG_R10], g_current_regs[REG_R11],
            g_current_regs[REG_R12], g_current_regs[REG_R13],
            g_current_regs[REG_R14], g_current_regs[REG_R15]);

      _alert("R%02d: %02x %02x %02x %02x %02x %02x %02x %02x\n",
            16,
            g_current_regs[REG_R16], g_current_regs[REG_R17],
            g_current_regs[REG_R18], g_current_regs[REG_R19],
            g_current_regs[REG_R20], g_current_regs[REG_R21],
            g_current_regs[REG_R22], g_current_regs[REG_R23]);

      _alert("R%02d: %02x %02x %02x %02x %02x %02x %02x %02x\n",
            24,
            g_current_regs[REG_R24], g_current_regs[REG_R25],
            g_current_regs[REG_R26], g_current_regs[REG_R27],
            g_current_regs[REG_R28], g_current_regs[REG_R29],
            g_current_regs[REG_R30], g_current_regs[REG_R31]);

#if !defined(REG_PC2)
      _alert("PC:  %02x%02x  SP: %02x%02x SREG: %02x\n",
            g_current_regs[REG_PC0], g_current_regs[REG_PC1],
            g_current_regs[REG_SPH], g_current_regs[REG_SPL],
            g_current_regs[REG_SREG]);
#else
      _alert("PC:  %02x%02x%02x  SP: %02x%02x SREG: %02x\n",
            g_current_regs[REG_PC0], g_current_regs[REG_PC1],
            g_current_regs[REG_PC2], g_current_regs[REG_SPH],
            g_current_regs[REG_SPL], g_current_regs[REG_SREG]);
#endif
    }
}

/****************************************************************************
 * Name: _up_assert
 ****************************************************************************/

/****************************************************************************
 * Name: up_dumpstate
 ****************************************************************************/

void up_dumpstate(void)
{
  struct tcb_s *rtcb = running_task();
  uint16_t sp = avr_getsp();
  uint16_t ustackbase;
  uint16_t ustacksize;
#if CONFIG_ARCH_INTERRUPTSTACK > 0
  uint16_t istackbase;
  uint16_t istacksize;
#endif

  /* Dump the registers (if available) */

  up_registerdump();

  /* Get the limits on the user stack memory */

  ustackbase = (uint16_t)rtcb->adj_stack_ptr;
  ustacksize = (uint16_t)rtcb->adj_stack_size;

  /* Get the limits on the interrupt stack memory */

#if CONFIG_ARCH_INTERRUPTSTACK > 0
  istackbase = (uint16_t)&g_intstackbase - 1;
  istacksize = CONFIG_ARCH_INTERRUPTSTACK;

  /* Show interrupt stack info */

  _alert("sp:     %04x\n", sp);
  _alert("IRQ stack:\n");
  _alert("  base: %04x\n", istackbase);
  _alert("  size: %04x\n", istacksize);
#ifdef CONFIG_STACK_COLORATION
  _alert("  used: %08x\n", up_check_intstack());
#endif

  /* Does the current stack pointer lie within the interrupt
   * stack?
   */

  if (sp <= istackbase && sp > istackbase - istacksize)
    {
      /* Yes.. dump the interrupt stack */

      up_stackdump(sp, istackbase);
    }
  else if (g_current_regs)
    {
      _alert("ERROR: Stack pointer is not within the interrupt stack\n");
      up_stackdump(istackbase - istacksize, istackbase);
    }

  /* Extract the user stack pointer if we are in an interrupt handler.
   * If we are not in an interrupt handler.  Then sp is the user stack
   * pointer (and the above range check should have failed).
   */

  if (g_current_regs)
    {
      sp = g_current_regs[REG_R13];
      _alert("sp:     %04x\n", sp);
    }

  _alert("User stack:\n");
  _alert("  base: %04x\n", ustackbase);
  _alert("  size: %04x\n", ustacksize);
#ifdef CONFIG_STACK_COLORATION
  _alert("  used: %08x\n", up_check_tcbstack(rtcb));
#endif

  /* Dump the user stack if the stack pointer lies within the allocated user
   * stack memory.
   */

  if (sp <= ustackbase && sp > ustackbase - ustacksize)
    {
      up_stackdump(sp, ustackbase);
    }
  else
    {
      _alert("ERROR: Stack pointer is not within allocated stack\n");
      up_stackdump(ustackbase - ustacksize, ustackbase);
    }
#else
  _alert("sp:         %04x\n", sp);
  _alert("stack base: %04x\n", ustackbase);
  _alert("stack size: %04x\n", ustacksize);
#ifdef CONFIG_STACK_COLORATION
  _alert("stack used: %08x\n", up_check_tcbstack(rtcb));
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
#endif
}

#endif /* CONFIG_ARCH_STACKDUMP */
