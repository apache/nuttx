/****************************************************************************
 * arch/mips/src/mips32/mips_dumpstate.c
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
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

#include <inttypes.h>
#include <stdint.h>
#include <stdlib.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "mips_arch.h"
#include "sched/sched.h"
#include "mips_internal.h"

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
      _alert("%08" PRIx32 ": %08" PRIx32 " %08" PRIx32
             " %08" PRIx32 " %08" PRIx32 " %08" PRIx32
             " %08" PRIx32 " %08" PRIx32 " %08" PRIx32 "\n",
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

  if (CURRENT_REGS)
    {
      _alert("MFLO:%08" PRIx32 " MFHI:%08" PRIx32
             " EPC:%08" PRIx32 " STATUS:%08" PRIx32 "\n",
             CURRENT_REGS[REG_MFLO], CURRENT_REGS[REG_MFHI],
             CURRENT_REGS[REG_EPC], CURRENT_REGS[REG_STATUS]);
      _alert("AT:%08" PRIx32 " V0:%08" PRIx32 " V1:%08" PRIx32
             " A0:%08" PRIx32 " A1:%08" PRIx32 " A2:%08" PRIx32
             " A3:%08" PRIx32 "\n",
             CURRENT_REGS[REG_AT], CURRENT_REGS[REG_V0],
             CURRENT_REGS[REG_V1], CURRENT_REGS[REG_A0],
             CURRENT_REGS[REG_A1], CURRENT_REGS[REG_A2],
             CURRENT_REGS[REG_A3]);
      _alert("T0:%08" PRIx32 " T1:%08" PRIx32 " T2:%08" PRIx32
             " T3:%08" PRIx32 " T4:%08" PRIx32 " T5:%08" PRIx32
             " T6:%08" PRIx32 " T7:%08" PRIx32 "\n",
             CURRENT_REGS[REG_T0], CURRENT_REGS[REG_T1],
             CURRENT_REGS[REG_T2], CURRENT_REGS[REG_T3],
             CURRENT_REGS[REG_T4], CURRENT_REGS[REG_T5],
             CURRENT_REGS[REG_T6], CURRENT_REGS[REG_T7]);
      _alert("S0:%08" PRIx32 " S1:%08" PRIx32 " S2:%08" PRIx32
             " S3:%08" PRIx32 " S4:%08" PRIx32 " S5:%08" PRIx32
             " S6:%08" PRIx32 " S7:%08" PRIx32 "\n",
             CURRENT_REGS[REG_S0], CURRENT_REGS[REG_S1],
             CURRENT_REGS[REG_S2], CURRENT_REGS[REG_S3],
             CURRENT_REGS[REG_S4], CURRENT_REGS[REG_S5],
             CURRENT_REGS[REG_S6], CURRENT_REGS[REG_S7]);
#ifdef MIPS32_SAVE_GP
      _alert("T8:%08" PRIx32 " T9:%08" PRIx32 " GP:%08" PRIx32
             " SP:%08" PRIx32 " FP:%08" PRIx32 " RA:%08" PRIx32 "\n",
             CURRENT_REGS[REG_T8], CURRENT_REGS[REG_T9],
             CURRENT_REGS[REG_GP], CURRENT_REGS[REG_SP],
             CURRENT_REGS[REG_FP], CURRENT_REGS[REG_RA]);
#else
      _alert("T8:%08" PRIx32 " T9:%08" PRIx32 " SP:%08" PRIx32
             " FP:%08" PRIx32 " RA:%08" PRIx32 "\n",
             CURRENT_REGS[REG_T8], CURRENT_REGS[REG_T9],
             CURRENT_REGS[REG_SP], CURRENT_REGS[REG_FP],
             CURRENT_REGS[REG_RA]);
#endif
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_dumpstate
 ****************************************************************************/

void up_dumpstate(void)
{
  struct tcb_s *rtcb = running_task();
  uint32_t sp = mips_getsp();
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

  _alert("sp:     %08" PRIx32 "\n", sp);
  _alert("IRQ stack:\n");
  _alert("  base: %08" PRIx32 "\n", istackbase);
  _alert("  size: %08" PRIx32 "\n", istacksize);

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
      _alert("sp:     %08" PRIx32 "\n", sp);
    }
  else if (CURRENT_REGS)
    {
      _alert("ERROR: Stack pointer is not within the interrupt stack\n");
      up_stackdump(istackbase - istacksize, istackbase);
    }

  /* Show user stack info */

  _alert("User stack:\n");
  _alert("  base: %08" PRIx32 "\n", ustackbase);
  _alert("  size: %08" PRIx32 "\n", ustacksize);
#else
  _alert("sp:         %08" PRIx32 "\n", sp);
  _alert("stack base: %08" PRIx32 "\n", ustackbase);
  _alert("stack size: %08" PRIx32 "\n", ustacksize);
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
