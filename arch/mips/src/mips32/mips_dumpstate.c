/****************************************************************************
 * arch/mips/src/mips32/mips_dumpstate.c
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

#include <inttypes.h>
#include <stdint.h>
#include <stdlib.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/syslog/syslog.h>
#include <arch/board/board.h>

#include "sched/sched.h"
#include "mips_internal.h"

#ifdef CONFIG_ARCH_STACKDUMP

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mips_stackdump
 ****************************************************************************/

static void mips_stackdump(uint32_t sp, uint32_t stack_top)
{
  uint32_t stack;

  /* Flush any buffered SYSLOG data to avoid overwrite */

  syslog_flush();

  for (stack = sp & ~0x1f; stack < (stack_top & ~0x1f); stack += 32)
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
 * Name: mips_registerdump
 ****************************************************************************/

static inline void mips_registerdump(void)
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
  uint32_t sp = up_getsp();
  uint32_t ustackbase;
  uint32_t ustacksize;
#if CONFIG_ARCH_INTERRUPTSTACK > 3
  uint32_t istackbase;
  uint32_t istacksize;
#endif

  /* Dump the registers (if available) */

  mips_registerdump();

  /* Get the limits on the user stack memory */

  ustackbase = (uint32_t)rtcb->stack_base_ptr;
  ustacksize = (uint32_t)rtcb->adj_stack_size;

  /* Get the limits on the interrupt stack memory */

#if CONFIG_ARCH_INTERRUPTSTACK > 3
  istackbase = (uint32_t)g_intstackalloc;
  istacksize = (CONFIG_ARCH_INTERRUPTSTACK & ~3);

  /* Show interrupt stack info */

  _alert("sp:     %08" PRIx32 "\n", sp);
  _alert("IRQ stack:\n");
  _alert("  base: %08" PRIx32 "\n", istackbase);
  _alert("  size: %08" PRIx32 "\n", istacksize);

  /* Does the current stack pointer lie within the interrupt
   * stack?
   */

  if (sp >= istackbase && sp < istackbase + istacksize)
    {
      /* Yes.. dump the interrupt stack */

      mips_stackdump(sp, istackbase + istacksize);

      /* Extract the user stack pointer which should lie
       * at the base of the interrupt stack.
       */

      sp = g_intstacktop;
      _alert("sp:     %08" PRIx32 "\n", sp);
    }
  else if (CURRENT_REGS)
    {
      _alert("ERROR: Stack pointer is not within the interrupt stack\n");
      mips_stackdump(istackbase, istackbase + istacksize);
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

  if (sp >= ustackbase && sp < ustackbase + ustacksize)
    {
      mips_stackdump(sp, ustackbase + ustacksize);
    }
  else
    {
      _alert("ERROR: Stack pointer is not within allocated stack\n");
      mips_stackdump(ustackbase, ustackbase + ustacksize);
    }
}

#endif /* CONFIG_ARCH_STACKDUMP */
