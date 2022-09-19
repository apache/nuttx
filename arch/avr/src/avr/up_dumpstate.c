/****************************************************************************
 * arch/avr/src/avr/up_dumpstate.c
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
#include <nuttx/syslog/syslog.h>
#include <arch/board/board.h>

#include "sched/sched.h"
#include "up_internal.h"

#ifdef CONFIG_ARCH_STACKDUMP

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: avr_stackdump
 ****************************************************************************/

static void avr_stackdump(uint16_t sp, uint16_t stack_top)
{
  uint16_t stack;

  /* Flush any buffered SYSLOG data to avoid overwrite */

  syslog_flush();

  for (stack = sp & ~0x7; stack < (stack_top & ~0x7); stack += 8)
    {
      uint8_t *ptr = (uint8_t *)stack;
      _alert("%04x: %02x %02x %02x %02x %02x %02x %02x %02x\n",
             stack, ptr[0], ptr[1], ptr[2], ptr[3],
             ptr[4], ptr[5], ptr[6], ptr[7]);
    }
}

/****************************************************************************
 * Name: avr_registerdump
 ****************************************************************************/

static inline void avr_registerdump(void)
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
  FAR struct tcb_s *rtcb = running_task();
  uint16_t sp = up_getsp();
  uint16_t ustackbase;
  uint16_t ustacksize;
#if CONFIG_ARCH_INTERRUPTSTACK > 0
  uint16_t istackbase;
  uint16_t istacksize;
#endif

  /* Dump the registers (if available) */

  avr_registerdump();

  /* Get the limits on the user stack memory */

  ustackbase = (uint16_t)rtcb->stack_base_ptr;
  ustacksize = (uint16_t)rtcb->adj_stack_size;

  /* Get the limits on the interrupt stack memory */

#if CONFIG_ARCH_INTERRUPTSTACK > 0
  istackbase = (uint16_t)g_intstackalloc;
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

  if (sp >= istackbase && sp < istackbase + istacksize)
    {
      /* Yes.. dump the interrupt stack */

      avr_stackdump(sp, istackbase + istacksize);
    }
  else if (g_current_regs)
    {
      _alert("ERROR: Stack pointer is not within the interrupt stack\n");
      avr_stackdump(istackbase, istackbase + istacksize);
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
#else
  _alert("sp:         %04x\n", sp);
  _alert("stack base: %04x\n", ustackbase);
  _alert("stack size: %04x\n", ustacksize);
#ifdef CONFIG_STACK_COLORATION
  _alert("stack used: %08x\n", up_check_tcbstack(rtcb));
#endif
#endif

  /* Dump the user stack if the stack pointer lies within the allocated user
   * stack memory.
   */

  if (sp >= ustackbase && sp < ustackbase + ustacksize)
    {
      avr_stackdump(sp, ustackbase + ustacksize);
    }
  else
    {
      _alert("ERROR: Stack pointer is not within allocated stack\n");
      avr_stackdump(ustackbase, ustackbase + ustacksize);
    }
}

#endif /* CONFIG_ARCH_STACKDUMP */
