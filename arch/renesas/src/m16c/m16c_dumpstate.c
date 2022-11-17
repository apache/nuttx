/****************************************************************************
 * arch/renesas/src/m16c/m16c_dumpstate.c
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
#include <nuttx/syslog/syslog.h>

#include "renesas_internal.h"
#include "sched/sched.h"
#include "chip.h"

#ifdef CONFIG_ARCH_STACKDUMP

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint8_t s_last_regs[XCPTCONTEXT_REGS];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: m16c_getusersp
 ****************************************************************************/

#if CONFIG_ARCH_INTERRUPTSTACK > 3
static inline uint16_t m16c_getusersp(void)
{
  uint8_t *ptr = (uint8_t *) g_current_regs;
  return (uint16_t)ptr[REG_SP] << 8 | ptr[REG_SP + 1];
}
#endif

/****************************************************************************
 * Name: m16c_stackdump
 ****************************************************************************/

static void m16c_stackdump(uint16_t sp, uint16_t stack_top)
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
 * Name: m16c_registerdump
 ****************************************************************************/

static inline void m16c_registerdump(void)
{
  volatile uint8_t *ptr = (uint8_t *)g_current_regs;

  /* Are user registers available from interrupt processing? */

  if (ptr == NULL)
    {
      /* No.. capture user registers by hand */

      up_saveusercontext((uint32_t *)s_last_regs);
      ptr = s_last_regs;
    }

  /* Dump the interrupt registers */

  _alert("PC: %02x%02x%02x FLG: %02x00%02x FB: %02x%02x SB: %02x%02x "
         "SP: %02x%02x\n",
         ptr[REG_FLGPCHI] & 0xff, ptr[REG_PC], ptr[REG_PC + 1],
         ptr[REG_FLGPCHI] >> 8, ptr[REG_FLG],
         ptr[REG_FB], ptr[REG_FB + 1],
         ptr[REG_SB], ptr[REG_SB + 1],
         ptr[REG_SP], ptr[REG_SP + 1]);

  _alert("R0: %02x%02x R1: %02x%02x R2: %02x%02x A0: %02x%02x "
         "A1: %02x%02x\n",
         ptr[REG_R0], ptr[REG_R0 + 1], ptr[REG_R1], ptr[REG_R1 + 1],
         ptr[REG_R2], ptr[REG_R2 + 1], ptr[REG_R3], ptr[REG_R3 + 1],
         ptr[REG_A0], ptr[REG_A0 + 1], ptr[REG_A1], ptr[REG_A1 + 1]);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: renesas_dumpstate
 ****************************************************************************/

void renesas_dumpstate(void)
{
  FAR struct tcb_s *rtcb = running_task();
  uint16_t sp = up_getsp();
  uint16_t ustackbase;
  uint16_t ustacksize;
#if CONFIG_ARCH_INTERRUPTSTACK > 3
  uint16_t istackbase;
  uint16_t istacksize;
#endif

  /* Dump the registers (if available) */

  m16c_registerdump();

  /* Get the limits on the user stack memory */

  ustackbase = (uint16_t)rtcb->stack_base_ptr;
  ustacksize = (uint16_t)rtcb->adj_stack_size;

  /* Get the limits on the interrupt stack memory.
   * The near RAM memory map is as follows:
   *
   * 0x00400 - DATA   Size: Determined by linker
   *           BSS    Size: Determined by linker
   *           Interrupt stack Size: CONFIG_ARCH_INTERRUPTSTACK
   *           Idle stack  Size: CONFIG_IDLETHREAD_STACKSIZE
   *           Heap        Size: Everything remaining
   * 0x00bff - (end+1)
   */

#if CONFIG_ARCH_INTERRUPTSTACK > 3
  istackbase = g_enbss;
  istacksize = CONFIG_ARCH_INTERRUPTSTACK;

  /* Show interrupt stack info */

  _alert("sp:     %04x\n", sp);
  _alert("IRQ stack:\n");
  _alert("  base: %04x\n", istackbase);
  _alert("  size: %04x\n", istacksize);

  /* Does the current stack pointer lie within the interrupt
   * stack?
   */

  if (sp >= istackbase && sp < istackbase + istacksize)
    {
      /* Yes.. dump the interrupt stack */

      m16c_stackdump(sp, istackbase + istacksize);

      /* Extract the user stack pointer from the register area */

      sp = m16c_getusersp();
      _alert("sp:     %04x\n", sp);
    }
  else if (g_current_regs)
    {
      _alert("ERROR: Stack pointer is not within the interrupt stack\n");
      m16c_stackdump(istackbase, istackbase + istacksize);
    }

  /* Show user stack info */

  _alert("User stack:\n");
  _alert("  base: %04x\n", ustackbase);
  _alert("  size: %04x\n", ustacksize);
#else
  _alert("sp:         %04x\n", sp);
  _alert("stack base: %04x\n", ustackbase);
  _alert("stack size: %04x\n", ustacksize);
#endif

  /* Dump the user stack if the stack pointer lies within the allocated user
   * stack memory.
   */

  if (sp >= ustackbase && sp < ustackbase + ustacksize)
    {
      m16c_stackdump(sp, ustackbase + ustacksize);
    }
  else
    {
      _alert("ERROR: Stack pointer is not within allocated stack\n");
      m16c_stackdump(ustackbase, ustackbase + ustacksize);
    }
}

#endif /* CONFIG_ARCH_STACKDUMP */
