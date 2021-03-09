/****************************************************************************
 * arch/renesas/src/m16c/m16c_dumpstate.c
 *
 *   Copyright (C) 2009, 2016 Gregory Nutt. All rights reserved.
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
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include "up_arch.h"
#include "up_internal.h"
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

static void m16c_stackdump(uint16_t sp, uint16_t stack_base)
{
  uint16_t stack;

  for (stack = sp & ~7; stack < stack_base; stack += 8)
    {
      uint8_t *ptr = (uint8_t *)stack;
      _alert("%04x: %02x %02x %02x %02x %02x %02x %02x %02x\n",
             stack, ptr[0], ptr[1], ptr[2], ptr[3], ptr[4],
             ptr[5], ptr[6], ptr[7]);
    }
}

/****************************************************************************
 * Name: m16c_registerdump
 ****************************************************************************/

static inline void m16c_registerdump(void)
{
  uint8_t *ptr = (uint8_t *) g_current_regs;

  /* Are user registers available from interrupt processing? */

  if (ptr == NULL)
    {
      /* No.. capture user registers by hand */

      up_saveusercontext((uint32_t *)s_last_regs);
      regs = s_last_regs;
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
 * Name: up_dumpstate
 ****************************************************************************/

void up_dumpstate(void)
{
  struct tcb_s *rtcb = running_task();
  uint16_t sp = renesas_getsp();
  uint16_t ustackbase;
  uint16_t ustacksize;
#if CONFIG_ARCH_INTERRUPTSTACK > 3
  uint16_t istackbase;
  uint16_t istacksize;
#endif

  /* Dump the registers (if available) */

  m16c_registerdump();

  /* Get the limits on the user stack memory */

  ustackbase = (uint16_t)rtcb->adj_stack_ptr;
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

  if (sp <= istackbase && sp > istackbase - istacksize)
    {
      /* Yes.. dump the interrupt stack */

      m16c_stackdump(sp, istackbase);

      /* Extract the user stack pointer from the register area */

      sp = m16c_getusersp();
      _alert("sp:     %04x\n", sp);
    }
  else if (g_current_regs)
    {
      _alert("ERROR: Stack pointer is not within the interrupt stack\n");
      m16c_stackdump(istackbase - istacksize, istackbase);
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

  if (sp > ustackbase || sp <= ustackbase - ustacksize)
    {
      _alert("ERROR: Stack pointer is not within allocated stack\n");
      m16c_stackdump(ustackbase - ustacksize, ustackbase);
    }
  else
    {
      m16c_stackdump(sp, ustackbase);
    }
}

#endif /* CONFIG_ARCH_STACKDUMP */
