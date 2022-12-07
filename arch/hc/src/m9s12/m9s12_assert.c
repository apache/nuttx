/****************************************************************************
 * arch/hc/src/m9s12/m9s12_assert.c
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
#include "hc_internal.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_ARCH_STACKDUMP
static uint8_t s_last_regs[XCPTCONTEXT_REGS];
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: hc_stackdump
 ****************************************************************************/

#ifdef CONFIG_ARCH_STACKDUMP
static void hc_stackdump(uint16_t sp, uint16_t stack_top)
{
  uint16_t stack;

  for (stack = sp & ~0x7; stack < (stack_top & ~0x7); stack += 8)
    {
      uint8_t *ptr = (uint8_t *)stack;
      _alert("%04x: %02x %02x %02x %02x %02x %02x %02x %02x\n",
             stack, ptr[0], ptr[1], ptr[2], ptr[3],
             ptr[4], ptr[5], ptr[6], ptr[7]);
    }
}
#endif

/****************************************************************************
 * Name: hc_registerdump
 ****************************************************************************/

#ifdef CONFIG_ARCH_STACKDUMP
static inline void hc_registerdump(void)
{
  volatile uint8_t *regs = g_current_regs;

  /* Are user registers available from interrupt processing? */

  if (regs == NULL)
    {
      /* No.. capture user registers by hand */

      up_saveusercontext(s_last_regs);
      regs = s_last_regs;
    }

  _alert("A:%02x B:%02x X:%02x%02x Y:%02x%02x PC:%02x%02x CCR:%02x\n",
         regs[REG_A],  regs[REG_B],  regs[REG_XH],  regs[REG_XL],
         regs[REG_YH], regs[REG_YL], regs[REG_PCH], regs[REG_PCL],
         regs[REG_CCR]);
  _alert("SP:%02x%02x FRAME:%02x%02x TMP:%02x%02x Z:%02x%02x XY:%02x\n",
         regs[REG_SPH],  regs[REG_SPL],  regs[REG_FRAMEH], regs[REG_FRAMEL],
         regs[REG_TMPL], regs[REG_TMPH], regs[REG_ZL],     regs[REG_ZH],
         regs[REG_XY],   regs[REG_XY + 1]);

#if CONFIG_HCS12_MSOFTREGS > 2
#  error "Need to save more registers"
#elif CONFIG_HCS12_MSOFTREGS == 2
  _alert("SOFTREGS: %02x%02x :%02x%02x\n",
         regs[REG_SOFTREG1], regs[REG_SOFTREG1 + 1],
         regs[REG_SOFTREG2], regs[REG_SOFTREG2 + 1]);
#elif CONFIG_HCS12_MSOFTREGS == 1
  _alert("SOFTREGS: %02x%02x\n",
         regs[REG_SOFTREG1], regs[REG_SOFTREG1 + 1]);
#endif

#ifndef CONFIG_HCS12_NONBANKED
  _alert("PPAGE: %02x\n", regs[REG_PPAGE]);
#endif
}
#endif

/****************************************************************************
 * Name: up_dumpstate
 ****************************************************************************/

#ifdef CONFIG_ARCH_STACKDUMP
static void up_dumpstate(void)
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

  hc_registerdump();

  /* Get the limits on the user stack memory */

  ustackbase = (uint16_t)rtcb->stack_base_ptr;
  ustacksize = (uint16_t)rtcb->adj_stack_size;

  /* Get the limits on the interrupt stack memory */

#if CONFIG_ARCH_INTERRUPTSTACK > 3
  istackbase = (uint16_t)g_intstackalloc;
  istacksize = (CONFIG_ARCH_INTERRUPTSTACK & ~3);

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

      hc_stackdump(sp, istackbase + istacksize);

      /* Extract the user stack pointer which should lie
       * at the base of the interrupt stack.
       */

      sp = g_intstacktop;
      _alert("sp:     %04x\n", sp);
    }
  else if (g_current_regs)
    {
      _alert("ERROR: Stack pointer is not within the interrupt stack\n");
      hc_stackdump(istackbase, istackbase + istacksize);
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
      hc_stackdump(sp, ustackbase + ustacksize);
    }
  else
    {
      _alert("ERROR: Stack pointer is not within allocated stack\n");
      hc_stackdump(ustackbase, ustackbase + ustacksize);
    }
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
