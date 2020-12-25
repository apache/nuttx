/****************************************************************************
 * arch/risc-v/src/nr5m100/nr5_irq.c
 *
 *   Copyright (C) 2016 Ken Pettit. All rights reserved.
 *   Author: Ken Pettit
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
#include <stdio.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <arch/irq.h>

#include "nr5.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

volatile uint32_t *g_current_regs;

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: epic_dump
 *
 * Description:
 *   Dump the EPIC priority register settings
 *
 ****************************************************************************/

void epic_dump(void)
{
  uint32_t reg;
  char     str[40];

  __asm__ volatile("csrr %0, 0x7e0" : "=r"(reg));
  sprintf(str, "IRQMASK  = 0x%08X\r", (int) reg);
  up_puts(str);
  __asm__ volatile("csrr %0, 0x7e4" : "=r"(reg));
  sprintf(str, "IRQSTACK = 0x%08X\r", (int) reg);
  up_puts(str);
  __asm__ volatile("csrr %0, 0x7e1" : "=r"(reg));
  sprintf(str, "PRI1     = 0x%08X\r", (int) reg);
  up_puts(str);
  __asm__ volatile("csrr %0, 0x7e2" : "=r"(reg));
  sprintf(str, "PRI2     = 0x%08X\r", (int) reg);
  up_puts(str);
  __asm__ volatile("csrr %0, 0x7e3" : "=r"(reg));
  sprintf(str, "PRI3     = 0x%08X\r", (int) reg);
  up_puts(str);
  __asm__ volatile("csrr %0, 0x7e5" : "=r"(reg));
  sprintf(str, "SYSTICK  = 0x%08X\r", (int) reg);
  up_puts(str);
}

/****************************************************************************
 * Name: nr5_trap
 *
 * Description:
 *   Handler for exceptions.  None are handled and all are fatal
 *   error conditions.  The only advantage these provided over the default
 *   unexpected interrupt handler is that they provide a diagnostic output.
 *
 ****************************************************************************/

#define CONFIG_DEBUG

int nr5_trap_handler(int irq, void *context, FAR void *arg)
{
  uint32_t  sp;

  /* Print a PANIC message */

  up_puts("PANIC!!! TRAP received\r\n");

#ifdef CONFIG_DEBUG

  /* restore the SP to that of the bad code */

  sp = g_current_regs[2];
  __asm__ volatile ("addi x2, %0, 0" ::"r"(sp));

  __asm__ volatile ("ebreak");
#endif
  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_irqinitialize
 ****************************************************************************/

void up_irqinitialize(void)
{
  uint32_t  mask;

  /* Disable all interrupts */

  mask = ~0;
  __asm__ volatile("csrw %0, %1" :: "i"(NR5_EPIC_IRQ_MASK), "r"(mask));

  /* Colorize the interrupt stack for debug purposes */

#if defined(CONFIG_STACK_COLORATION) && CONFIG_ARCH_INTERRUPTSTACK > 3
    {
      size_t intstack_size = (CONFIG_ARCH_INTERRUPTSTACK & ~3);
      up_stack_color((FAR void *)((uintptr_t)&g_intstackbase -
                     intstack_size), intstack_size);
    }
#endif

  /* Set the location of the vector table */

  /* Set all interrupts (and exceptions) to the default priority */

#ifdef NR5_EPIC_PRI_REG
  __asm__ volatile (" \
        csrw %0, 0(zero) \
        csrw %1, 0(zero) \
        csrw %2, 0(zero) " ::
        "i"(NR5_EPIC_PRI1_REG), "i"(NR5_EPIC_PRI2_REG),
        "i"(NR5_EPIC_PRI3_REG));
#endif

  /* Initialize the IRQ stack to Pri level 5 with interrupts disabled */

  mask = 0x05 << 2;
  __asm__ volatile("csrw %0, %1" :: "i"(NR5_EPIC_PRIMASK), "r"(mask));

  /* currents_regs is non-NULL only while processing an interrupt */

  g_current_regs = NULL;

  /* Attach the Trap exception handler.  */

  irq_attach(NR5_IRQ_TRAP, nr5_trap_handler, NULL);

  /* Attach software interrupt handler */

  irq_attach(NR5_IRQ_SOFTWARE, up_swint, NULL);
  up_enable_irq(NR5_IRQ_SOFTWARE);

  /* Set the software interrupt priority higher */

  up_setpri2bit(1 << NR5_IRQ_SOFTWARE);

#ifndef CONFIG_SUPPRESS_INTERRUPTS

  /* And finally, enable interrupts */

  up_enable_irq(NR5_IRQ_TRAP);

#endif

  /* Now enable Global Interrupts */

  __asm__ volatile("csrrs a0, %0, 3" :: "i"(NR5_EPIC_PRIMASK));
}

/****************************************************************************
 * Name: up_disable_irq
 *
 * Description:
 *   Disable the IRQ specified by 'irq'
 *
 ****************************************************************************/

void up_disable_irq(int irq)
{
  up_setirqmaskbit(1 << irq);
}

/****************************************************************************
 * Name: up_enable_irq
 *
 * Description:
 *   Enable the IRQ specified by 'irq'
 *
 ****************************************************************************/

void up_enable_irq(int irq)
{
  up_clearirqmaskbit(1 << irq);
}

/****************************************************************************
 * Name: up_ack_irq
 *
 * Description:
 *   Acknowledge the IRQ
 *
 ****************************************************************************/

void up_ack_irq(int irq)
{
}

/****************************************************************************
 * Name: up_get_newintctx
 *
 * Description:
 *   Acknowledge the IRQ
 *
 ****************************************************************************/

uint32_t up_get_newintctx(void)
{
  int32_t   regval;

  /* Set priority level 5, enabled upon return from interrupt */

  regval = ((5 << 2) | 2) << 4;

  return regval;
}

/****************************************************************************
 * Name: up_prioritize_irq
 *
 * Description:
 *   Set the priority of an IRQ.
 *
 *   Since this API is not supported on all architectures, it should be
 *   avoided in common implementations where possible.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_IRQPRIO
int up_prioritize_irq(int irq, int priority)
{
  return OK;
}
#endif

/****************************************************************************
 * Name: up_irq_save
 *
 * Description:
 *   Return the current interrupt state and disable interrupts
 *
 ****************************************************************************/

irqstate_t up_irq_save(void)
{
  irqstate_t   newpri = (2 << 2) | 3;
  irqstate_t   oldpri;

  /* Set the new IRQ Priority level to level 2, enabled.
   * This will allow SW and DEBUG / TRAP interrupts to
   * continue to fire, but no general purpose ints.
   */

  __asm__ volatile("csrrw %0, %1, %2" : "=r"(oldpri) :
                   "i"(NR5_EPIC_PRIMASK), "r"(newpri));

  return oldpri;
}

/****************************************************************************
 * Name: up_irq_restore
 *
 * Description:
 *   Restore previous IRQ mask state
 *
 ****************************************************************************/

void up_irq_restore(irqstate_t pri)
{
  __asm__ volatile("csrw %0, %1" :: "i"(NR5_EPIC_PRIMASK), "r"(pri));
}

/****************************************************************************
 * Name: up_irq_enable
 *
 * Description:
 *   Return the current interrupt state and enable interrupts
 *
 ****************************************************************************/

irqstate_t up_irq_enable(void)
{
  irqstate_t   newpri = up_get_newintctx();
  irqstate_t   oldpri;

  /* Set the new IRQ Priority level to level 5, enabled.  This will allow
   * all interrupt.
   */

  __asm__ volatile("csrrw %0, %1, %2" : "=r"(oldpri) :
                   "i"(NR5_EPIC_PRIMASK), "r"(newpri));

  return oldpri;
}
