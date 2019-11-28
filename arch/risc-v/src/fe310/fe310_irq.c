/****************************************************************************
 * arch/risc-v/src/fe310/fe310_irq.c
 *
 *   Copyright (C) 2019 Masayuki Ishikawa. All rights reserved.
 *   Author: Masayuki Ishikawa <masayuki.ishikawa@gmail.com>
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

#include "up_internal.h"
#include "up_arch.h"

#include "fe310.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

volatile uint32_t *g_current_regs;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_irqinitialize
 ****************************************************************************/

void up_irqinitialize(void)
{
  /* Disable Machine interrupts */

  (void)up_irq_save();

  /* Colorize the interrupt stack for debug purposes */

#if defined(CONFIG_STACK_COLORATION) && CONFIG_ARCH_INTERRUPTSTACK > 3
  size_t intstack_size = (CONFIG_ARCH_INTERRUPTSTACK & ~3);
  up_stack_color((FAR void *)((uintptr_t)&g_intstackbase - intstack_size),
                 intstack_size);
#endif

  /* Set priority for all global interrupts to 1 (lowest) */

  int id;

  for (id = 1; id <= 52; id++)
    {
      putreg32(1, FE310_PLIC_PRIORITY + 4 * id);
    }

  /* Set irq threshold to 0 (permits all global interrupts) */

  putreg32(0, FE310_PLIC_THRESHOLD);

  /* currents_regs is non-NULL only while processing an interrupt */

  g_current_regs = NULL;

  /* Attach the ecall interrupt handler */

  irq_attach(FE310_IRQ_ECALLM, up_swint, NULL);
  up_enable_irq(FE310_IRQ_ECALLM);

#ifndef CONFIG_SUPPRESS_INTERRUPTS

  /* And finally, enable interrupts */

  (void)up_irq_enable();
#endif
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
  int extirq;

  if (irq > FE310_IRQ_MEXT)
    {
      extirq = irq - FE310_IRQ_MEXT;
      ASSERT(31 >= extirq); /* TODO */

      /* Clear enable bit for the irq */

      modifyreg32(FE310_PLIC_ENABLE1, 1 << extirq, 0);
    }
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
  int extirq;

  if (irq > FE310_IRQ_MEXT)
    {
      extirq = irq - FE310_IRQ_MEXT;
      ASSERT(31 >= extirq); /* TODO */

      /* Set enable bit for the irq */

      modifyreg32(FE310_PLIC_ENABLE1, 0, 1 << extirq);
    }
}

/****************************************************************************
 * Name: up_get_newintctx
 *
 * Description:
 *   Return a value for EPIC. But FE310 doesn't use EPIC for event control.
 *
 ****************************************************************************/

uint32_t up_get_newintctx(void)
{
  return 0;
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
 * Name: up_irq_save
 *
 * Description:
 *   Return the current interrupt state and disable interrupts
 *
 ****************************************************************************/

irqstate_t up_irq_save(void)
{
  uint32_t oldstat;
  uint32_t newstat;

  /* Clear machine interrupt enable bit */

  asm volatile ("csrr %0, mstatus": "=r" (oldstat));
  newstat = oldstat & ~MSTATUS_MIE;
  asm volatile("csrw mstatus, %0" : /* no output */ : "r" (newstat));

  return oldstat;
}

/****************************************************************************
 * Name: up_irq_restore
 *
 * Description:
 *   Restore previous IRQ mask state
 *
 ****************************************************************************/

void up_irq_restore(irqstate_t flags)
{
  /* Machine mode - mstatus */

  asm volatile("csrw mstatus, %0" : /* no output */ : "r" (flags));
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
  uint32_t oldstat;
  uint32_t newstat;
  uint32_t mie;

#if 1
  /* Enable MEIE (machine external interrupt enable)
   * and MTIE (machine timer interrupt enable)
   */

  /* TODO: should move to up_enable_irq() */

  mie = 0x1 << 11 | 0x1 << 7;
  asm volatile("csrw mie, %0" : /* no output */ : "r" (mie));
#endif

  /* Set machine interrupt enable (MIE) in mstatus */

  asm volatile ("csrr %0, mstatus": "=r" (oldstat));
  newstat = oldstat | MSTATUS_MIE;
  asm volatile("csrw mstatus, %0" : /* no output */ : "r" (newstat));

  return oldstat;
}
