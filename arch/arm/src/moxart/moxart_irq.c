/****************************************************************************
 * arch/arm/src/moxart/moxart_irq.c
 * Driver for MoxaRT IRQ controller
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
 *   Author: Anton D. Kachalov <mouse@mayc.ru>
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

#include <stdio.h>
#include <stdint.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "arm.h"
#include "arm_arch.h"
#include "arm_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IRQ_ADDR    0x98800000
#define IRQ_REG(x)  (IRQ_ADDR + x)

#define IRQ__SRC    0x00
#define IRQ__MASK   0x04
#define IRQ__CLEAR  0x08
#define IRQ__MODE   0x0C
#define IRQ__LEVEL  0x10
#define IRQ__STATUS 0x14

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* g_current_regs[] holds a references to the current interrupt level
 * register storage structure.  If is non-NULL only during interrupt
 * processing.  Access to g_current_regs[] must be through the macro
 * CURRENT_REGS for portability.
 */

volatile uint32_t *g_current_regs[1];

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

extern void uart_decodeirq(int irq, uint32_t *regs);

/****************************************************************************
 * Name: up_irqinitialize
 *
 * Description:
 *   Setup the IRQ and FIQ controllers
 *
 ****************************************************************************/

void up_irqinitialize(void)
{
  /* Prepare hardware */

  *(volatile int *)0x98700000 |= 0x3f;

  /* PMU setup */

  (*(volatile uint32_t *)0x98100008) &= ~0x9;

  while (!((*(volatile uint32_t *)0x98100008) & 0x2))
    ;

  (*(volatile uint32_t *)0x98100008) |= 0x4;

  (*(volatile uint32_t *)0x98800100) = 0xDFF8003F;

  /* Check board type */

  /* Mask all interrupts off */

  putreg32(0, IRQ_REG(IRQ__MASK));
  putreg32(0, IRQ_REG(IRQ__MASK+0x20));
  putreg32(0xffffffff, IRQ_REG(IRQ__CLEAR));
  putreg32(0xffffffff, IRQ_REG(IRQ__CLEAR+0x20));

  /* Initial trigger mode and level */

  putreg32(0, IRQ_REG(IRQ__MODE));
  putreg32(0, IRQ_REG(IRQ__LEVEL));
  putreg32(0, IRQ_REG(IRQ__MODE+0x20));
  putreg32(0, IRQ_REG(IRQ__LEVEL+0x20));

  /* currents_regs is non-NULL only while processing an interrupt */

  CURRENT_REGS = NULL;

  /* Setup UART shared interrupt */

  irq_attach(CONFIG_UART_MOXA_SHARED_IRQ, uart_decodeirq, NULL);
  up_enable_irq(CONFIG_UART_MOXA_SHARED_IRQ);

  /* And finally, enable interrupts */

  irqinfo("TM CNTL=%08x INTRS=%08x MASK=%08x LOAD=%08x COUNT=%08x M1=%08x\n",
          getreg32(0x98400030), getreg32(0x98400034), getreg32(0x98400038),
          getreg32(0x98400004), getreg32(0x98400000), getreg32(0x98400008));
  irqinfo("IRQ STATUS=%08x MASK=%08x MODE=%08x LEVEL=%08x\n",
          getreg32(0x98800014), getreg32(0x98800004), getreg32(0x9880000C),
          getreg32(0x98800010));
  irqinfo("FIQ STATUS=%08x MASK=%08x MODE=%08x LEVEL=%08x\n",
          getreg32(0x98800034), getreg32(0x98800024), getreg32(0x9880002C),
          getreg32(0x98800020));

#ifndef CONFIG_SUPPRESS_INTERRUPTS
  up_irq_restore(SVC_MODE | PSR_F_BIT);
#endif
}

inline void ftintc010_mask_irq(int irq)
{
  /* 0: masked
   * 1: unmasked
   */

  uint32_t mask;

  mask = getreg32(IRQ_REG(IRQ__MASK));
  mask &= ~(1 << irq);
  putreg32(mask, IRQ_REG(IRQ__MASK));
}

inline void ftintc010_unmask_irq(int irq)
{
  /* 0: masked
   * 1: unmasked
   */

  uint32_t mask;

  mask = getreg32(IRQ_REG(IRQ__MASK));
  mask |= 1 << irq;
  putreg32(mask, IRQ_REG(IRQ__MASK));
}

inline void ftintc010_set_trig_mode(int irq, int mode)
{
  uint32_t irqmode;

  irqmode = getreg32(IRQ_REG(IRQ__MODE));

  /* 0: level trigger
   * 1: edge trigger
   */

  if (mode)
    {
      irqmode |= (1 << irq);
    }
  else
    {
      irqmode &= ~(1 << irq);
    }

  putreg32(irqmode, IRQ_REG(IRQ__MODE));
}

inline void ftintc010_set_trig_level(int irq, int level)
{
  uint32_t irqlevel;

  irqlevel = getreg32(IRQ_REG(IRQ__LEVEL));

  /* 0: active-high level trigger / rising edge trigger
   * 1: active-low level trigger / falling edge trigger
   */

  if (level)
    {
      irqlevel |= (1 << irq);
    }
  else
    {
      irqlevel &= ~(1 << irq);
    }

  putreg32(irqlevel, IRQ_REG(IRQ__LEVEL));
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
  if ((unsigned)irq < NR_IRQS)
    {
      ftintc010_mask_irq(irq);
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
  if ((unsigned)irq < NR_IRQS)
    {
      ftintc010_unmask_irq(irq);
    }
}

static int ffs(uint32_t word)
{
  int t, r;

  if (word == 0)
    {
      return 0;
    }

  t = r = 1;

  while (!(word & t))
    {
      t <<= 1;
      r++;
    }

  return r;
}

/****************************************************************************
 * Name: arm_ack_irq
 *
 * Description:
 *   Acknowledge the interrupt
 *
 ****************************************************************************/

void arm_ack_irq(int irq)
{
  putreg32((1 << irq), IRQ_REG(IRQ__CLEAR));
}

/****************************************************************************
 * Entry point for interrupts
 ****************************************************************************/

uint32_t *arm_decodeirq(uint32_t *regs)
{
  uint32_t num, status;

  /* Detect & deliver the IRQ */

  status = getreg32(IRQ_REG(IRQ__STATUS));
  if (!status)
    {
      return NULL;
    }

  /* Ack IRQ */

  num = ffs(status) - 1;
  arm_ack_irq(num);

  DEBUGASSERT(CURRENT_REGS == NULL);
  CURRENT_REGS = regs;

  irq_dispatch(num, regs);
  CURRENT_REGS = NULL;

  return NULL;  /* Return not used in this architecture */
}
