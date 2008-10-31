/****************************************************************************
 * arch/arm/src/st71x/st71x_irq.c
 *
 *   Copyright (C) 2008 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
#include <sys/types.h>
#include <nuttx/irq.h>
#include "up_arch.h"
#include "os_internal.h"
#include "up_internal.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

uint32 *current_regs;

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Funtions
 ****************************************************************************/

/****************************************************************************
 * Name: up_irqinitialize
 ****************************************************************************/

void up_irqinitialize(void)
{
  uint32 reg32;

  /* The bulk of IRQ initialization if performed in str71x_head.S, so we
   * have very little to do here:
   */

  /* Enable IRQs (but not FIQs -- they aren't used) */

  putreg32(STR71X_EICICR_IRQEN, STR71X_EIC_ICR)

  /* Currents_regs is non-NULL only while processing an interrupt */

  current_regs = NULL;

  /* Enable interrupts */

#ifndef CONFIG_SUPPRESS_INTERRUPTS
  irqrestore(SVC_MODE | PSR_F_BIT);
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
  uint32 reg32;

  if ((unsigned)irq < NR_IRQS)
    {
      /* Mask the IRQ by clearing the associated bit in the IER register */

      reg32  = getreg32(STR71X_EIC_IER);
      reg32 &= ~(1 << irq);
      putreg32(reg32, STR71X_EIC_IER);
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
  uint32 reg32;

  if ((unsigned)irq < NR_IRQS)
    {
      /* Enable the IRQ by setting the associated bit in the IER register */

      reg32  = getreg32(STR71X_EIC_IER);
      reg32 |= (1 << irq);
      putreg32(reg32, STR71X_EIC_IER);
    }
}

/****************************************************************************
 * Name: up_maskack_irq
 *
 * Description:
 *   Mask the IRQ and acknowledge it
 *
 ****************************************************************************/

void up_maskack_irq(int irq)
{
  uint32 reg32;

  if ((unsigned)irq < NR_IRQS)
    {
      /* Mask the IRQ by clearing the associated bit in the IER register */

      reg32  = getreg32(STR71X_EIC_IER);
      reg32 &= ~(1 << irq);
      putreg32(reg32, STR71X_EIC_IER);

      /* Clear the interrupt by writing a one to the corresponding bit in the
       * IPR register.
       */

      reg32  = getreg32(STR71X_EIC_IPR);
      reg32 |= (1 << irq);
      putreg32(reg32, STR71X_EIC_IPR);

    }
}

/****************************************************************************
 * Name: up_irqpriority
 *
 * Description:
 *   set interrupt priority
 *
 ****************************************************************************/

int up_irqpriority(int irq, ubyte priority)
{
  uint32 reg32;

  if ((unsigned)irq < NR_IRQS && priority < 16)
    {
      uint32 addr = STR71X_EIC_SIR(irq);
      reg32  = getreg32(addr);
      reg32 &= STR71X_EICSIR_SIPLMASK;
      reg32 |= priority;
      putreg32(reg32, addr);
      return OK;
    }

  return -EINVAL;
}

